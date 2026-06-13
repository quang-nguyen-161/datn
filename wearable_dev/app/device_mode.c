#include "device_mode.h"

#include "app_timer.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "main.h"
#include "ble_app.h"
#include "flash_user.h"

/* ══════════════════════════════════════════════════════════════
 *  State
 * ════════════════════════════════════════════════════════════ */
device_mode_t    g_device_mode       = MODE_CONTINUOUS;
uint16_t         g_period_ms         = 10000;   /* 10 s default cycle interval */
uint16_t         g_capture_ms        = 5000;    /* 5 s default measurement window */
volatile bool    g_sensor_tick       = false;
volatile bool    g_periodic_send_due = false;

/* ── Sensor polling timer ── */
APP_TIMER_DEF(m_sensor_timer);

#define SENSOR_TIMER_MS   10            /* base tick = 10 ms */

/* PERIODIC duty cycle: capture for s_capture_ticks (full 10 ms rate), then
 * idle for s_sleep_ticks (no sensor reads), repeating. */
static bool     s_in_capture    = true;   /* PERIODIC starts by capturing */
static uint32_t s_phase_ticks   = 0;
static uint32_t s_capture_ticks = 1;      /* g_capture_ms / 10                 */
static uint32_t s_sleep_ticks   = 0;      /* (g_period_ms - g_capture_ms) / 10 */

/* Derive the capture/sleep tick counts from g_period_ms / g_capture_ms.
 * Clamps capture ≤ period; if cycle ≤ capture, sleep = 0 (never sleeps). */
static void periodic_recompute(void)
{
    uint32_t cap = (g_capture_ms + SENSOR_TIMER_MS - 1) / SENSOR_TIMER_MS;
    uint32_t cyc = (g_period_ms  + SENSOR_TIMER_MS - 1) / SENSOR_TIMER_MS;
    if (cap < 1)   { cap = 1; }
    if (cap > cyc) { cap = cyc; }
    s_capture_ticks = cap;
    s_sleep_ticks   = (cyc > cap) ? (cyc - cap) : 0;
}

static void sensor_timer_cb(void *ctx)
{
    (void)ctx;

    if (g_device_mode == MODE_ECG) { return; }  /* ECG mode: SAADC drives itself */

    if (g_device_mode == MODE_CONTINUOUS)
    {
        g_sensor_tick = true;                   /* sample every 10 ms tick */
        return;
    }

    /* MODE_PERIODIC: wake → capture → sleep duty cycle */
    if (s_in_capture)
    {
        g_sensor_tick = true;                   /* full 10 ms rate while capturing */
        if (++s_phase_ticks >= s_capture_ticks)
        {
            s_in_capture        = false;
            s_phase_ticks       = 0;
            g_periodic_send_due = true;         /* emit one averaged vital this cycle */
        }
    }
    else if (s_sleep_ticks)                     /* idle: never set g_sensor_tick */
    {
        if (++s_phase_ticks >= s_sleep_ticks)
        {
            s_in_capture  = true;
            s_phase_ticks = 0;
        }
    }
}

/* ══════════════════════════════════════════════════════════════
 *  Helpers
 * ════════════════════════════════════════════════════════════ */
static void apply_ble_interval(device_mode_t mode)
{
    if (mode == MODE_ECG)
    {
        ble_app_set_conn_interval(20, 50);      /* fast: 20–50 ms for ECG stream */
    }
    else
    {
        ble_app_set_conn_interval(100, 200);    /* power-friendly for vitals */
    }
}

/* ══════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════ */
void device_mode_init(void)
{
    /* CONTINUOUS fires every 10 ms tick; PERIODIC runs the capture/sleep duty
     * cycle. Gateway updates mode/period/capture via CMD_MODE_CFG. */
    periodic_recompute();
    s_in_capture  = true;
    s_phase_ticks = 0;

    /* Start base timer (10 ms, repeating) */
    APP_ERROR_CHECK(app_timer_create(&m_sensor_timer, APP_TIMER_MODE_REPEATED, sensor_timer_cb));
    APP_ERROR_CHECK(app_timer_start(m_sensor_timer, APP_TIMER_TICKS(SENSOR_TIMER_MS), NULL));

#if !SENSOR_TEST_MODE
    apply_ble_interval(g_device_mode);
#endif

    NRF_LOG_INFO("device_mode_init: mode=%d period=%d ms capture=%d ms",
                 g_device_mode, g_period_ms, g_capture_ms);
    NRF_LOG_FLUSH();
}

void device_mode_set(device_mode_t mode)
{
    if (mode >= MODE_COUNT) { return; }
    g_device_mode = mode;

    if (mode == MODE_PERIODIC)
    {
        /* Begin capturing immediately on entry */
        s_in_capture        = true;
        s_phase_ticks       = 0;
        g_periodic_send_due = false;
    }
    periodic_recompute();
    /* MODE_CONTINUOUS / MODE_ECG: handled directly in sensor_timer_cb */

    apply_ble_interval(mode);
    flash_user_mark_dirty();

    NRF_LOG_INFO("Mode → %d", mode);
    NRF_LOG_FLUSH();
}

void device_mode_set_period(uint16_t seconds)
{
    if (seconds < 5)  { seconds = 5;  }   /* PERIODIC interval: 5 s … 60 s */
    if (seconds > 60) { seconds = 60; }
    g_period_ms = (uint16_t)((uint32_t)seconds * 1000);
    if (g_capture_ms > g_period_ms) { g_capture_ms = g_period_ms; }
    periodic_recompute();
    flash_user_mark_dirty();
    NRF_LOG_INFO("device_mode_set_period: period=%u ms, capture=%u ms", g_period_ms, g_capture_ms);
}

void device_mode_set_capture(uint16_t seconds)
{
    uint16_t period_s = (uint16_t)(g_period_ms / 1000);
    if (seconds < 5)        { seconds = 5;        }   /* capture window: ≥5 s, ≤ interval */
    if (seconds > period_s) { seconds = period_s; }
    g_capture_ms = (uint16_t)((uint32_t)seconds * 1000);
    periodic_recompute();
    flash_user_mark_dirty();
    NRF_LOG_INFO("device_mode_set_capture: capture=%u ms", g_capture_ms);
}

uint16_t device_mode_get_period(void)
{
    return g_period_ms;
}

/* ══════════════════════════════════════════════════════════════
 *  BLE RX command parser
 * ════════════════════════════════════════════════════════════ */
void device_mode_on_ble_rx(uint8_t const *data, uint16_t len)
{
    if (len < 1) { return; }

    switch (data[0])
    {
        case 0x01:  /* set mode */
            if (len >= 2 && data[1] < MODE_COUNT)
            {
                device_mode_set((device_mode_t)data[1]);
            }
            break;

        case 0x02:  /* set period (bytes 1-2 = seconds, big-endian) */
            if (len >= 3)
            {
                uint16_t secs = ((uint16_t)data[1] << 8) | data[2];
                device_mode_set_period(secs);
            }
            break;

        case 0x03:  /* status request — reply with 0x12 packet */
        {
            uint8_t reply[4];
            reply[0] = 0x12;                        /* TX type: status */
            reply[1] = (uint8_t)g_device_mode;
            reply[2] = (uint8_t)(g_period_ms >> 8);
            reply[3] = (uint8_t)(g_period_ms & 0xFF);
            ble_app_send(reply, sizeof(reply));
            break;
        }

        default:
            break;
    }
}
