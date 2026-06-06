#include "device_mode.h"

#include "app_timer.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ble_app.h"

/* ══════════════════════════════════════════════════════════════
 *  State
 * ════════════════════════════════════════════════════════════ */
device_mode_t    g_device_mode  = MODE_CONTINUOUS;
uint16_t         g_period_ms    = 10000;    /* 10 s default */
volatile bool    g_sensor_tick  = false;

/* ── Sensor polling timer ── */
APP_TIMER_DEF(m_sensor_timer);

/* Tick counter for PERIODIC mode: fires every 10 ms, counts to period */
static uint32_t s_tick_count   = 0;
static uint32_t s_tick_target  = 1;     /* ticks before a sensor read */

#define SENSOR_TIMER_MS   10            /* base tick = 10 ms */

static void sensor_timer_cb(void *ctx)
{
    (void)ctx;
    if (g_device_mode == MODE_ECG) { return; }  /* ECG mode: SAADC drives itself */

    s_tick_count++;
    if (s_tick_count >= s_tick_target)
    {
        s_tick_count  = 0;
        g_sensor_tick = true;
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
    /* MODE_CONTINUOUS: fire every 10 ms tick.
     * PERIODIC: fire every g_period_ms. Gateway updates mode/period via BLE. */
    if (g_device_mode == MODE_CONTINUOUS)
    {
        s_tick_target = 1;
    }
    else
    {
        s_tick_target = (g_period_ms + SENSOR_TIMER_MS - 1) / SENSOR_TIMER_MS;
        if (s_tick_target < 1) { s_tick_target = 1; }
    }

    /* Start base timer (10 ms, repeating) */
    APP_ERROR_CHECK(app_timer_create(&m_sensor_timer, APP_TIMER_MODE_REPEATED, sensor_timer_cb));
    APP_ERROR_CHECK(app_timer_start(m_sensor_timer, APP_TIMER_TICKS(SENSOR_TIMER_MS), NULL));

    apply_ble_interval(g_device_mode);

    NRF_LOG_INFO("device_mode_init: mode=%d period=%d ms", g_device_mode, g_period_ms);
    NRF_LOG_FLUSH();
}

void device_mode_set(device_mode_t mode)
{
    if (mode >= MODE_COUNT) { return; }
    g_device_mode = mode;

    /* Update polling period */
    if (mode == MODE_CONTINUOUS)
    {
        s_tick_target = 1;              /* fire every 10 ms tick */
    }
    else if (mode == MODE_PERIODIC)
    {
        s_tick_target = (g_period_ms + SENSOR_TIMER_MS - 1) / SENSOR_TIMER_MS;
    }
    /* MODE_ECG: timer keeps running but sensor_timer_cb returns early */

    apply_ble_interval(mode);

    NRF_LOG_INFO("Mode → %d", mode);
    NRF_LOG_FLUSH();
}

void device_mode_set_period(uint16_t seconds)
{
    if (seconds == 0) { seconds = 1; }
    g_period_ms   = (uint16_t)((uint32_t)seconds * 1000 > 65535
                    ? 65535 : (uint32_t)seconds * 1000);
    s_tick_target = (g_period_ms + SENSOR_TIMER_MS - 1) / SENSOR_TIMER_MS;
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
