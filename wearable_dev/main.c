#include <stdint.h>
#include <string.h>

#include "boards.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "main.h"
#include "ble_app.h"
#include "cus_service.h"
#include "ecg.h"
#include "cmd.h"
#include "max.h"            /* max30102_init/read_samples/compute, timer2_init/now */
#include "mma845.h"         /* MMA8452Q_init(), MMA8452Q_read(), g_accel */
#include "tmp117_v2.h"      /* tmp117_Init(), tmp117_wake_oneshot(), tmp117_get_Temp() */
#include "temp_filter.h"    /* temp_filter_t, temp_filter_update() */
#include "pedometer.h"      /* pedometer_t, pedometer_update() */
#include "dashboard.h"      /* dashboard_data_t, dashboard_update_* */
#include "device_mode.h"

/* ================================================================
 *  Peripheral instances — declared extern in main.h
 * ================================================================ */
#define TWI_SCL_PIN  29
#define TWI_SDA_PIN  28

nrf_drv_twi_t    m_twi        = NRF_DRV_TWI_INSTANCE(1);
volatile bool    m_xfer_done  = false;
volatile bool    m_xfer_error = false;
nrf_drv_spi_t    m_lcd_spi    = NRF_DRV_SPI_INSTANCE(0);
sensor_data_t    g_sensor     = {0};

void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    if (p_event->type == NRF_DRV_TWI_EVT_ADDRESS_NACK ||
        p_event->type == NRF_DRV_TWI_EVT_DATA_NACK) {
        m_xfer_error = true;
    }
    m_xfer_done = true;
}

/* Spin-waits for the in-flight IRQ-driven transfer to complete.
 * Timeout ~5 ms at 64 MHz. Resets peripheral if IRQ never fires. */
bool twi_wait(void)
{
    uint32_t timeout = 200000;
    while (!m_xfer_done && --timeout);
    if (!timeout) {
        nrf_drv_twi_disable(&m_twi);
        nrf_drv_twi_enable(&m_twi);
    }
    bool ok = m_xfer_done && !m_xfer_error;
    m_xfer_done  = false;
    m_xfer_error = false;
    return ok;
}

void twi_init(void)
{
    const nrf_drv_twi_config_t cfg = {
        .scl                = TWI_SCL_PIN,
        .sda                = TWI_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };
    ret_code_t err = nrf_drv_twi_init(&m_twi, &cfg, twi_handler, NULL);
    APP_ERROR_CHECK(err);
    nrf_drv_twi_enable(&m_twi);
}

/* ================================================================
 *  BLE packet helpers
 * ================================================================ */
static int16_t  s_ecg_buf[ECG_BUF_SAMPLES];
static uint16_t s_ecg_idx = 0;

static void send_vitals_packet(void)
{
    /* 4×float32 LE = 16 bytes: [ecgHr][ppgHr][spo2][temp]
     * Dispatched by gateway on len==16; published as ecgHeartRate/ppgHeartRate/spo2/temperature */
    float pkt[4] = { g_sensor.hr_ecg, g_sensor.hr_ppg, g_sensor.spo2, g_sensor.temp };
    ble_app_send((uint8_t *)pkt, sizeof(pkt));
}

/* ================================================================
 *  Sensor state
 * ================================================================ */
static pedometer_t      g_pedometer;
static temp_filter_t    g_temp_filter;
static dashboard_data_t s_dash;
static uint16_t         s_ecg_display = 500;  /* ECG mapped to 0–999 for LCD */

/* Sensor presence — set at init, guards all tick reads */
static bool s_ppg_present   = false;
static bool s_accel_present = false;
static bool s_tmp_present   = false;

/* MAX30102 sample accumulation — filled at 100 Hz (1 sample / 10 ms tick) */
#define PPG_BUF_SIZE 100
static uint32_t s_ir_buf[PPG_BUF_SIZE];
static uint32_t s_red_buf[PPG_BUF_SIZE];
static int      s_ppg_count = 0;

/* TMP117 one-shot state machine */
static bool     s_tmp_triggered  = false;
static uint32_t s_tmp_trigger_ms = 0;

/* ================================================================
 *  Entry point
 * ================================================================ */
int main(void)
{
    log_init();
    timer_init();
    power_management_init();

    /* BLE / SoftDevice first — TWI IRQ must be registered after SD enables NVIC */
    ble_app_init();

    /* TWI after SoftDevice: SD resets NVIC on enable, so registering
     * twi_handler before ble_app_init() loses the IRQ registration */
    twi_init();

    /* ── Sensor init — non-fatal if hardware absent ── */
    s_ppg_present   = max30102_init();
    nrf_drv_twi_disable(&m_twi); nrf_drv_twi_enable(&m_twi);
    s_accel_present = MMA8452Q_init(0x1C, SCALE_2G, ODR_100);
    nrf_drv_twi_disable(&m_twi); nrf_drv_twi_enable(&m_twi);
    s_tmp_present   = tmp117_Init();

    if (!s_ppg_present)   NRF_LOG_WARNING("[HW] MAX30102 not found");
    if (!s_accel_present) NRF_LOG_WARNING("[HW] MMA8452Q not found");
    if (!s_tmp_present)   NRF_LOG_WARNING("[HW] TMP117 not found");

    if (s_accel_present) { mma8452q_alert_init(); }

    timer2_init();     /* free-running µs counter on TIMER2         */
    ecg_init();        /* SAADC + PPI → 250 Hz autonomous ECG       */

    pedometer_reset(&g_pedometer);
    temp_filter_reset(&g_temp_filter);
    memset(&s_dash, 0, sizeof(s_dash));


    ble_app_advertising_start();

    /* ── Main loop ─────────────────────────────────────────── */
    while (1)
    {
        uint32_t now_ms = timer2_now() / 1000;

        /* ── ECG sample ready (250 Hz, set by SAADC ISR) ── */
        if (g_ecg_ready)
        {
            g_ecg_ready = false;
            float filtered = ecg_process(g_ecg_raw);

            /* Scale to 0–999 for LCD waveform */
            int32_t ecg_disp = (int32_t)(g_ecg.filtered * 0.25f) + 500;
            if (ecg_disp < 0)   ecg_disp = 0;
            if (ecg_disp > 999) ecg_disp = 999;
            s_ecg_display = (uint16_t)ecg_disp;

            s_ecg_buf[s_ecg_idx++] = (int16_t)filtered;

            if (s_ecg_idx >= g_cmd_pkt_samples && ble_app_is_connected())
            {
                uint16_t total = g_cmd_pkt_samples;
                s_ecg_idx = 0;
                for (uint16_t off = 0; off < total; off += ECG_MAX_SAMPLES)
                {
                    uint16_t chunk = total - off;
                    if (chunk > ECG_MAX_SAMPLES) chunk = ECG_MAX_SAMPLES;
                    ret_code_t err = ble_app_send(
                        (uint8_t *)(s_ecg_buf + off), chunk * sizeof(int16_t));
                    if (err != NRF_SUCCESS             &&
                        err != NRF_ERROR_INVALID_STATE  &&
                        err != NRF_ERROR_RESOURCES      &&
                        err != NRF_ERROR_DATA_SIZE       &&
                        err != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                    {
                        APP_ERROR_HANDLER(err);
                    }
                }
            }
        }

        /* ── Sensor tick (10 ms CONTINUOUS / g_period_ms PERIODIC) ── */
        if (g_sensor_tick)
        {
            g_sensor_tick = false;
            now_ms = timer2_now() / 1000;

            /* ── MAX30102: read FIFO, compute when buffer full ── */
            if (s_ppg_present)
            {
                int n = max30102_read_samples(s_ir_buf + s_ppg_count,
                                              s_red_buf + s_ppg_count,
                                              PPG_BUF_SIZE - s_ppg_count);
                s_ppg_count += n;

                if (s_ppg_count >= PPG_BUF_SIZE)
                {
                    float hr = 0.0f, spo2 = 0.0f;
                    if (max30102_compute(s_ir_buf, s_red_buf, s_ppg_count, &hr, &spo2))
                    {
                        g_sensor.hr_ppg = (uint8_t)hr;
                        g_sensor.spo2   = (uint8_t)spo2;
                        s_dash.hr       = hr;
                        s_dash.spo2     = spo2;
                        dashboard_update_hr(&s_dash);
                    }
                    s_ppg_count = 0;
                }
            }

            /* ── Accelerometer → pedometer + LCD wake ── */
            if (s_accel_present)
            {
                MMA8452Q_read();
                pedometer_update(&g_pedometer, g_accel.ac, now_ms);

                g_sensor.steps   = pedometer_get_steps(&g_pedometer);
                g_sensor.cadence = pedometer_get_cadence(&g_pedometer, now_ms);

                s_dash.steps        = g_sensor.steps;
                s_dash.cadence      = g_sensor.cadence;
                s_dash.timestamp_ms = now_ms;
            }

            dashboard_update_steps(&s_dash, s_accel_present ? g_accel.ac : 0.0f);
            dashboard_update_ecg(&s_dash, s_ecg_display);

            /* ── TMP117 one-shot state machine ── */
            if (s_tmp_present)
            {
                if (!s_tmp_triggered)
                {
                    tmp117_wake_oneshot();
                    s_tmp_triggered  = true;
                    s_tmp_trigger_ms = now_ms;
                }
                else if ((now_ms - s_tmp_trigger_ms) >= 200)
                {
                    float raw_temp = tmp117_get_Temp();
                    tmp117_shutdown_mode();
                    s_tmp_triggered = false;

                    if (raw_temp > TMP117_TEMP_INVALID + 1.0f)
                    {
                        float ft = temp_filter_update(&g_temp_filter, raw_temp);
                        g_sensor.temp      = ft;
                        g_sensor.temp_valid = true;

                        s_dash.temperature = ft;
                        s_dash.temp_valid  = true;
                        dashboard_update_temp(&s_dash);
                    }
                }
            }

            if (ble_app_is_connected())
            {
                send_vitals_packet();
            }

            /* ── Status log every ~1 s (100 × 10 ms ticks) ── */
            static uint32_t s_log_tick = 0;
            if (++s_log_tick >= 100)
            {
                s_log_tick = 0;
                NRF_LOG_INFO("--- [STATUS] t=%u ms ---", now_ms);
                NRF_LOG_INFO("  BLE : %s",
                             ble_app_is_connected() ? "connected" : "advertising");
                NRF_LOG_INFO("  ECG : buf=%u/%u  hr=%u bpm",
                             s_ecg_idx, g_cmd_pkt_samples, g_sensor.hr_ecg);
                NRF_LOG_INFO("  PPG : hr=%u bpm  spo2=%u%%  buf=%u/%u",
                             g_sensor.hr_ppg, g_sensor.spo2,
                             s_ppg_count, PPG_BUF_SIZE);
                if (g_sensor.temp_valid)
                {
                    NRF_LOG_INFO("  TMP : " NRF_LOG_FLOAT_MARKER " C",
                                 NRF_LOG_FLOAT(g_sensor.temp));
                }
                else
                {
                    NRF_LOG_INFO("  TMP : no reading");
                }
                NRF_LOG_INFO("  HW  : ppg=%d accel=%d tmp=%d",
                             s_ppg_present, s_accel_present, s_tmp_present);
            }
        }

        /* ── Idle ── */
        if (!g_ecg_ready && !g_sensor_tick)
        {
            idle_state_handle();
        }
    }
}
