#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nrf_delay.h"
#include "GC9A01.h"

/* ================================================================
 *  Display mode select
 *    0 = NORMAL OPERATION — real sensors drive the dashboard (default)
 *    1 = TEST DISPLAY     — bypass all sensors; every dashboard element
 *                            (HR/SpO2/Temp/Steps/ECG/Battery/BLE) sweeps
 *                            through simulated values for UI testing
 * ================================================================ */
#define DASHBOARD_TEST_MODE  0

#include "boards.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "main.h"
#include "peripheral.h"
#include "ble_app.h"
#include "ecg.h"
#include "cmd.h"
#include "max.h"            /* max30102_init/read_samples/compute */
#include "mma845.h"         /* MMA8452Q_init(), MMA8452Q_read(), g_accel */
#include "tmp117_v2.h"      /* tmp117_Init(), tmp117_wake_oneshot(), tmp117_get_Temp() */
#include "temp_filter.h"    /* temp_filter_t, temp_filter_update() */
#include "pedometer.h"      /* pedometer_t, pedometer_update() */
#include "dashboard.h"      /* dashboard_data_t, dashboard_update_* */
#include "device_mode.h"
#include "filter.h"
#include "flash_user.h"

/* ================================================================
 *  Unified sensor data — peripheral instances/init now live in
 *  peripheral.c (TWI1, SPI0, SAADC/PPI/TIMER3, TIMER2).
 * ================================================================ */
sensor_data_t    g_sensor     = {0};

/* ================================================================
 *  BLE packet helpers
 * ================================================================ */
static int16_t  s_ecg_buf[ECG_BUF_SAMPLES];
static uint16_t s_ecg_idx = 0;

static void send_vitals_values(uint8_t hr_ecg, uint8_t hr_ppg, uint8_t spo2, float temp)
{
    /* 5 bytes: [hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10]
     * Dispatched by gateway on len==5; published as ecgHeartRate/ppgHeartRate/spo2/temperature */
    uint16_t temp_x10 = (uint16_t)(temp * 10.0f + 0.5f);
    uint8_t pkt[5] = {
        hr_ecg,
        hr_ppg,
        spo2,
        (uint8_t)(temp_x10 & 0xFF),
        (uint8_t)(temp_x10 >> 8),
    };
    ble_app_send(pkt, sizeof(pkt));
}

static void send_vitals_packet(void)
{
    /* 0 = no valid reading for that param → gateway omits the key (dashboard "__") */
    send_vitals_values(
        g_sensor.hr_ecg_valid ? g_sensor.hr_ecg : 0,
        g_sensor.hr_ppg_valid ? g_sensor.hr_ppg : 0,
        g_sensor.spo2_valid   ? g_sensor.spo2   : 0,
        g_sensor.temp_valid   ? g_sensor.temp   : 0.0f);
}

/* ── PERIODIC mode: per-capture-window averaging accumulators ── */
static uint32_t s_acc_hr_ppg = 0, s_acc_spo2 = 0, s_acc_n_ppg = 0;
static float    s_acc_temp   = 0.0f; static uint32_t s_acc_n_temp = 0;
static uint32_t s_acc_hr_ecg = 0, s_acc_n_ecg = 0;

static void periodic_acc_reset(void)
{
    s_acc_hr_ppg = s_acc_spo2 = s_acc_n_ppg = 0;
    s_acc_temp   = 0.0f; s_acc_n_temp = 0;
    s_acc_hr_ecg = s_acc_n_ecg = 0;
}

/* ================================================================
 *  Sensor state
 * ================================================================ */
static pedometer_t      g_pedometer;
static temp_filter_t    g_temp_filter;
static dashboard_data_t s_dash;
static uint16_t         s_ecg_display = 500;  /* ECG mapped to 0–999 for LCD */

/* Sensor presence — set at init, guards all tick reads */
static bool s_lcd_present   = false;
static bool s_ppg_present   = false;
static bool s_accel_present = false;
static bool s_tmp_present   = false;

/* MAX30102 HR/SpO2 is computed incrementally inside max30102_process(). */

/* TMP117 one-shot state machine */
static bool     s_tmp_triggered  = false;
static uint32_t s_tmp_trigger_ms = 0;

/* True whenever the MAX30102 is currently powered down (periodic sleep) */
static bool     s_ppg_shutdown   = false;

/* Mirrors g_ecg_stream_enabled — tracks whether the ECG SAADC/TIMER3 chain is running */
static bool     s_ecg_adc_enabled = true;

#define DEAD_BEEF                   0xDEADBEEF

/* ECG reconfiguration pending — written by cmd_rx_handle(), consumed here */
/* g_cmd_cfg_pending, g_cmd_sample_us, g_cmd_pkt_samples defined in cmd.c  */

static max30102_sr_t ppg_hz_to_sr(uint16_t hz)
{
    if (hz <= 50)   return max30102_sr_50;
    if (hz <= 100)  return max30102_sr_100;
    if (hz <= 200)  return max30102_sr_200;
    if (hz <= 400)  return max30102_sr_400;
    if (hz <= 800)  return max30102_sr_800;
    if (hz <= 1000) return max30102_sr_1000;
    if (hz <= 1600) return max30102_sr_1600;
    return max30102_sr_3200;
}

/* ---------- System helpers ---------- */
void log_init(void)
{
    ret_code_t err = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void timer_init(void)
{
    ret_code_t err = app_timer_init();
    APP_ERROR_CHECK(err);
}

void power_management_init(void)
{
    ret_code_t err = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err);
}

void idle_state_handle(void)
{
    if (!NRF_LOG_PROCESS()) { nrf_pwr_mgmt_run(); }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/* ================================================================
 *  Entry point
 * ================================================================ */
int main(void)
{
    log_init();
    timer_init();
    twi_init();
    power_management_init();
    ble_stack_init();   /* enables SoftDevice — required by app_timer & nrf_pwr_mgmt_run even in test mode */
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    /* FDS requires the SoftDevice (enabled by ble_stack_init above).
     * Loads persisted mode/period, ECG/PPG/vital config, and thresholds
     * (m_dummy_cfg holds defaults even if no record was found/loaded). */
    m_record_init();
    flash_user_load_config();


		timer2_init();
		ecg_init();
		if (!g_ecg_stream_enabled)
		{
			adc_disable();
			s_ecg_adc_enabled = false;
		}

    s_ppg_present   = max30102_init();
    nrf_drv_twi_disable(&m_twi); nrf_drv_twi_enable(&m_twi);
    s_accel_present = MMA8452Q_init(0x1C, SCALE_2G, ODR_100);
    nrf_drv_twi_disable(&m_twi); nrf_drv_twi_enable(&m_twi);
    s_tmp_present   = tmp117_Init();

    s_lcd_present = lcd_spi_init();
    if (s_lcd_present) {
        GC9A01_init();
        GC9A01_set_rotation(1);   /* 270° CW — physical mount orientation */
        dashboard_init_layout();
    }

    if (!s_lcd_present)   NRF_LOG_WARNING("[HW] GC9A01 LCD not found");
    if (!s_ppg_present)   NRF_LOG_WARNING("[HW] MAX30102 not found");
    if (!s_accel_present) NRF_LOG_WARNING("[HW] MMA8452Q not found");
    if (!s_tmp_present)   NRF_LOG_WARNING("[HW] TMP117 not found");

    if (s_accel_present) { mma8452q_alert_init(); }

    pedometer_reset(&g_pedometer);
    temp_filter_reset(&g_temp_filter);
    memset(&s_dash, 0, sizeof(s_dash));

    device_mode_init();   /* starts m_sensor_timer → drives g_sensor_tick */
		advertising_start();
		
    /* PPG DSP filters live inside drivers/ppg/max.c (max30102_process). */

    while (1)
    {
        uint32_t now_ms = timer2_now() / 1000;
			
       
        /* ── Sensor tick (10 ms CONTINUOUS / g_period_ms PERIODIC) ── */
				
					if (g_sensor_tick)
        {		
				
            g_sensor_tick = false;
            now_ms = timer2_now() / 1000;

#if DASHBOARD_TEST_MODE
            /* ── TEST DISPLAY MODE: bypass all sensors, sweep simulated
             *    values across every dashboard element ── */
            {
                float t = (float)now_ms / 1000.0f;

                float hr   = 80.0f  + 35.0f * sinf(t * 0.5f);   /* ~45-115 bpm   */
                float spo2 = 96.5f  + 2.5f  * sinf(t * 0.3f);   /* ~94-99 %      */
                float temp = 36.5f  + 1.2f  * sinf(t * 0.2f);   /* ~35.3-37.7 °C */

                g_sensor.hr_ppg       = (uint8_t)hr;
                g_sensor.spo2         = (uint8_t)spo2;
                g_sensor.hr_ppg_valid = true;
                g_sensor.spo2_valid   = true;
                g_sensor.temp         = temp;
                g_sensor.temp_valid   = true;

                s_dash.hr          = hr;
                s_dash.spo2        = spo2;
                s_dash.hr_valid    = true;
                s_dash.temperature = temp;
                s_dash.temp_valid  = true;

                s_dash.timestamp_ms = now_ms;

                s_dash.ecg_enabled  = true;
                s_dash.hr_ecg       = (uint8_t)hr;
                s_dash.hr_ecg_valid = true;
                uint16_t ecg_disp   = (uint16_t)(500.0f + 450.0f * sinf(t * 6.0f));

                s_dash.battery_valid = true;
                s_dash.battery_pct   = (uint8_t)(now_ms / 100) % 101;

                s_dash.ble_connected = ((now_ms / 5000) % 2) == 0;
                s_dash.rssi          = (int8_t)(-65.0f + 25.0f * sinf(t * 0.4f));  /* sweeps signal bars */
                uint8_t fake_mac[6]  = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};
                memcpy(s_dash.mac, fake_mac, sizeof(fake_mac));
                strncpy(s_dash.device_name, "TEST PATIENT", sizeof(s_dash.device_name) - 1);
                s_dash.device_name[sizeof(s_dash.device_name) - 1] = '\0';

                if (s_lcd_present)
                {
                    dashboard_update_hr(&s_dash);
                    dashboard_update_temp(&s_dash);
                    dashboard_update_ecg(&s_dash, ecg_disp);
                    dashboard_update_battery(&s_dash);
                    dashboard_update_ble_status(&s_dash);
                }
            }
#else
            /* ── Apply pending gateway config commands ── */
            if (g_cmd_cfg_pending)
            {
                g_cmd_cfg_pending = false;
                NRF_LOG_INFO("ECG cfg apply: sample_us=%u, pkt_samples=%u",
                             (unsigned)g_cmd_sample_us, (unsigned)g_cmd_pkt_samples);
                adc_set_sample_us(g_cmd_sample_us);
            }
            if (g_ppg_cfg_pending && s_ppg_present)
            {
                g_ppg_cfg_pending = false;
                NRF_LOG_INFO("PPG cfg apply: %u Hz hrSrc=%s",
                             (unsigned)g_ppg_sample_freq, g_ppg_hr_source ? "RED" : "IR");

                if (!s_ppg_shutdown)
                {
                    max30102_set_sampling_rate(ppg_hz_to_sr(g_ppg_sample_freq));
                    max30102_reset_filters((float)g_ppg_sample_freq);
                }
                device_mode_set_tick_ms(1000U / g_ppg_sample_freq);
            }
            if (g_vital_cfg_pending)
            {
                g_vital_cfg_pending = false;
                NRF_LOG_INFO("Vital cfg apply: interval=%u ms, lcd=%u ms", (unsigned)g_vital_interval_ms, (unsigned)g_lcd_interval_ms);
            }
            if (g_ecg_stream_enabled != s_ecg_adc_enabled)
            {
                s_ecg_adc_enabled = g_ecg_stream_enabled;
                if (s_ecg_adc_enabled)
                {
                    adc_enable();
                    g_ecg_ready = false;
                    s_ecg_idx   = 0;
                }
                else
                {
                    adc_disable();
                    g_ecg_ready = false;
                    g_sensor.hr_ecg_valid = false;
                }
                NRF_LOG_INFO("ECG SAADC %s", s_ecg_adc_enabled ? "enabled" : "disabled");
            }
            if (g_cmd_update_pending)
            {
                g_cmd_update_pending = false;
                if (s_lcd_present) {
                    char title[20], val[180];
                    strncpy(title, (const char *)g_cmd_update_msg, sizeof(title) - 1);
                    title[sizeof(title) - 1] = '\0';
                    strncpy(val, (const char *)g_cmd_update_val, sizeof(val) - 1);
                    val[sizeof(val) - 1] = '\0';
                    dashboard_show_update_splash(title, val);
                    dashboard_init_layout();
                }
            }

            /* MAX30102: read one sample, process incrementally */
            if (s_ppg_present && !s_ppg_shutdown)
            {
                uint32_t ir, red;
                float    hr = 0.0f, spo2 = 0.0f;
                bool     ppg_read = max30102_read_1_sample(&ir, &red);
                bool     ppg_fresh = ppg_read && max30102_process(ir, red, &hr, &spo2);

                if (ppg_read)
                {
                    /* Updated on every sample (not just once per UPDATE_PERIOD) so the
                     * LCD "SAT!" badge blinks smoothly. */
                    s_dash.ppg_saturated   = max30102_hr_saturated();
                    s_dash.ppg_calibrating = max30102_is_calibrating();
                    if (s_lcd_present && !g_periodic_sleeping) dashboard_update_sat_badge(&s_dash);
                }

                if (ppg_fresh)
                {
                    g_sensor.hr_ppg = (uint8_t)hr;
                    g_sensor.spo2   = (uint8_t)spo2;
                    g_sensor.hr_ppg_valid = (hr > 0.0f);   /* hr==0 → no finger detected */
                    g_sensor.spo2_valid   = (hr > 0.0f);
                    s_dash.hr       = hr;
                    s_dash.spo2     = spo2;
                    s_dash.hr_valid = (hr > 0.0f);   /* gates HR + SpO2 draw in dashboard_update_hr */
                    if (s_lcd_present && !g_periodic_sleeping) dashboard_update_hr(&s_dash);

                    /* hr==0 → no finger detected this sample (see max30102_process);
                     * exclude from the capture-window average so a missing finger
                     * doesn't drag the reported HR/SpO2 down toward 0. */
                    if (g_device_mode == MODE_PERIODIC && hr > 0.0f)
                    {
                        s_acc_hr_ppg += (uint8_t)hr;
                        s_acc_spo2   += (uint8_t)spo2;
                        s_acc_n_ppg++;
                    }
                }
            }
            /* ── Accelerometer → pedometer + LCD wake ── */
            if (s_accel_present)
            {
                MMA8452Q_read();
                pedometer_update(&g_pedometer, g_accel.ac, now_ms);

                g_sensor.steps   = pedometer_get_steps(&g_pedometer);
                g_sensor.cadence = pedometer_get_cadence(&g_pedometer, now_ms);

                s_dash.timestamp_ms = now_ms;
            }
							
            if (s_lcd_present && !g_periodic_sleeping) {
                s_dash.ecg_enabled  = g_ecg_stream_enabled;   /* V4: ECG ON/OFF badge + sweep gating */
                s_dash.hr_ecg       = g_sensor.hr_ecg;
                s_dash.hr_ecg_valid = g_ecg_stream_enabled && g_sensor.hr_ecg_valid;
                dashboard_update_ecg(&s_dash, s_ecg_display);
            }
					
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
                        g_sensor.temp       = ft;
                        g_sensor.temp_valid = true;

                        s_dash.temperature = ft;
                        s_dash.temp_valid  = true;
                        if (s_lcd_present && !g_periodic_sleeping) dashboard_update_temp(&s_dash);

                        if (g_device_mode == MODE_PERIODIC)
                        {
                            s_acc_temp += ft;
                            s_acc_n_temp++;
                        }
                    }
                }
            }
			
            if (g_device_mode == MODE_PERIODIC)
            {
                /* Snapshot ECG HR each capture tick (this block only runs while capturing) */
                if (g_ecg_stream_enabled && g_sensor.hr_ecg_valid)
                {
                    s_acc_hr_ecg += g_sensor.hr_ecg;
                    s_acc_n_ecg++;
                }

                /* End of capture window → emit one vital averaged over the window.
                 * No valid samples in the window → 0 → gateway omits the key ("__"). */
                if (g_periodic_send_due)
                {
                    g_periodic_send_due = false;

                    uint8_t hp = s_acc_n_ppg  ? (uint8_t)(s_acc_hr_ppg / s_acc_n_ppg) : 0;
                    uint8_t sp = s_acc_n_ppg  ? (uint8_t)(s_acc_spo2   / s_acc_n_ppg) : 0;
                    uint8_t he = s_acc_n_ecg  ? (uint8_t)(s_acc_hr_ecg / s_acc_n_ecg) : 0;
                    float   tc = s_acc_n_temp ? (s_acc_temp / (float)s_acc_n_temp)    : 0.0f;

                    if (ble_app_ready_to_send())
                    {
                        send_vitals_values(he, hp, sp, tc);
                    }
                    periodic_acc_reset();
                }
            }
            else
            {
                static uint32_t s_ble_tick = 0;

                if (++s_ble_tick >= (uint32_t)(g_vital_interval_ms / device_mode_get_tick_ms()))
                {
                    s_ble_tick = 0;

                    if (ble_app_ready_to_send())
                    {
                        send_vitals_packet();
                    }
                }
            }

            /* ── Status log every ~1 s (100 × 10 ms ticks) ── */
            static uint32_t s_log_tick = 0;
						
						
            if (++s_log_tick >= (1000U / device_mode_get_tick_ms()))
            {
                s_log_tick = 0;
                NRF_LOG_INFO("--- [STATUS] t=%u ms ---", now_ms);
                NRF_LOG_INFO("  BLE : %s",
                             ble_app_is_connected() ? "connected" : "advertising");
                NRF_LOG_INFO("  ECG : buf=%u/%u  hr=%u bpm",
                             s_ecg_idx, g_cmd_pkt_samples, g_sensor.hr_ecg);
                NRF_LOG_INFO("  PPG : hr=%u bpm  spo2=%u%%",
                             g_sensor.hr_ppg, g_sensor.spo2);
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

                /* ── Persist mode/params/thresholds if changed since last save ── */
                if (m_fds_initialized && flash_user_config_dirty())
                {
                    flash_user_save_config();
                }
            }
#endif /* DASHBOARD_TEST_MODE */
        }

        if (g_periodic_enter_sleep)
        {
            g_periodic_enter_sleep = false;
            if (s_ppg_present && !s_ppg_shutdown)
            {
                max30102_shutdown();
                s_ppg_shutdown = true;
            }
            if (s_lcd_present)
            {
                GC9A01A_sleep_mode(1);   /* SLPIN — panel low-power sleep */
							
            }
        }

        if (g_periodic_enter_capture)
        {
            g_periodic_enter_capture = false;
            if (s_ppg_present && s_ppg_shutdown)
            {
                max30102_wakeup();
                max30102_reset_filters((float)g_ppg_sample_freq);
                s_ppg_shutdown = false;
            }
            if (s_lcd_present)
            {
                GC9A01A_sleep_mode(0);   /* SLPOUT — wake panel from sleep */
						
            }
        }

        /* ── LCD vitals refresh — push latest HR / SpO2 / Temp at
         *    g_lcd_interval_ms (default 1000 ms, configurable via
         *    CMD_VITAL_CFG bytes 4-5), independent of the BLE vitals send
         *    cadence (g_vital_interval_ms) and of when individual sensor
         *    readings arrive. Shows "--" for any vital with no valid
         *    reading. ── */
        if (s_lcd_present && !g_periodic_sleeping)
        {
            static uint32_t s_lcd_vitals_ms = 0;
            if (now_ms - s_lcd_vitals_ms >= g_lcd_interval_ms)
            {
                s_lcd_vitals_ms = now_ms;

                s_dash.hr          = g_sensor.hr_ppg;
                s_dash.spo2        = g_sensor.spo2;
                s_dash.hr_valid    = g_sensor.hr_ppg_valid;   /* gates HR + SpO2 draw */
                s_dash.temperature = g_sensor.temp;
                s_dash.temp_valid  = g_sensor.temp_valid;

                dashboard_update_hr(&s_dash);
                dashboard_update_temp(&s_dash);

                /* ── Row 1: BLE status (address/patient name, RSSI, signal bars) ── */
                s_dash.ble_connected = ble_app_is_connected();
                s_dash.rssi          = s_dash.ble_connected ? ble_app_get_rssi() : 0;
                ble_app_get_addr(s_dash.mac);
                if (s_dash.ble_connected && g_patient_name[0] != '\0') {
                    strncpy(s_dash.device_name, (const char *)g_patient_name,
                            sizeof(s_dash.device_name) - 1);
                    s_dash.device_name[sizeof(s_dash.device_name) - 1] = '\0';
                } else {
                    s_dash.device_name[0] = '\0';   /* show address */
                }
                dashboard_update_ble_status(&s_dash);
            }
        }

        /* ── ECG sample ready (250 Hz, set by SAADC ISR) ── */
        if (g_ecg_ready)
        {
            g_ecg_ready = false;

            if (g_ecg_raw < 1000)
            {
                /* Electrode likely disconnected — ADC reading too low to be a
                 * valid ECG sample. Skip HR calc, sweep update, and buffering. */
                g_sensor.hr_ecg_valid = false;
            }
            else
            {
            float filtered = ecg_process(g_ecg_raw);

            /* Scale to 0–999 for LCD waveform */
            int32_t ecg_disp = (int32_t)(g_ecg.filtered * 0.25f) + 500;
            if (ecg_disp < 0)   ecg_disp = 0;
            if (ecg_disp > 999) ecg_disp = 999;
            s_ecg_display = (uint16_t)ecg_disp;

            s_ecg_buf[s_ecg_idx++] = (int16_t)filtered;

            if (s_ecg_idx >= g_cmd_pkt_samples)
            {
                uint16_t total = s_ecg_idx;
                s_ecg_idx = 0;                 /* always drain to avoid overflow */
                if (g_ecg_stream_enabled && ble_app_ready_to_send())
                {
                for (uint16_t off = 0; off < total; off += ECG_MAX_SAMPLES)
                {
                    uint16_t chunk = total - off;
                    if (chunk > ECG_MAX_SAMPLES) chunk = ECG_MAX_SAMPLES;
                    ret_code_t err = ble_app_send(
                        (uint8_t *)(s_ecg_buf + off), chunk * sizeof(int16_t));
                    if (err != NRF_SUCCESS              &&
                        err != NRF_ERROR_INVALID_STATE  &&
                        err != NRF_ERROR_RESOURCES      &&
                        err != NRF_ERROR_BUSY           &&
                        err != NRF_ERROR_DATA_SIZE      &&
                        err != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                    {
                        APP_ERROR_HANDLER(err);
                    }
                }
                }
            }
            }
        }

        /* ── Idle ── */
        if (!g_ecg_ready && !g_sensor_tick)
        {
						
            idle_state_handle();
        }
    }
}
