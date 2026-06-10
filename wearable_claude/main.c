#include <stdint.h>
#include <string.h>

#include "boards.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nordic_common.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "main.h"
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
static bool m_notify_enabled = false;
nrf_drv_spi_t    m_lcd_spi    = NRF_DRV_SPI_INSTANCE(0);
sensor_data_t    g_sensor     = {0};

/* ================================================================
 *  ECG ADC capture — read out here, hardware armed by ecg_init()
 *  (saadc_init/ppi_init in ecg.c register this callback; it fires
 *  every 4 ms (250 Hz) and just stashes the raw sample for the
 *  main loop, which calls ecg_process() to run the DSP chain).
 * ================================================================ */
volatile int16_t g_ecg_raw   = 0;
volatile bool    g_ecg_ready = false;

void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type != NRF_DRV_SAADC_EVT_DONE) { return; }

    g_ecg_raw   = p_event->data.done.p_buffer[0];
    g_ecg_ready = true;

    /* Re-queue the same buffer; matches SAMPLES_IN_BUFFER (1) in ecg.c */
    ret_code_t err = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
    APP_ERROR_CHECK(err);
}

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
    float pkt[4] = {
        g_sensor.hr_ecg,
        g_sensor.hr_ppg,
        g_sensor.spo2,
        g_sensor.temp
    };

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

/* ------------------------------------------------------------------ */
/*  BLE configuration                                                  */
/* ------------------------------------------------------------------ */
#define APP_BLE_CONN_CFG_TAG        1
#define DEVICE_NAME                 "ECG_dev"
#define NUS_SERVICE_UUID_TYPE       BLE_UUID_TYPE_VENDOR_BEGIN
#define APP_BLE_OBSERVER_PRIO       3
#define APP_ADV_INTERVAL            64          /* 40 ms */
#define APP_ADV_DURATION            18000       /* 180 s  */
#define MIN_CONN_INTERVAL           MSEC_TO_UNITS(20,  UNIT_1_25_MS)
#define MAX_CONN_INTERVAL           MSEC_TO_UNITS(75,  UNIT_1_25_MS)
#define SLAVE_LATENCY               0
#define CONN_SUP_TIMEOUT            MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3
#define DEAD_BEEF                   0xDEADBEEF

#define PACKET_SAMPLES_DEFAULT   50U     /* 250 Hz x 200 ms */
#define PACKET_SAMPLES_MAX       128U    /* hard cap: 256 bytes < any negotiated MTU */

/* ECG reconfiguration pending — written by cmd_rx_handle(), consumed here */
/* g_cmd_cfg_pending, g_cmd_sample_us, g_cmd_pkt_samples defined in cmd.c  */

/* ------------------------------------------------------------------ */
/*  BLE instances                                                       */
/* ------------------------------------------------------------------ */
BLE_CUS_DEF(m_cus);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

static uint16_t   m_conn_handle        = BLE_CONN_HANDLE_INVALID;
static uint16_t   m_ble_max_data_len   = BLE_GATT_ATT_MTU_DEFAULT - 3;
static ble_uuid_t m_adv_uuids[]        = { {CUS_SERVICE_UUID, NUS_SERVICE_UUID_TYPE} };

/* ---------- GAP ---------- */
void gap_params_init(void)
{
    ble_gap_conn_params_t   p;
    ble_gap_conn_sec_mode_t sec;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec);
    sd_ble_gap_device_name_set(&sec, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));

    memset(&p, 0, sizeof(p));
    p.min_conn_interval = MIN_CONN_INTERVAL;
    p.max_conn_interval = MAX_CONN_INTERVAL;
    p.slave_latency     = SLAVE_LATENCY;
    p.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    sd_ble_gap_ppcp_set(&p);
}

/* ---------- QWR error ---------- */
void nrf_qwr_error_handler(uint32_t nrf_error)  { APP_ERROR_HANDLER(nrf_error); }

/* ---------- CUS data handler ---------- */
void cus_data_handler(ble_cus_evt_t * p_evt)
{
    if (p_evt->type == BLE_CUS_EVT_NOTIFY_ENABLE)
    {
				m_notify_enabled = true;
        NRF_LOG_INFO("Notify ON");
    }
    else if (p_evt->type == BLE_CUS_EVT_NOTIFY_DISABLE)
    {
				 m_notify_enabled = false;
        NRF_LOG_INFO("Notify OFF");
    }
    else if (p_evt->type == BLE_CUS_EVT_RX_DATA)
    {
        cmd_rx_handle(p_evt->params.rx_data.p_data,
                      p_evt->params.rx_data.length,
                      PACKET_SAMPLES_MAX);
    }
}

/* ---------- Services ---------- */
void services_init(void)
{
    nrf_ble_qwr_init_t qwr = {0};
    qwr.error_handler = nrf_qwr_error_handler;
    nrf_ble_qwr_init(&m_qwr, &qwr);

    ble_cus_init_t cus = {0};
    cus.data_handler = cus_data_handler;
    ble_cus_init(&m_cus, &cus);
}

/* ---------- Conn params ---------- */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
}
static void conn_params_error_handler(uint32_t e) { APP_ERROR_HANDLER(e); }

void conn_params_init(void)
{
    ble_conn_params_init_t cp = {0};
    cp.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp.disconnect_on_fail             = false;
    cp.evt_handler                    = on_conn_params_evt;
    cp.error_handler                  = conn_params_error_handler;
    ret_code_t err = ble_conn_params_init(&cp);
    APP_ERROR_CHECK(err);
}

/* ---------- BLE events ---------- */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,
                                    p_ble_evt->evt.gap_evt.conn_handle, 0);
            nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
						m_notify_enabled = false;
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            ble_gap_phys_t phys = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
            sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            sd_ble_gap_sec_params_reply(m_conn_handle,
                BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        default: break;
    }
}

void ble_stack_init(void)
{
    nrf_sdh_enable_request();
    uint32_t ram_start = 0;
    nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    nrf_sdh_ble_enable(&ram_start);
    NRF_SDH_BLE_OBSERVER(m_ble_obs, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/* ---------- GATT ---------- */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) &&
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}

void gatt_init(void)
{
    nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
}

/* ---------- Advertising ---------- */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    if (ble_adv_evt == BLE_ADV_EVT_IDLE)
    {
        /* Restart advertising instead of sleeping � keeps ECG running */
        ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    }
}

void advertising_init(void)
{
    ble_advertising_init_t init = {0};
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = ARRAY_SIZE(m_adv_uuids);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_fast_enabled    = true;
    init.config.ble_adv_fast_interval   = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout    = APP_ADV_DURATION;
    init.evt_handler                    = on_adv_evt;
    ble_advertising_init(&m_advertising, &init);
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

void advertising_start(void)
{
    ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    NRF_LOG_INFO("Advertising started");
}

/* ---------- Thin accessors over the BLE globals above (used by main loop / device_mode) ---------- */
uint16_t ble_app_conn_handle(void)  { return m_conn_handle; }
bool     ble_app_is_connected(void) { return m_conn_handle != BLE_CONN_HANDLE_INVALID; }

void ble_app_set_conn_interval(uint16_t min_ms, uint16_t max_ms)
{
    ble_gap_conn_params_t params = {
        .min_conn_interval = MSEC_TO_UNITS(min_ms, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(max_ms, UNIT_1_25_MS),
        .slave_latency     = SLAVE_LATENCY,
        .conn_sup_timeout  = CONN_SUP_TIMEOUT,
    };
    sd_ble_gap_ppcp_set(&params);
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        sd_ble_gap_conn_param_update(m_conn_handle, &params);
    }
}

uint32_t ble_app_send(uint8_t const *data, uint16_t len)
{
     if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
        return NRF_ERROR_INVALID_STATE;

    if (len > m_ble_max_data_len)                 { return NRF_ERROR_DATA_SIZE; }
    uint16_t l = len;
    return ble_cus_data_send(&m_cus, (uint8_t *)data, &l, m_conn_handle);
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
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
		
		advertising_start();
		
				
		timer2_init();
		ecg_init();

		nrf_gpio_cfg_output(3);
		nrf_gpio_pin_set(3);
		
		/*
     ── Sensor init — non-fatal if hardware absent  
    s_ppg_present   = max30102_init();
    nrf_drv_twi_disable(&m_twi); nrf_drv_twi_enable(&m_twi);
    s_accel_present = MMA8452Q_init(0x1C, SCALE_2G, ODR_100);
    nrf_drv_twi_disable(&m_twi); nrf_drv_twi_enable(&m_twi);
    s_tmp_present   = tmp117_Init();

    if (!s_ppg_present)   NRF_LOG_WARNING("[HW] MAX30102 not found");
    if (!s_accel_present) NRF_LOG_WARNING("[HW] MMA8452Q not found");
    if (!s_tmp_present)   NRF_LOG_WARNING("[HW] TMP117 not found");

    if (s_accel_present) { mma8452q_alert_init(); }

    pedometer_reset(&g_pedometer);
    temp_filter_reset(&g_temp_filter);
    memset(&s_dash, 0, sizeof(s_dash));

		
   

     ── Main loop ─────────────────────────────────────────── */
		 
    while (1)
    {
				NRF_LOG_PROCESS();
				
        uint32_t now_ms = timer2_now() / 1000;

        /* ── ECG sample ready (250 Hz, set by SAADC ISR) ── */
        if (g_ecg_ready)
        {
            g_ecg_ready = false;
            float filtered = ecg_process(g_ecg_raw);
		//			NRF_LOG_INFO("raw, %d,filt: %d\n",g_ecg_raw,(int)(filtered *100.0f));
		//			NRF_LOG_FLUSH();
            /* Scale to 0–999 for LCD waveform */
            int32_t ecg_disp = (int32_t)(g_ecg.filtered * 0.25f) + 500;
            if (ecg_disp < 0)   ecg_disp = 0;
            if (ecg_disp > 999) ecg_disp = 999;
            s_ecg_display = (uint16_t)ecg_disp;

            s_ecg_buf[s_ecg_idx++] = (int16_t)filtered;

            if (s_ecg_idx >= g_cmd_pkt_samples && (ble_app_is_connected() && m_notify_enabled))
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

            if (ble_app_is_connected() && m_notify_enabled)
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
