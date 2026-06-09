/**
 * @file    main_ecg_ble_ppi.c
 * @brief   ECG acquisition on nRF52832 � SAADC driven by PPI+TIMER (no CPU polling)
 *          with BLE notification of filtered samples.
 *
 * Root cause of "no ECG on nRF but works on ESP32":
 *   1. Polling blocked timing: busy-wait in get_ecg_polling() distorts 250 Hz cadence.
 *   2. saadc_sampling_event_init / enable were commented out ? PPI never armed.
 *   3. TIMER1 conflict: APP_PWM_INSTANCE(PWM1,1) and old m_timer both claimed TIMER1.
 *      Fix: SAADC PPI now uses TIMER3.
 *   4. ecg * 0.01f scale is fine for DC-remove but verify your AD8232 bias is
 *      inside [0 � 3.6 V] (GAIN1_6 + internal 0.6 V ref).
 *
 * Signal chain (250 Hz):
 *   raw ADC  ?  �scale  ?  dc_remove  ?  notch50  ?  lpf2_ecg (�2 biquad)  ?  BLE notify
 */

/* ------------------------------------------------------------------ */
/*  Includes                                                           */
/* ------------------------------------------------------------------ */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_ppi.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* BLE */
#include "nordic_common.h"
#include "nrf.h"
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
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "cus_service.h"

/* DSP / drivers */
#include "filter.h"
#include "dashboard.h"   /* timer2_init(), timer2_now() */
#include "max.h"
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

/* ------------------------------------------------------------------ */
/*  Hardware                                                           */
/* ------------------------------------------------------------------ */
#define TWI_SCL_PIN     29
#define TWI_SDA_PIN     28

/* ------------------------------------------------------------------ */
/*  SAADC / PPI                                                        */
/* ------------------------------------------------------------------ */
#define SAMPLES_IN_BUFFER   1
#define SAADC_SAMPLE_US     4000    /* 250 Hz */

/*  Use TIMER3 � TIMER0 reserved by SD, TIMER1 used by PWM, TIMER2 by timer2_now() */
static const nrf_drv_timer_t   m_saadc_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t       m_buf[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_chan;

/* ------------------------------------------------------------------ */
/*  ECG packet buffering for BLE                                       */
/* ------------------------------------------------------------------ */
/* 50 samples × 2 bytes = 100-byte payload; at 250 Hz → one notify every 200 ms */
#define PACKET_SAMPLES  50
static int16_t  s_send_buf[PACKET_SAMPLES];
static uint8_t  s_send_idx = 0;

/* Shared between ISR and main � use volatile */
volatile int16_t  g_ecg_raw      = 0;
volatile float    g_ecg_filtered = 0.0f;
volatile bool     g_ecg_ready    = false;

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

/* ------------------------------------------------------------------ */
/*  DSP state                                                          */
/* ------------------------------------------------------------------ */
static dc_filter_t  s_dc_ecg;
static sg_filter_t  s_sg_ecg;

/* 50 Hz notch (fs=250) */
static biquad_df2t_t s_notch50 = {
    .b0 =  0.98351943f, .b1 = -1.05399213f, .b2 = 0.98351943f,
    .a1 = -1.05399213f, .a2 =  0.96703886f,
    .s1 = 0.0f, .s2 = 0.0f
};

/* LP Butterworth biquad cascade, ~40 Hz corner */
static biquad_df2t_t s_lpf_ecg_1 = {
    .b0 = 0.06745527f, .b1 = 0.13491055f, .b2 = 0.06745527f,
    .a1 = -1.14298050f, .a2 = 0.41280160f,
    .s1 = 0.0f, .s2 = 0.0f
};
static biquad_df2t_t s_lpf_ecg_2 = {
    .b0 = 0.06745038f, .b1 = 0.13490076f, .b2 = 0.06745038f,
    .a1 = -1.14289760f, .a2 = 0.41269904f,
    .s1 = 0.0f, .s2 = 0.0f
};

/* ------------------------------------------------------------------ */
/*  Forward declarations                                               */
/* ------------------------------------------------------------------ */
void log_init(void);
void timer_init(void);
void power_management_init(void);
void ble_stack_init(void);
void gap_params_init(void);
void gatt_init(void);
void services_init(void);
void advertising_init(void);
void conn_params_init(void);
void advertising_start(void);
void idle_state_handle(void);

void saadc_init(void);
void saadc_ppi_init(void);
void saadc_ppi_enable(void);

/* TWI helpers kept for I2C sensor expansion */
extern volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(1);
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void twi_init(void);


/* ================================================================== */
/*  SAADC PPI                                                          */
/* ================================================================== */

/**
 * @brief  Dead-simple timer handler � PPI fires the SAADC sample task,
 *         so this callback body can remain empty.
 */
static void saadc_timer_handler(nrf_timer_event_t event_type, void *p_context)
{
    (void)event_type;
    (void)p_context;
}

/**
 * @brief  Initialise TIMER3 ? PPI ? SAADC sample task chain (250 Hz).
 *         Call AFTER saadc_init().
 */
void saadc_ppi_init(void)
{
    ret_code_t err_code;

    /* --- PPI driver --- */
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    /* --- Timer --- */
    nrf_drv_timer_config_t tcfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    tcfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_saadc_timer, &tcfg, saadc_timer_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_saadc_timer, SAADC_SAMPLE_US);
    nrf_drv_timer_extended_compare(
        &m_saadc_timer,
        NRF_TIMER_CC_CHANNEL0,
        ticks,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
        false);                          /* no interrupt needed */
    nrf_drv_timer_enable(&m_saadc_timer);

    /* --- PPI channel: TIMER compare ? SAADC sample --- */
    uint32_t timer_evt  = nrf_drv_timer_compare_event_address_get(&m_saadc_timer,
                                                                   NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_task = nrf_drv_saadc_sample_task_get();

    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_chan);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_chan, timer_evt, saadc_task);
    APP_ERROR_CHECK(err_code);
}

void saadc_ppi_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_chan);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief  SAADC event callback � fires every SAADC_SAMPLE_US �s via PPI.
 *         Heavy DSP is deferred to main loop via g_ecg_ready flag.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type != NRF_DRV_SAADC_EVT_DONE) { return; }

    /* Save raw value and re-queue buffer immediately so no samples are missed */
    g_ecg_raw   = p_event->data.done.p_buffer[0];
    g_ecg_ready = true;

    ret_code_t err_code = nrf_drv_saadc_buffer_convert(
        p_event->data.done.p_buffer,
        SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief  Configure SAADC: 12-bit, AIN0, GAIN1_6, internal ref 0.6 V.
 *         Input range = 0.6 � 6 = 3.6 V full scale.
 *         AD8232 output is typically 0.5�2.5 V � fully within range.
 */
void saadc_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t ch = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    ch.gain      = NRF_SAADC_GAIN1_6;
    ch.reference = NRF_SAADC_REFERENCE_INTERNAL;  /* 0.6 V */
    ch.acq_time  = NRF_SAADC_ACQTIME_20US;
    ch.mode      = NRF_SAADC_MODE_SINGLE_ENDED;
    ch.burst     = NRF_SAADC_BURST_DISABLED;

    nrf_drv_saadc_config_t cfg = NRF_DRV_SAADC_DEFAULT_CONFIG;
    cfg.resolution = NRF_SAADC_RESOLUTION_12BIT;
    cfg.oversample = NRF_SAADC_OVERSAMPLE_4X;     /* oversampling + burst for lower noise */

    err_code = nrf_drv_saadc_init(&cfg, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &ch);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buf[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_buffer_convert(m_buf[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    nrf_drv_saadc_calibrate_offset();
}


/* ================================================================== */
/*  dc_remove helper                                                    */
/* ================================================================== */
float dc_remove(dc_filter_t *f, float x)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = x;
    f->sum += x;
    f->idx = (f->idx + 1) % DC_WINDOW;
    f->dc_comp = f->sum / DC_WINDOW;
    return x - f->dc_comp;
}


/* ================================================================== */
/*  MAIN                                                               */
/* ================================================================== */
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

    timer2_init();      /* free-running �s counter on TIMER2 */

    saadc_init();
    saadc_ppi_init();   /* arm TIMER3 ? PPI ? SAADC chain */
    saadc_ppi_enable(); /* start 250 Hz autonomous sampling */
		
    nlms_t nlms_ecg;
    delay_t delay_ecg;
    nlms_init(&nlms_ecg, 0.01f, 1e-6f);
    delay_init(&delay_ecg);
    sg_init(&s_sg_ecg);

    NRF_LOG_INFO("ECG PPI sampling started at 250 Hz");
    NRF_LOG_FLUSH();

    advertising_start();

    /* ------------------------------------------------------------ */
    /*  Main loop � no blocking, no busy-wait                        */
    /* ------------------------------------------------------------ */
    while (1)
    {
        /* g_ecg_ready is set inside SAADC IRQ */
        if (!g_ecg_ready) {
            idle_state_handle();   /* sleep until next event */
            continue;
        }
        g_ecg_ready = false;

        /* --- Snapshot volatile raw value safely --- */
        int16_t raw = g_ecg_raw;

        /* -------------------------------------------------- */
        /*  Signal chain                                        */
        /*  raw (12-bit, ~0�4095) ? scale ? dc ? notch ? lpf  */
        /* -------------------------------------------------- */

        /*
         * Scale: 1 LSB = 3.6 V / 4096 � 0.879 mV
         * Multiply by 1.0 keeps units in ADC counts � dc_remove
         * handles the large DC offset (~1800 counts for 1.65 V bias).
         */
        float x0 = (float)raw;

        float x1 = dc_remove(&s_dc_ecg, x0);       /* remove DC bias   */
        float x2 = biquad_step(&s_notch50,  x1);    /* 50 Hz notch      */
        float x3 = biquad_step(&s_lpf_ecg_1, x2);  /* anti-alias LPF   */
        float x4 = biquad_step(&s_lpf_ecg_2, x3);  /* 2nd LPF section  */
				float x5 = ale_process(&nlms_ecg, &delay_ecg, x4);
        float x6 = sg_step(&s_sg_ecg, x5);          /* SG smooth post-ALE */

        /* -------------------------------------------------- */
        /*  BLE: 50 samples * 2 bytes = 100-byte payload       */
        /*  At 250 Hz -> one notification every 200 ms         */
        /* -------------------------------------------------- */
        #define ECG_TX_SCALE  10.0f   /* tune up if waveform is weak, down if clipping */

				float  vs = x5 * ECG_TX_SCALE;
				s_send_buf[s_send_idx++] = (vs >  32767.0f) ?  32767 :
                            (vs < -32768.0f) ? -32768 :
                            (int16_t)vs;


        if (s_send_idx >= PACKET_SAMPLES)
        {
            s_send_idx = 0;
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID &&
                m_ble_max_data_len >= PACKET_SAMPLES * sizeof(int16_t))
            {
                uint16_t len = PACKET_SAMPLES * sizeof(int16_t);
                uint32_t err = ble_cus_data_send(&m_cus,
                                                 (uint8_t *)s_send_buf,
                                                 &len,
                                                 m_conn_handle);
                if (err != NRF_SUCCESS &&
                    err != NRF_ERROR_INVALID_STATE &&
                    err != NRF_ERROR_RESOURCES &&
                    err != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                {
                    APP_ERROR_HANDLER(err);
                }
            }
        }

        /* -------------------------------------------------- */
        /*  Debug log (comment out in production)              */
        /* -------------------------------------------------- */
        // was: NRF_LOG_INFO("raw:%d filt:%d\n", raw, (int16_t)x4);
				NRF_LOG_INFO("raw:%d filt:%d tx:%d\n", raw, (int16_t)x4, s_send_buf[s_send_idx - 1]);

        NRF_LOG_FLUSH();
    }
}


/* ================================================================== */
/*  TWI                                                                */
/* ================================================================== */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    if (p_event->type == NRF_DRV_TWI_EVT_DONE) { m_xfer_done = true; }
}

void twi_init(void)
{
    const nrf_drv_twi_config_t cfg = {
        .scl                = TWI_SCL_PIN,
        .sda                = TWI_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };
    ret_code_t err = nrf_drv_twi_init(&m_twi, &cfg, twi_handler, NULL);
    APP_ERROR_CHECK(err);
    nrf_drv_twi_enable(&m_twi);
}


/* ================================================================== */
/*  BLE stack                                                          */
/* ================================================================== */
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
    if (NRF_LOG_PROCESS() == false) { nrf_pwr_mgmt_run(); }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

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
    if      (p_evt->type == BLE_CUS_EVT_NOTIFY_ENABLE)  { NRF_LOG_INFO("Notify ON");  }
    else if (p_evt->type == BLE_CUS_EVT_NOTIFY_DISABLE) { NRF_LOG_INFO("Notify OFF"); }
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