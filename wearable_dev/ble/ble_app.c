#include "ble_app.h"

#include <string.h>
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
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "cus_service.h"
#include "device_mode.h"
#include "cmd.h"
#include "ecg.h"

/* ================================================================
 *  Configuration
 * ================================================================ */
#define APP_BLE_CONN_CFG_TAG        1
#define DEVICE_NAME                 "ECG_dev"
#define NUS_SERVICE_UUID_TYPE       BLE_UUID_TYPE_VENDOR_BEGIN
#define APP_BLE_OBSERVER_PRIO       3

#define APP_ADV_INTERVAL            64          /* 40 ms */
#define APP_ADV_DURATION            0           /* 0 = advertise forever */

#define MIN_CONN_INTERVAL           MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL           MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define SLAVE_LATENCY               0
#define CONN_SUP_TIMEOUT            MSEC_TO_UNITS(4000, UNIT_10_MS)

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define DEAD_BEEF                   0xDEADBEEF

/* ================================================================
 *  Module instances
 * ================================================================ */
BLE_CUS_DEF(m_cus);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

static uint16_t   m_conn_handle      = BLE_CONN_HANDLE_INVALID;
static uint16_t   m_ble_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;  /* 20 bytes; updated by gatt_evt_handler */
static ble_uuid_t m_adv_uuids[]      = { {CUS_SERVICE_UUID, NUS_SERVICE_UUID_TYPE} };

/* ================================================================
 *  Public accessors
 * ================================================================ */
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
    /* Update preferred parameters (used on next connection) */
    sd_ble_gap_ppcp_set(&params);
    /* If already connected, request update now */
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        sd_ble_gap_conn_param_update(m_conn_handle, &params);
    }
}

uint32_t ble_app_send(uint8_t const *data, uint16_t len)
{
    if (!ble_app_is_connected())    { return NRF_ERROR_INVALID_STATE; }
    if (len > m_ble_max_data_len)   { return NRF_ERROR_DATA_SIZE; }
    uint16_t l = len;
    return ble_cus_data_send(&m_cus, (uint8_t *)data, &l, m_conn_handle);
}

/* ================================================================
 *  System helpers
 * ================================================================ */
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

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/* ================================================================
 *  GAP
 * ================================================================ */
static void gap_params_init(void)
{
    ble_gap_conn_params_t   cp;
    ble_gap_conn_sec_mode_t sec;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec);
    sd_ble_gap_device_name_set(&sec, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));

    memset(&cp, 0, sizeof(cp));
    cp.min_conn_interval = MIN_CONN_INTERVAL;
    cp.max_conn_interval = MAX_CONN_INTERVAL;
    cp.slave_latency     = SLAVE_LATENCY;
    cp.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    sd_ble_gap_ppcp_set(&cp);
}

/* ================================================================
 *  GATT
 * ================================================================ */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) &&
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("MTU updated: eff=%u data_len=%u", p_evt->params.att_mtu_effective, m_ble_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

static void gatt_init(void)
{
    nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 247);
    NRF_LOG_INFO("[GATT] Peripheral MTU requested = 247");
}

/* ================================================================
 *  Services
 * ================================================================ */
static void nrf_qwr_error_handler(uint32_t nrf_error) { APP_ERROR_HANDLER(nrf_error); }

static void cus_data_handler(ble_cus_evt_t *p_evt)
{
    switch (p_evt->type)
    {
        case BLE_CUS_EVT_NOTIFY_ENABLE:
            NRF_LOG_INFO("Notify ON");
            break;
        case BLE_CUS_EVT_NOTIFY_DISABLE:
            NRF_LOG_INFO("Notify OFF");
            break;
        case BLE_CUS_EVT_RX_DATA:
            /* Route to gateway command handler (CMD_ACK, CMD_ECG_CFG, CMD_THR, etc.) */
            cmd_rx_handle(p_evt->params.rx_data.p_data,
                          p_evt->params.rx_data.length,
                          ECG_BUF_SAMPLES);
            break;
        default:
            break;
    }
}

static void services_init(void)
{
    nrf_ble_qwr_init_t qwr = {0};
    qwr.error_handler = nrf_qwr_error_handler;
    nrf_ble_qwr_init(&m_qwr, &qwr);

    ble_cus_init_t cus = {0};
    cus.data_handler = cus_data_handler;
    ble_cus_init(&m_cus, &cus);
}

/* ================================================================
 *  Connection parameters
 * ================================================================ */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
}

static void conn_params_error_handler(uint32_t e) { APP_ERROR_HANDLER(e); }

static void conn_params_init(void)
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

/* ================================================================
 *  BLE event handler
 * ================================================================ */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("[GAP] Connected  handle=0x%04X", p_ble_evt->evt.gap_evt.conn_handle);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);

            ble_gap_data_length_params_t dl = {
                .max_tx_octets  = 251,
                .max_rx_octets  = 251,
                .max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO,
                .max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO,
            };
            ret_code_t err = sd_ble_gap_data_length_update(m_conn_handle, &dl, NULL);
            if (err != NRF_SUCCESS && err != NRF_ERROR_INVALID_STATE)
            {
                NRF_LOG_WARNING("DLE request 0x%X (central may not support)", err);
            }
        }
        break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("[GAP] Disconnected  reason=0x%02X",
                         p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            /* Advertising restarts automatically via on_adv_evt */
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t const *p =
                &p_ble_evt->evt.gap_evt.params.data_length_update_request.peer_params;
            NRF_LOG_INFO("[DLE] Central proposes TX=%u RX=%u - accepting",
                         p->max_tx_octets, p->max_rx_octets);
            sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, NULL, NULL);
        }
        break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
        {
            ble_gap_data_length_params_t const *p =
                &p_ble_evt->evt.gap_evt.params.data_length_update.effective_params;
            NRF_LOG_INFO("DLE: TX=%u B/%u us  RX=%u B/%u us",
                         p->max_tx_octets, p->max_tx_time_us,
                         p->max_rx_octets, p->max_rx_time_us);
        }
        break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys = {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            ret_code_t err = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err);
        }
        break;

        case BLE_GAP_EVT_PHY_UPDATE:
        {
            ble_gap_evt_phy_update_t const *p = &p_ble_evt->evt.gap_evt.params.phy_update;
            NRF_LOG_INFO("[PHY] TX=%u RX=%u Mbps",
                         (p->tx_phy == BLE_GAP_PHY_AUTO) ? 2 : 1,
                         (p->rx_phy == BLE_GAP_PHY_AUTO) ? 2 : 1);
        }
        break;

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

/* ================================================================
 *  BLE stack init
 * ================================================================ */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_obs, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/* ================================================================
 *  Advertising
 * ================================================================ */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    /* Restart on timeout so ECG keeps running without user intervention */
    if (ble_adv_evt == BLE_ADV_EVT_IDLE)
        ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
}

static void advertising_init(void)
{
    ble_advertising_init_t init = {0};
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; 	//never set this to limited_disc_mode
    init.srdata.uuids_complete.uuid_cnt  = ARRAY_SIZE(m_adv_uuids);
    init.srdata.uuids_complete.p_uuids   = m_adv_uuids;
    init.config.ble_adv_fast_enabled     = true;
    init.config.ble_adv_fast_interval    = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout     = APP_ADV_DURATION;
    init.evt_handler                     = on_adv_evt;
    ble_advertising_init(&m_advertising, &init);
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/* ================================================================
 *  Public init
 * ================================================================ */
void ble_app_init(void)
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
}

void ble_app_advertising_start(void)
{
    ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    NRF_LOG_INFO("Advertising started");
}
