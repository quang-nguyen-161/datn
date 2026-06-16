#include <string.h>

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
#include "app_error.h"
#include "nrf_log.h"

#include "ble_app.h"
#include "cus_service.h"
#include "cmd.h"

/* ------------------------------------------------------------------ */
/*  BLE configuration                                                  */
/* ------------------------------------------------------------------ */
#define APP_BLE_CONN_CFG_TAG        1
#define DEVICE_NAME                 "ECG_dev"
#define NUS_SERVICE_UUID_TYPE       BLE_UUID_TYPE_VENDOR_BEGIN
#define APP_BLE_OBSERVER_PRIO       3
#define APP_ADV_INTERVAL            64          /* 40 ms */
#define APP_ADV_DURATION            18000       /* 180 s  */
#define MIN_CONN_INTERVAL           MSEC_TO_UNITS(200,  UNIT_1_25_MS)
#define MAX_CONN_INTERVAL           MSEC_TO_UNITS(200,  UNIT_1_25_MS)
#define SLAVE_LATENCY               0
#define CONN_SUP_TIMEOUT            MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define PACKET_SAMPLES_DEFAULT   50U     /* 250 Hz x 200 ms */
#define PACKET_SAMPLES_MAX       128U    /* hard cap: 256 bytes < any negotiated MTU */

/* ------------------------------------------------------------------ */
/*  BLE instances                                                       */
/* ------------------------------------------------------------------ */
BLE_CUS_DEF(m_cus);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

static uint16_t   m_conn_handle        = BLE_CONN_HANDLE_INVALID;
static uint16_t   m_ble_max_data_len   = BLE_GATT_ATT_MTU_DEFAULT - 3;
static volatile bool m_mtu_negotiated  = false;  /* set once ATT MTU exchange completes */
static volatile int8_t m_rssi          = 0;      /* latest RSSI, updated via BLE_GAP_EVT_RSSI_CHANGED */
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
        NRF_LOG_INFO("Notify ON");
    }
    else if (p_evt->type == BLE_CUS_EVT_NOTIFY_DISABLE)
    {
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
            m_conn_handle    = p_ble_evt->evt.gap_evt.conn_handle;
            m_mtu_negotiated = false;
            m_rssi           = 0;
            sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,
                                    p_ble_evt->evt.gap_evt.conn_handle, 0);
            nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            sd_ble_gap_rssi_start(m_conn_handle, 0, 0);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_rssi        = 0;
            break;

        case BLE_GAP_EVT_RSSI_CHANGED:
            m_rssi = p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
            NRF_LOG_INFO("Data length updated: tx_octets=%u rx_octets=%u tx_time=%u rx_time=%u",
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets,
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets,
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_time_us,
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_time_us);
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
        m_mtu_negotiated   = true;
        NRF_LOG_INFO("MTU negotiated: max_data_len=%u", m_ble_max_data_len);
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
        /* Restart advertising instead of sleeping — keeps ECG running */
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
bool     ble_app_ready_to_send(void) { return ble_app_is_connected() && m_mtu_negotiated; }

void ble_app_get_addr(uint8_t addr[6])
{
    ble_gap_addr_t gap_addr;
    sd_ble_gap_addr_get(&gap_addr);
    memcpy(addr, gap_addr.addr, 6);
}

int8_t ble_app_get_rssi(void) { return (int8_t)m_rssi; }

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
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) { return NRF_ERROR_INVALID_STATE; }
    if (len > m_ble_max_data_len)                 { return NRF_ERROR_DATA_SIZE; }
    uint16_t l = len;
    return ble_cus_data_send(&m_cus, (uint8_t *)data, &l, m_conn_handle);
}
