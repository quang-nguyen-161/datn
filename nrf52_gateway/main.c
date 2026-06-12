/**
 * nrf52_gateway/main.c  —  nRF52832 BLE central / UART bridge
 *
 * Connects to one ECG node advertising custom service 6e401400-...,
 * then bridges two directions:
 *
 *   BLE → UART  (ECG data)
 *     Receives notify packets on TX char (0x1401), wraps them in the
 *     project binary framing, sends to ESP32 as TYPE 0x01.
 *
 *   UART → BLE  (ECG config)
 *     Parses framed TYPE 0x03 config packets from ESP32 and writes the
 *     5-byte payload to the node's RX char (0x1402).
 *
 * UART framing (115200 baud, matches firmware/src/main.cpp on ESP32):
 *   [0xAA][0x55][TYPE][NAME_LEN][NAME…][LEN_LO][LEN_HI][DATA…][XOR_CHK]
 *   XOR_CHK = XOR of every byte from TYPE through last DATA byte.
 *
 * Configurable:
 *   TB_NODE_NAME   ThingsBoard device name embedded in UART TX frames ("Node6")
 *   BLE_SCAN_NAME  BLE advertisement name to scan for              ("ECG_dev")
 *
 * sdk_config.h notes:
 *   Disable BLE_NUS_C_ENABLED (no longer used).
 *   BLE_DB_DISCOVERY_MAX_SRV must be >= 1.
 *   NRF_SDH_BLE_CENTRAL_LINK_COUNT = 1.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "ble_db_discovery.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_gq.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* ── Project config ──────────────────────────────────────────────────────── */

#define TB_NODE_NAME    "Node6"     /* ThingsBoard device name → UART frame  */
#define BLE_SCAN_NAME   "ECG_dev"   /* BLE advertisement name to connect to  */

/* ── UART framing (must match ESP32 firmware and gateway.py) ─────────────── */

/* Inbound (nRF → ESP32) */
#define PKT_TYPE_ECG        0x01   /* up to 100×int16 LE = 200 bytes (dynamic, size-agnostic dispatch) */
#define PKT_TYPE_VITALS     0x02   /* 4×float32 LE = 16 bytes: [ecgHr][ppgHr][spo2][temp] */
/* Outbound (ESP32 → nRF, forwarded to BLE node) */
#define PKT_TYPE_CFG        0x03   /* ECG config:   5 B [0xCF][fLo][fHi][iLo][iHi] */
#define PKT_TYPE_THR        0x04   /* Threshold:   31 B [0xCE][18×u8 PPG/ECG/SpO2][6×u16LE temp×10] */
#define PKT_TYPE_PPG        0x05   /* PPG config:   5 B [0xCD][fLo][fHi][redMa][irMa] */
#define PKT_TYPE_VCF        0x06   /* Vital cfg:    3 B [0xCC][iLo][iHi] */
#define PKT_TYPE_MODE       0x07   /* Mode cfg:     7 B [0xCB][mode][periodSec u16 LE][capSec u16 LE][ecgEnabled] */
#define ECG_CFG_CMD         0xCF
#define THR_CMD             0xCE
#define PPG_CFG_CMD         0xCD
#define VITAL_CFG_CMD       0xCC
#define MODE_CFG_CMD        0xCB
#define MAX_CMD_LEN         31     /* largest outbound payload = threshold   */
#define UART_MAGIC_0        0xAA
#define UART_MAGIC_1        0x55
#define UART_TX_BUF_SIZE    256
#define UART_RX_BUF_SIZE    256
/* UART pins connecting to ESP32 (nRF TX → ESP32 RX=16, nRF RX ← ESP32 TX=17) */
#define UART_TX_PIN         2       /* P0.02 */
#define UART_RX_PIN         3       /* P0.03 */

/* ── Custom BLE service UUIDs (must match cus_service.h on the node) ─────── */
/* Base: 6e401400-b5a3-f393-e0a9-e50e24dcca9e  (16-byte LE, positions 12-13 = 0) */
#define CUS_UUID_BASE_INIT  { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0, \
                              0x93,0xF3,0xA3,0xB5,0x00,0x00,0x40,0x6E }
#define CUS_SERVICE_UUID    0x1400
#define CUS_TX_CHAR_UUID    0x1401  /* node → central, notify  (ECG data)   */
#define CUS_RX_CHAR_UUID    0x1402  /* central → node, write   (ECG config) */

/* ── BLE SDK config ──────────────────────────────────────────────────────── */

#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3

/* ── BLE module instances ────────────────────────────────────────────────── */

NRF_BLE_GATT_DEF(m_gatt);
BLE_DB_DISCOVERY_DEF(m_db_disc);
NRF_BLE_SCAN_DEF(m_scan);
NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);

/* ── Runtime state ───────────────────────────────────────────────────────── */

static uint8_t  m_cus_uuid_type  = BLE_UUID_TYPE_UNKNOWN;
static uint16_t m_conn_handle    = BLE_CONN_HANDLE_INVALID;
static uint16_t m_tx_char_handle = BLE_GATT_HANDLE_INVALID; /* notify source */
static uint16_t m_tx_cccd_handle = BLE_GATT_HANDLE_INVALID; /* CCCD for TX  */
static uint16_t m_rx_char_handle = BLE_GATT_HANDLE_INVALID; /* write target  */

/* ══════════════════════════════════════════════════════════════════════════
   UART TX  —  send framed ECG packet to ESP32
   Packet: [AA][55][01][nlen][name…][len_lo][len_hi][data…][xor]
   ══════════════════════════════════════════════════════════════════════════ */

static void uart_send_ecg(const uint8_t *data, uint16_t len)
{
    const char    *name  = TB_NODE_NAME;
    const uint8_t  nlen  = (uint8_t)strlen(name);

    /* Build checksum: XOR of TYPE through last DATA byte */
    uint8_t chk = PKT_TYPE_ECG;
    chk ^= nlen;
    for (uint8_t i = 0; i < nlen; i++)   chk ^= (uint8_t)name[i];
    chk ^= (uint8_t)(len & 0xFF);
    chk ^= (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++)   chk ^= data[i];

    /* Write frame to UART FIFO — non-blocking; drop on overflow */
#define PUT(b)  if (app_uart_put(b) != NRF_SUCCESS) { \
                    NRF_LOG_WARNING("[UART] TX overflow"); return; }
    PUT(UART_MAGIC_0); PUT(UART_MAGIC_1); PUT(PKT_TYPE_ECG); PUT(nlen);
    for (uint8_t  i = 0; i < nlen; i++) { PUT((uint8_t)name[i]); }
    PUT((uint8_t)(len & 0xFF)); PUT((uint8_t)(len >> 8));
    for (uint16_t i = 0; i < len;  i++) { PUT(data[i]); }
    PUT(chk);
#undef PUT
}

/* ══════════════════════════════════════════════════════════════════════════
   UART TX  —  send framed vitals packet to ESP32
   Packet: [AA][55][02][nlen][name…][len_lo][len_hi][data…][xor]
   ══════════════════════════════════════════════════════════════════════════ */

static void uart_send_vitals(const uint8_t *data, uint16_t len)
{
    const char    *name = TB_NODE_NAME;
    const uint8_t  nlen = (uint8_t)strlen(name);

    uint8_t chk = PKT_TYPE_VITALS;
    chk ^= nlen;
    for (uint8_t i = 0; i < nlen; i++)   chk ^= (uint8_t)name[i];
    chk ^= (uint8_t)(len & 0xFF);
    chk ^= (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++)   chk ^= data[i];

#define PUT(b)  if (app_uart_put(b) != NRF_SUCCESS) { \
                    NRF_LOG_WARNING("[UART] TX overflow"); return; }
    PUT(UART_MAGIC_0); PUT(UART_MAGIC_1); PUT(PKT_TYPE_VITALS); PUT(nlen);
    for (uint8_t  i = 0; i < nlen; i++) { PUT((uint8_t)name[i]); }
    PUT((uint8_t)(len & 0xFF)); PUT((uint8_t)(len >> 8));
    for (uint16_t i = 0; i < len;  i++) { PUT(data[i]); }
    PUT(chk);
#undef PUT
}

/* ══════════════════════════════════════════════════════════════════════════
   UART RX  —  parse framed config packets from ESP32
   Only TYPE 0x03 (config) is acted on; all other types are discarded.
   ══════════════════════════════════════════════════════════════════════════ */

typedef enum { SM_M0,SM_M1,SM_TYPE,SM_NL,SM_NAME,SM_LL,SM_LH,SM_DATA,SM_CHK } rx_sm_t;

static rx_sm_t  s_sm     = SM_M0;
static uint8_t  s_type   = 0;
static uint8_t  s_nlen   = 0, s_nidx = 0;
static uint16_t s_dlen   = 0, s_didx = 0;
static uint8_t  s_data[32];        /* 5 bytes for config, 32 for safety     */
static uint8_t  s_xor    = 0;

/* Two alternating write buffers — keeps the pointer valid until GATT queue drains */
static uint8_t  s_wr_buf[2][MAX_CMD_LEN];
static uint8_t  s_wr_sel = 0;

static void ble_write_config(const uint8_t *payload, uint16_t len); /* fwd */

static void uart_handle_pkt(void)
{
    if (s_dlen == 0 || s_dlen > MAX_CMD_LEN) return;

    bool fwd = false;
    switch (s_type) {
        case PKT_TYPE_CFG: fwd = (s_dlen == 5  && s_data[0] == ECG_CFG_CMD);   break;
        case PKT_TYPE_THR: fwd = (s_dlen == 31 && s_data[0] == THR_CMD);       break;
        case PKT_TYPE_PPG: fwd = (s_dlen == 5  && s_data[0] == PPG_CFG_CMD);   break;
        case PKT_TYPE_VCF: fwd = (s_dlen == 3  && s_data[0] == VITAL_CFG_CMD); break;
        case PKT_TYPE_MODE: fwd = (s_dlen == 7 && s_data[0] == MODE_CFG_CMD); break;
        default: break;
    }
    if (!fwd) {
        NRF_LOG_WARNING("[UART] unrecognised cmd type=0x%02X len=%u cmd=0x%02X",
                        s_type, s_dlen, s_data[0]);
        return;
    }
    memcpy(s_wr_buf[s_wr_sel], s_data, s_dlen);
    ble_write_config(s_wr_buf[s_wr_sel], (uint16_t)s_dlen);
    s_wr_sel ^= 1;
}

static void uart_rx_byte(uint8_t b)
{
    switch (s_sm) {
        case SM_M0: if (b == UART_MAGIC_0) s_sm = SM_M1; break;
        case SM_M1: s_sm = (b == UART_MAGIC_1) ? SM_TYPE : SM_M0; break;
        case SM_TYPE:
            s_type = b; s_xor = b; s_sm = SM_NL; break;
        case SM_NL:
            s_nlen = (b <= 15) ? b : 15; s_nidx = 0; s_xor ^= b;
            s_sm = s_nlen ? SM_NAME : SM_LL; break;
        case SM_NAME:
            s_nidx++; s_xor ^= b;
            if (s_nidx >= s_nlen) s_sm = SM_LL; break;
        case SM_LL:
            s_dlen = b; s_xor ^= b; s_sm = SM_LH; break;
        case SM_LH:
            s_dlen |= (uint16_t)b << 8; s_xor ^= b; s_didx = 0;
            s_sm = s_dlen ? SM_DATA : SM_CHK; break;
        case SM_DATA:
            if (s_didx < sizeof(s_data)) s_data[s_didx] = b;
            s_xor ^= b;
            if (++s_didx >= s_dlen) s_sm = SM_CHK;
            break;
        case SM_CHK:
            if (b == s_xor) uart_handle_pkt();
            else NRF_LOG_WARNING("[UART] bad XOR got=0x%02X exp=0x%02X", b, s_xor);
            s_sm = SM_M0; break;
    }
}

void uart_event_handle(app_uart_evt_t *p_event)
{
    uint8_t byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&byte));
            uart_rx_byte(byte);
            break;
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("UART comm error 0x%x", p_event->data.error_communication);
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("UART FIFO error 0x%x", p_event->data.error_code);
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
        default: break;
    }
}

static void uart_init(void)
{
    ret_code_t err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_RX_PIN,
        .tx_pin_no    = UART_TX_PIN,
        .rts_pin_no   = RTS_PIN_NUMBER,   /* unused — flow control disabled */
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200,
    };
    APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
                       uart_event_handle, APP_IRQ_PRIORITY_LOWEST, err_code);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("[UART] init TX=%d RX=%d 115200baud", UART_TX_PIN, UART_RX_PIN);
}

/* ══════════════════════════════════════════════════════════════════════════
   BLE GATT write helpers
   ══════════════════════════════════════════════════════════════════════════ */

/* Enable notifications on the node's TX characteristic (write 0x0001 to CCCD) */
static uint8_t s_cccd_buf[BLE_CCCD_VALUE_LEN] = {BLE_GATT_HVX_NOTIFICATION, 0x00};

static void enable_tx_notification(uint16_t conn_handle)
{
    if (m_tx_cccd_handle == BLE_GATT_HANDLE_INVALID) return;

    nrf_ble_gq_req_t req;
    memset(&req, 0, sizeof(req));
    req.type                          = NRF_BLE_GQ_REQ_GATTC_WRITE;
    req.params.gattc_write.handle     = m_tx_cccd_handle;
    req.params.gattc_write.write_op   = BLE_GATT_OP_WRITE_REQ;
    req.params.gattc_write.p_data     = s_cccd_buf;
    req.params.gattc_write.len        = BLE_CCCD_VALUE_LEN;

    ret_code_t err = nrf_ble_gq_item_add(&m_ble_gatt_queue, &req, conn_handle);
    if (err != NRF_SUCCESS) NRF_LOG_ERROR("CCCD enable queue err %d", err);
    else                    NRF_LOG_INFO("[BLE] Enabling ECG notifications");
}

/* Forward 5-byte config payload to node's RX characteristic */
static void ble_write_config(const uint8_t *payload, uint16_t len)
{
    if (m_rx_char_handle == BLE_GATT_HANDLE_INVALID || m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_WARNING("[BLE] config write: not connected");
        return;
    }

    nrf_ble_gq_req_t req;
    memset(&req, 0, sizeof(req));
    req.type                          = NRF_BLE_GQ_REQ_GATTC_WRITE;
    req.params.gattc_write.handle     = m_rx_char_handle;
    req.params.gattc_write.write_op   = BLE_GATT_OP_WRITE_REQ;
    req.params.gattc_write.p_data     = payload;
    req.params.gattc_write.len        = len;

    ret_code_t err = nrf_ble_gq_item_add(&m_ble_gatt_queue, &req, m_conn_handle);
    if (err != NRF_SUCCESS) NRF_LOG_ERROR("BLE write queue err %d", err);
    else NRF_LOG_INFO("[BLE] -> node cmd=0x%02X len=%u", payload[0], len);
}

/* ══════════════════════════════════════════════════════════════════════════
   Custom UUID registration
   ══════════════════════════════════════════════════════════════════════════ */

static void cus_uuid_init(void)
{
    ble_uuid128_t base = {CUS_UUID_BASE_INIT};
    ret_code_t    err  = sd_ble_uuid_vs_add(&base, &m_cus_uuid_type);
    APP_ERROR_CHECK(err);
    NRF_LOG_INFO("[BLE] custom UUID type = %u", m_cus_uuid_type);
}

/* ══════════════════════════════════════════════════════════════════════════
   Database discovery — find TX/RX char handles, enable notify
   ══════════════════════════════════════════════════════════════════════════ */

static void db_disc_handler(ble_db_discovery_evt_t *p_evt)
{
    if (p_evt->evt_type != BLE_DB_DISCOVERY_COMPLETE) return;

    ble_db_discovery_srv_t *srv = &p_evt->params.discovered_db;
    if (srv->srv_uuid.uuid != CUS_SERVICE_UUID ||
        srv->srv_uuid.type != m_cus_uuid_type) return;

    NRF_LOG_INFO("[BLE] Custom service found. %d characteristics.", srv->char_count);

    for (uint8_t i = 0; i < srv->char_count; i++)
    {
        /* Note: SDK typo — field is "charateristics" (one 'c') */
        ble_db_discovery_char_t *c = &srv->charateristics[i];
        uint16_t uuid = c->characteristic.uuid.uuid;

        if (uuid == CUS_TX_CHAR_UUID)
        {
            m_tx_char_handle = c->characteristic.handle_value;
            m_tx_cccd_handle = c->cccd_handle;
            NRF_LOG_INFO("[BLE] TX char 0x%04X  CCCD 0x%04X",
                         m_tx_char_handle, m_tx_cccd_handle);
        }
        else if (uuid == CUS_RX_CHAR_UUID)
        {
            m_rx_char_handle = c->characteristic.handle_value;
            NRF_LOG_INFO("[BLE] RX char 0x%04X", m_rx_char_handle);
        }
    }

    enable_tx_notification(p_evt->conn_handle);
}

/* ══════════════════════════════════════════════════════════════════════════
   Scanning
   ══════════════════════════════════════════════════════════════════════════ */

static void scan_start(void)
{
    ret_code_t err = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err);
    NRF_LOG_INFO("[SCAN] scanning for \"%s\"...", BLE_SCAN_NAME);
}

static void scan_evt_handler(scan_evt_t const *p_evt)
{
    switch (p_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            APP_ERROR_CHECK(p_evt->params.connecting_err.err_code);
            break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
            NRF_LOG_INFO("[SCAN] connecting to %02X:%02X:%02X:%02X:%02X:%02X",
                p_evt->params.connected.p_connected->peer_addr.addr[5],
                p_evt->params.connected.p_connected->peer_addr.addr[4],
                p_evt->params.connected.p_connected->peer_addr.addr[3],
                p_evt->params.connected.p_connected->peer_addr.addr[2],
                p_evt->params.connected.p_connected->peer_addr.addr[1],
                p_evt->params.connected.p_connected->peer_addr.addr[0]);
            break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
            NRF_LOG_INFO("[SCAN] timeout, restarting");
            scan_start();
            break;

        default: break;
    }
}

static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    /* Filter by advertisement name — no dependency on UUID type at scan time */
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, BLE_SCAN_NAME);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

/* ══════════════════════════════════════════════════════════════════════════
   BLE event handler
   ══════════════════════════════════════════════════════════════════════════ */

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const  *p_gap = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("[BLE] connected");
            m_conn_handle    = p_gap->conn_handle;
            m_tx_char_handle = BLE_GATT_HANDLE_INVALID;
            m_tx_cccd_handle = BLE_GATT_HANDLE_INVALID;
            m_rx_char_handle = BLE_GATT_HANDLE_INVALID;
            err_code = ble_db_discovery_start(&m_db_disc, p_gap->conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("[BLE] disconnected reason=0x%x", p_gap->params.disconnected.reason);
            m_conn_handle    = BLE_CONN_HANDLE_INVALID;
            m_tx_char_handle = BLE_GATT_HANDLE_INVALID;
            m_tx_cccd_handle = BLE_GATT_HANDLE_INVALID;
            m_rx_char_handle = BLE_GATT_HANDLE_INVALID;
            scan_start();
            break;

        /* BLE notify from node — dispatch by packet length, forward to ESP32 */
        case BLE_GATTC_EVT_HVX:
        {
            ble_gattc_evt_hvx_t const *hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
            if (hvx->handle == m_tx_char_handle &&
                hvx->type   == BLE_GATT_HVX_NOTIFICATION)
            {
                if (hvx->len == 16)                        /* 4×float32 vitals (always 16 B) */
                {
                    uart_send_vitals(hvx->data, hvx->len);
                }
                else if (hvx->len >= 2 && (hvx->len & 1) == 0) /* N×int16 ECG (dynamic batch) */
                {
                    uart_send_ecg(hvx->data, hvx->len);
                }
                else
                {
                    NRF_LOG_WARNING("[BLE] unexpected HVX len=%u", hvx->len);
                }
            }
        }
        break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("[BLE] connection timeout, scanning");
                scan_start();
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            sd_ble_gap_sec_params_reply(p_gap->conn_handle,
                                        BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            sd_ble_gap_conn_param_update(p_gap->conn_handle,
                &p_gap->params.conn_param_update_request.conn_params);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys = {BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO};
            sd_ble_gap_phy_update(p_gap->conn_handle, &phys);
        }
        break;

        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_DEBUG("[BLE] GATT client timeout");
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_DEBUG("[BLE] GATT server timeout");
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        default: break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
   GATT  —  MTU negotiation
   The central requests NRF_SDH_BLE_GATT_MAX_MTU_SIZE; after exchange the
   node's m_ble_max_data_len is updated so it starts sending 100-byte ECG
   packets (fixes the MTU gate issue in the node firmware).
   ══════════════════════════════════════════════════════════════════════════ */

static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("[BLE] MTU updated to %u bytes",
                     p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH);
    }
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    /* Request max MTU — resolves the silent packet-drop issue in the node */
    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/* ══════════════════════════════════════════════════════════════════════════
   BLE stack / DB discovery init
   ══════════════════════════════════════════════════════════════════════════ */

static void ble_stack_init(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;
    memset(&db_init, 0, sizeof(db_init));
    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    /* Register the custom service UUID — must be done after cus_uuid_init() */
    ble_uuid_t cus_svc = {.uuid = CUS_SERVICE_UUID, .type = m_cus_uuid_type};
    err_code = ble_db_discovery_evt_register(&cus_svc);
    APP_ERROR_CHECK(err_code);
}

/* ══════════════════════════════════════════════════════════════════════════
   Misc init / main loop
   ══════════════════════════════════════════════════════════════════════════ */

static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false) { nrf_pwr_mgmt_run(); }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/* ══════════════════════════════════════════════════════════════════════════
   main
   ══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    log_init();
    timer_init();
    power_management_init();

    ble_stack_init();
    cus_uuid_init();        /* sd_ble_uuid_vs_add — must be after ble_stack_init   */
    gatt_init();
    db_discovery_init();    /* register CUS UUID — must be after cus_uuid_init      */
    scan_init();            /* name filter — no UUID type dependency at scan time   */
    uart_init();            /* UART to ESP32                                        */

    NRF_LOG_INFO("nRF52 gateway started. Node=%s BLE=%s", TB_NODE_NAME, BLE_SCAN_NAME);
    NRF_LOG_FLUSH();

    scan_start();

    for (;;)
    {
        idle_state_handle();
    }
}
