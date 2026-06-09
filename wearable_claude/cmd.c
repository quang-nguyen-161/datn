#include "cmd.h"
#include "ble_gap.h"
#include "nrf_log.h"

/* -----------------------------------------------------------------------
 * Shared state definitions (declared extern in cmd.h)
 * ----------------------------------------------------------------------- */

/* ECG reconfiguration — defaults: 250 Hz, 200 ms batches */
volatile bool     g_cmd_cfg_pending  = false;
volatile uint32_t g_cmd_sample_us    = 4000U;  /* 1 000 000 / 250 Hz */
volatile uint16_t g_cmd_pkt_samples  = 50U;    /* 250 Hz * 200 ms / 1000 */

/* Node identity — 0xFF until CMD_ACK is received */
volatile uint8_t  g_node_id          = 0xFF;

/* Vital warning thresholds — defaults match PAYLOAD_CONFIG.md §9 */
volatile uint8_t  g_thr_ppg_hr_min   = 50;
volatile uint8_t  g_thr_ppg_hr_max   = 120;
volatile uint8_t  g_thr_ecg_hr_min   = 50;
volatile uint8_t  g_thr_ecg_hr_max   = 120;
volatile uint8_t  g_thr_spo2_min     = 90;
volatile uint8_t  g_thr_temp_min     = 35;
volatile uint8_t  g_thr_temp_max     = 38;

/* -----------------------------------------------------------------------
 * cmd_rx_handle
 * ----------------------------------------------------------------------- */
bool cmd_rx_handle(const uint8_t *data, uint16_t len, uint16_t max_samples)
{
    if (len < 1) { return false; }

    switch (data[0])
    {
        /* -----------------------------------------------------------
         * CMD_ACK 0xA0
         * [CMD][addr_b2][addr_b3][addr_b4][addr_b5][node_id]  — 6 B
         *
         * Verify the embedded BLE MAC fragment against our own address
         * before accepting the node_id.  In ble_gap_addr_t, addr[0] is
         * the LSB (last octet in colon notation), so:
         *   data[1] == addr_b2 == own.addr[3]
         *   data[2] == addr_b3 == own.addr[2]
         *   data[3] == addr_b4 == own.addr[1]
         *   data[4] == addr_b5 == own.addr[0]
         * ----------------------------------------------------------- */
        case CMD_ACK:
        {
            if (len < 6) { return false; }
            ble_gap_addr_t own;
            sd_ble_gap_addr_get(&own);
            if (data[1] != own.addr[3] || data[2] != own.addr[2] ||
                data[3] != own.addr[1] || data[4] != own.addr[0])
            {
                return false;
            }
            g_node_id = data[5];
            NRF_LOG_INFO("CMD_ACK: node_id=%u", (unsigned)g_node_id);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_ECG_CFG 0xCF
         * [CMD][node_id][freq_lo][freq_hi][int_lo][int_hi]  — 6 B
         * freq   : sample rate in Hz (uint16 LE)
         * int    : notify interval in ms (uint16 LE)
         * ----------------------------------------------------------- */
        case CMD_ECG_CFG:
        {
            if (len != 6 || data[1] != g_node_id) { return false; }

            uint16_t freq_hz     = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
            uint16_t interval_ms = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
            if (freq_hz == 0 || interval_ms == 0) { return false; }

            uint32_t us  = 1000000UL / freq_hz;
            uint16_t pkt = (uint16_t)((uint32_t)freq_hz * interval_ms / 1000);
            if (pkt == 0) { pkt = 1; }
            if (pkt > max_samples) { pkt = max_samples; }

            g_cmd_sample_us   = us;
            g_cmd_pkt_samples = pkt;
            g_cmd_cfg_pending = true;

            NRF_LOG_INFO("CMD_ECG_CFG: %u Hz, %u ms -> %u smp/pkt",
                         (unsigned)freq_hz, (unsigned)interval_ms, (unsigned)pkt);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_THR 0xCE
         * [CMD][node_id][ppgHrMin][ppgHrMax][ecgHrMin][ecgHrMax]
         *               [spo2Min][tempMin][tempMax]  — 9 B
         * All threshold values are uint8 (bpm / % / whole °C).
         * ----------------------------------------------------------- */
        case CMD_THR:
        {
            if (len != 9 || data[1] != g_node_id) { return false; }

            g_thr_ppg_hr_min = data[2];
            g_thr_ppg_hr_max = data[3];
            g_thr_ecg_hr_min = data[4];
            g_thr_ecg_hr_max = data[5];
            g_thr_spo2_min   = data[6];
            g_thr_temp_min   = data[7];
            g_thr_temp_max   = data[8];

            NRF_LOG_INFO("CMD_THR: ppgHR=%u-%u ecgHR=%u-%u",
                         data[2], data[3], data[4], data[5]);
            NRF_LOG_INFO("CMD_THR: spo2>=%u temp=%u-%u",
                         data[6], data[7], data[8]);
            return true;
        }

        default:
            return false;
    }
}
