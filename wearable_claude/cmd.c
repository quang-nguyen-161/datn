#include "cmd.h"
#include "ble_gap.h"
#define NRF_LOG_MODULE_NAME cmd
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* -----------------------------------------------------------------------
 * ECG reconfiguration — defaults: 250 Hz, 200 ms batches
 * ----------------------------------------------------------------------- */
volatile bool     g_cmd_cfg_pending  = false;
volatile uint32_t g_cmd_sample_us    = 4000U;
volatile uint16_t g_cmd_pkt_samples  = 50U;

/* Node identity — 0xFF until CMD_ACK */
volatile uint8_t  g_node_id          = 0xFF;

/* PPG (MAX30102) reconfiguration — defaults from PAYLOAD_CONFIG §9 */
volatile bool     g_ppg_cfg_pending  = false;
volatile uint16_t g_ppg_sample_freq  = 100U;  /* Hz */
volatile uint8_t  g_ppg_red_ma       = 6U;    /* mA */
volatile uint8_t  g_ppg_ir_ma        = 6U;    /* mA */

/* Vital reporting interval */
volatile bool     g_vital_cfg_pending = false;
volatile uint16_t g_vital_interval_ms = 1000U; /* ms */

/* PPG heart rate thresholds (bpm) */
volatile uint8_t  g_thr_ppg_norm_min = 30;
volatile uint8_t  g_thr_ppg_norm_max = 100;
volatile uint8_t  g_thr_ppg_warn_min = 40;
volatile uint8_t  g_thr_ppg_warn_max = 50;
volatile uint8_t  g_thr_ppg_dang_min = 60;
volatile uint8_t  g_thr_ppg_dang_max = 130;

/* ECG heart rate thresholds (bpm) */
volatile uint8_t  g_thr_ecg_norm_min = 40;
volatile uint8_t  g_thr_ecg_norm_max = 100;
volatile uint8_t  g_thr_ecg_warn_min = 50;
volatile uint8_t  g_thr_ecg_warn_max = 120;
volatile uint8_t  g_thr_ecg_dang_min = 40;
volatile uint8_t  g_thr_ecg_dang_max = 130;

/* SpO2 thresholds (%) */
volatile uint8_t  g_thr_spo2_norm_min = 90;
volatile uint8_t  g_thr_spo2_norm_max = 100;
volatile uint8_t  g_thr_spo2_warn_min = 90;
volatile uint8_t  g_thr_spo2_warn_max = 100;
volatile uint8_t  g_thr_spo2_dang_min = 88;
volatile uint8_t  g_thr_spo2_dang_max = 100;

/* Temperature thresholds (×10 °C, uint16)  e.g. 361 = 36.1 °C */
volatile uint16_t g_thr_temp_norm_min = 361;
volatile uint16_t g_thr_temp_norm_max = 372;
volatile uint16_t g_thr_temp_warn_min = 355;
volatile uint16_t g_thr_temp_warn_max = 385;
volatile uint16_t g_thr_temp_dang_min = 350;
volatile uint16_t g_thr_temp_dang_max = 395;

/* -----------------------------------------------------------------------
 * cmd_rx_handle
 * ----------------------------------------------------------------------- */
bool cmd_rx_handle(const uint8_t *data, uint16_t len, uint16_t max_samples)
{
    if (len < 1)
    {
        NRF_LOG_WARNING("GW RX: empty packet");
        return false;
    }

    NRF_LOG_INFO("GW RX: cmd=0x%02X len=%u node_id=%u",
                 (unsigned)data[0], (unsigned)len, (unsigned)g_node_id);

    switch (data[0])
    {
        /* -----------------------------------------------------------
         * CMD_ACK 0xA0  —  6 bytes
         * [CMD][addr_b2][addr_b3][addr_b4][addr_b5][node_id]
         * addr_b2..b5 = own MAC bytes [3..0] (addr[0] is LSB)
         * ----------------------------------------------------------- */
        case CMD_ACK:
        {
            if (len < 6)
            {
                NRF_LOG_WARNING("CMD_ACK: short packet (%u B)", (unsigned)len);
                return false;
            }
            ble_gap_addr_t own;
            sd_ble_gap_addr_get(&own);
            NRF_LOG_INFO("CMD_ACK: gw_mac=%02X:%02X:%02X:%02X",
                         data[1], data[2], data[3], data[4]);
            NRF_LOG_INFO("CMD_ACK: own_mac=%02X:%02X:%02X:%02X",
                         own.addr[3], own.addr[2], own.addr[1], own.addr[0]);
            if (data[1] != own.addr[3] || data[2] != own.addr[2] ||
                data[3] != own.addr[1] || data[4] != own.addr[0])
            {
                NRF_LOG_WARNING("CMD_ACK: MAC mismatch - ignored");
                return false;
            }
            g_node_id = data[5];
            NRF_LOG_INFO("CMD_ACK: OK node_id=%u", (unsigned)g_node_id);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_ECG_CFG 0xCF  —  6 bytes
         * [CMD][node_id][freq_lo][freq_hi][int_lo][int_hi]
         * ----------------------------------------------------------- */
        case CMD_ECG_CFG:
        {
            if (len != 6)
            {
                NRF_LOG_WARNING("CMD_ECG_CFG: bad len %u (expect 6)", (unsigned)len);
                return false;
            }
            if (data[1] != g_node_id)
            {
                NRF_LOG_WARNING("CMD_ECG_CFG: node_id mismatch %u vs %u",
                                (unsigned)data[1], (unsigned)g_node_id);
                return false;
            }

            uint16_t freq_hz     = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
            uint16_t interval_ms = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
            if (freq_hz == 0 || interval_ms == 0)
            {
                NRF_LOG_WARNING("CMD_ECG_CFG: zero freq or interval");
                return false;
            }

            uint32_t us  = 1000000UL / freq_hz;
            uint16_t pkt = (uint16_t)((uint32_t)freq_hz * interval_ms / 1000);
            if (pkt == 0) { pkt = 1; }
            if (pkt > max_samples) { pkt = max_samples; }

            g_cmd_sample_us   = us;
            g_cmd_pkt_samples = pkt;
            g_cmd_cfg_pending = true;

            NRF_LOG_INFO("CMD_ECG_CFG: OK %u Hz, %u ms -> %u smp/pkt",
                         (unsigned)freq_hz, (unsigned)interval_ms, (unsigned)pkt);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_THR 0xCE  —  32 bytes
         *
         * [0]  0xCE
         * [1]  node_id
         * [2-7]   PPG HR  norm_min norm_max warn_min warn_max dang_min dang_max  (uint8)
         * [8-13]  ECG HR  same layout                                             (uint8)
         * [14-19] SpO2    same layout                                             (uint8)
         * [20-31] Temp    norm_min norm_max warn_min warn_max dang_min dang_max  (uint16 LE ×10)
         * ----------------------------------------------------------- */
        case CMD_THR:
        {
            if (len != 32)
            {
                NRF_LOG_WARNING("CMD_THR: bad len %u (expect 32)", (unsigned)len);
                return false;
            }
            if (data[1] != g_node_id)
            {
                NRF_LOG_WARNING("CMD_THR: node_id mismatch %u vs %u",
                                (unsigned)data[1], (unsigned)g_node_id);
                return false;
            }

            /* PPG HR */
            g_thr_ppg_norm_min = data[2];
            g_thr_ppg_norm_max = data[3];
            g_thr_ppg_warn_min = data[4];
            g_thr_ppg_warn_max = data[5];
            g_thr_ppg_dang_min = data[6];
            g_thr_ppg_dang_max = data[7];

            /* ECG HR */
            g_thr_ecg_norm_min = data[8];
            g_thr_ecg_norm_max = data[9];
            g_thr_ecg_warn_min = data[10];
            g_thr_ecg_warn_max = data[11];
            g_thr_ecg_dang_min = data[12];
            g_thr_ecg_dang_max = data[13];

            /* SpO2 */
            g_thr_spo2_norm_min = data[14];
            g_thr_spo2_norm_max = data[15];
            g_thr_spo2_warn_min = data[16];
            g_thr_spo2_warn_max = data[17];
            g_thr_spo2_dang_min = data[18];
            g_thr_spo2_dang_max = data[19];

            /* Temperature (uint16 LE, ×10) */
            g_thr_temp_norm_min = (uint16_t)data[20] | ((uint16_t)data[21] << 8);
            g_thr_temp_norm_max = (uint16_t)data[22] | ((uint16_t)data[23] << 8);
            g_thr_temp_warn_min = (uint16_t)data[24] | ((uint16_t)data[25] << 8);
            g_thr_temp_warn_max = (uint16_t)data[26] | ((uint16_t)data[27] << 8);
            g_thr_temp_dang_min = (uint16_t)data[28] | ((uint16_t)data[29] << 8);
            g_thr_temp_dang_max = (uint16_t)data[30] | ((uint16_t)data[31] << 8);

            /* 2 args per call = 1 pool element each (pool element holds timestamp + 2 args) */
            NRF_LOG_INFO("CMD_THR ppgHR norm=%u-%u", data[2],  data[3]);
            NRF_LOG_INFO("CMD_THR ppgHR warn=%u-%u", data[4],  data[5]);
            NRF_LOG_INFO("CMD_THR ppgHR dang=%u-%u", data[6],  data[7]);
            NRF_LOG_INFO("CMD_THR ecgHR norm=%u-%u", data[8],  data[9]);
            NRF_LOG_INFO("CMD_THR ecgHR warn=%u-%u", data[10], data[11]);
            NRF_LOG_INFO("CMD_THR ecgHR dang=%u-%u", data[12], data[13]);
            NRF_LOG_INFO("CMD_THR spo2  norm=%u-%u", data[14], data[15]);
            NRF_LOG_INFO("CMD_THR spo2  warn=%u-%u", data[16], data[17]);
            NRF_LOG_INFO("CMD_THR spo2  dang=%u-%u", data[18], data[19]);
            NRF_LOG_INFO("CMD_THR temp  norm=%u.%u-%u.%u",
                         g_thr_temp_norm_min / 10, g_thr_temp_norm_min % 10,
                         g_thr_temp_norm_max / 10, g_thr_temp_norm_max % 10);
            NRF_LOG_INFO("CMD_THR temp  warn=%u.%u-%u.%u",
                         g_thr_temp_warn_min / 10, g_thr_temp_warn_min % 10,
                         g_thr_temp_warn_max / 10, g_thr_temp_warn_max % 10);
            NRF_LOG_INFO("CMD_THR temp  dang=%u.%u-%u.%u",
                         g_thr_temp_dang_min / 10, g_thr_temp_dang_min % 10,
                         g_thr_temp_dang_max / 10, g_thr_temp_dang_max % 10);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_PPG_CFG 0xCD  —  6 bytes
         * [CMD][node_id][freqLo][freqHi][redMa][irMa]
         * freq   : MAX30102 sample rate in Hz (uint16 LE)
         * redMa  : red LED current in mA (uint8)
         * irMa   : IR  LED current in mA (uint8)
         * ----------------------------------------------------------- */
        case CMD_PPG_CFG:
        {
            if (len != 6)
            {
                NRF_LOG_WARNING("CMD_PPG_CFG: bad len %u (expect 6)", (unsigned)len);
                return false;
            }
            if (data[1] != g_node_id)
            {
                NRF_LOG_WARNING("CMD_PPG_CFG: node_id mismatch %u vs %u",
                                (unsigned)data[1], (unsigned)g_node_id);
                return false;
            }

            uint16_t freq = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
            if (freq == 0)
            {
                NRF_LOG_WARNING("CMD_PPG_CFG: zero sample freq");
                return false;
            }

            g_ppg_sample_freq  = freq;
            g_ppg_red_ma       = data[4];
            g_ppg_ir_ma        = data[5];
            g_ppg_cfg_pending  = true;

            NRF_LOG_INFO("CMD_PPG_CFG: OK %u Hz red=%u mA ir=%u mA",
                         (unsigned)freq, (unsigned)data[4], (unsigned)data[5]);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_VITAL_CFG 0xCC  —  4 bytes
         * [CMD][node_id][intervalLo][intervalHi]
         * interval : vital reporting period in ms (uint16 LE)
         * ----------------------------------------------------------- */
        case CMD_VITAL_CFG:
        {
            if (len != 4)
            {
                NRF_LOG_WARNING("CMD_VITAL_CFG: bad len %u (expect 4)", (unsigned)len);
                return false;
            }
            if (data[1] != g_node_id)
            {
                NRF_LOG_WARNING("CMD_VITAL_CFG: node_id mismatch %u vs %u",
                                (unsigned)data[1], (unsigned)g_node_id);
                return false;
            }

            uint16_t interval = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
            if (interval == 0)
            {
                NRF_LOG_WARNING("CMD_VITAL_CFG: zero interval");
                return false;
            }

            g_vital_interval_ms  = interval;
            g_vital_cfg_pending  = true;

            NRF_LOG_INFO("CMD_VITAL_CFG: OK %u ms", (unsigned)interval);
            return true;
        }

        default:
            NRF_LOG_WARNING("GW RX: unknown cmd=0x%02X", (unsigned)data[0]);
            return false;
    }
}
