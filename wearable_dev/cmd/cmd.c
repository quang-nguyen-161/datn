#include "cmd.h"
#include "ble_gap.h"
#include "flash_user.h"
#include "device_mode.h"
#undef NRF_LOG_MODULE_NAME
#define NRF_LOG_MODULE_NAME cmd
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* -----------------------------------------------------------------------
 * ECG reconfiguration — defaults: 250 Hz, 200 ms batches
 * ----------------------------------------------------------------------- */
volatile bool     g_cmd_cfg_pending  = false;
volatile uint32_t g_cmd_sample_us    = 4000U;
volatile uint16_t g_cmd_pkt_samples  = 50U;

/* PPG (MAX30102) reconfiguration — defaults from PAYLOAD_CONFIG §9 */
volatile bool     g_ppg_cfg_pending  = false;
volatile uint16_t g_ppg_sample_freq  = 100U;  /* Hz */
volatile uint8_t  g_ppg_red_ma       = 6U;    /* mA */
volatile uint8_t  g_ppg_ir_ma        = 6U;    /* mA */

/* Vital reporting interval */
volatile bool     g_vital_cfg_pending = false;
volatile uint16_t g_vital_interval_ms = 1000U; /* ms */

/* ECG streaming flag — gates BLE ECG batch send (see cmd.h) */
volatile bool     g_ecg_stream_enabled = true;

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

    NRF_LOG_INFO("GW RX: cmd=0x%02X len=%u", (unsigned)data[0], (unsigned)len);

    switch (data[0])
    {
        /* -----------------------------------------------------------
         * CMD_ECG_CFG 0xCF  —  5 bytes
         * [CMD][freq_lo][freq_hi][int_lo][int_hi]
         * ----------------------------------------------------------- */
        case CMD_ECG_CFG:
        {
            if (len != 5)
            {
                NRF_LOG_WARNING("CMD_ECG_CFG: bad len %u (expect 5)", (unsigned)len);
                return false;
            }

            uint16_t freq_hz     = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            uint16_t interval_ms = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
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
            flash_user_mark_dirty();

            NRF_LOG_INFO("CMD_ECG_CFG: OK %u Hz, %u ms -> %u smp/pkt",
                         (unsigned)freq_hz, (unsigned)interval_ms, (unsigned)pkt);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_THR 0xCE  —  31 bytes
         *
         * [0]  0xCE
         * [1-6]   PPG HR  norm_min norm_max warn_min warn_max dang_min dang_max  (uint8)
         * [7-12]  ECG HR  same layout                                             (uint8)
         * [13-18] SpO2    same layout                                             (uint8)
         * [19-30] Temp    norm_min norm_max warn_min warn_max dang_min dang_max  (uint16 LE ×10)
         * ----------------------------------------------------------- */
        case CMD_THR:
        {
            if (len != 31)
            {
                NRF_LOG_WARNING("CMD_THR: bad len %u (expect 31)", (unsigned)len);
                return false;
            }

            /* PPG HR */
            g_thr_ppg_norm_min = data[1];
            g_thr_ppg_norm_max = data[2];
            g_thr_ppg_warn_min = data[3];
            g_thr_ppg_warn_max = data[4];
            g_thr_ppg_dang_min = data[5];
            g_thr_ppg_dang_max = data[6];

            /* ECG HR */
            g_thr_ecg_norm_min = data[7];
            g_thr_ecg_norm_max = data[8];
            g_thr_ecg_warn_min = data[9];
            g_thr_ecg_warn_max = data[10];
            g_thr_ecg_dang_min = data[11];
            g_thr_ecg_dang_max = data[12];

            /* SpO2 */
            g_thr_spo2_norm_min = data[13];
            g_thr_spo2_norm_max = data[14];
            g_thr_spo2_warn_min = data[15];
            g_thr_spo2_warn_max = data[16];
            g_thr_spo2_dang_min = data[17];
            g_thr_spo2_dang_max = data[18];

            /* Temperature (uint16 LE, ×10) */
            g_thr_temp_norm_min = (uint16_t)data[19] | ((uint16_t)data[20] << 8);
            g_thr_temp_norm_max = (uint16_t)data[21] | ((uint16_t)data[22] << 8);
            g_thr_temp_warn_min = (uint16_t)data[23] | ((uint16_t)data[24] << 8);
            g_thr_temp_warn_max = (uint16_t)data[25] | ((uint16_t)data[26] << 8);
            g_thr_temp_dang_min = (uint16_t)data[27] | ((uint16_t)data[28] << 8);
            g_thr_temp_dang_max = (uint16_t)data[29] | ((uint16_t)data[30] << 8);

            flash_user_mark_dirty();

            NRF_LOG_INFO("CMD_THR ppgHR norm=%u-%u", data[1],  data[2]);
            NRF_LOG_INFO("CMD_THR ppgHR warn=%u-%u", data[3],  data[4]);
            NRF_LOG_INFO("CMD_THR ppgHR dang=%u-%u", data[5],  data[6]);
            NRF_LOG_INFO("CMD_THR ecgHR norm=%u-%u", data[7],  data[8]);
            NRF_LOG_INFO("CMD_THR ecgHR warn=%u-%u", data[9],  data[10]);
            NRF_LOG_INFO("CMD_THR ecgHR dang=%u-%u", data[11], data[12]);
            NRF_LOG_INFO("CMD_THR spo2  norm=%u-%u", data[13], data[14]);
            NRF_LOG_INFO("CMD_THR spo2  warn=%u-%u", data[15], data[16]);
            NRF_LOG_INFO("CMD_THR spo2  dang=%u-%u", data[17], data[18]);
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
         * CMD_PPG_CFG 0xCD  —  5 bytes
         * [CMD][freqLo][freqHi][redMa][irMa]
         * freq   : MAX30102 sample rate in Hz (uint16 LE)
         * redMa  : red LED current in mA (uint8)
         * irMa   : IR  LED current in mA (uint8)
         * ----------------------------------------------------------- */
        case CMD_PPG_CFG:
        {
            if (len != 5)
            {
                NRF_LOG_WARNING("CMD_PPG_CFG: bad len %u (expect 5)", (unsigned)len);
                return false;
            }

            uint16_t freq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            if (freq == 0)
            {
                NRF_LOG_WARNING("CMD_PPG_CFG: zero sample freq");
                return false;
            }

            g_ppg_sample_freq  = freq;
            g_ppg_red_ma       = data[3];
            g_ppg_ir_ma        = data[4];
            g_ppg_cfg_pending  = true;
            flash_user_mark_dirty();

            NRF_LOG_INFO("CMD_PPG_CFG: OK %u Hz red=%u mA ir=%u mA",
                         (unsigned)freq, (unsigned)data[3], (unsigned)data[4]);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_VITAL_CFG 0xCC  —  3 bytes
         * [CMD][intervalLo][intervalHi]
         * interval : vital reporting period in ms (uint16 LE)
         * ----------------------------------------------------------- */
        case CMD_VITAL_CFG:
        {
            if (len != 3)
            {
                NRF_LOG_WARNING("CMD_VITAL_CFG: bad len %u (expect 3)", (unsigned)len);
                return false;
            }

            uint16_t interval = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            if (interval == 0)
            {
                NRF_LOG_WARNING("CMD_VITAL_CFG: zero interval");
                return false;
            }

            g_vital_interval_ms  = interval;
            g_vital_cfg_pending  = true;
            flash_user_mark_dirty();

            NRF_LOG_INFO("CMD_VITAL_CFG: OK %u ms", (unsigned)interval);
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_MODE_CFG 0xCB  —  7 bytes
         * [CMD][mode][periodSecLo][periodSecHi][capSecLo][capSecHi][ecgEnabled]
         * mode       : 0=CONTINUOUS 1=PERIODIC 2=ECG
         * period     : PERIODIC wake-to-wake interval, seconds (uint16 LE)
         * capture    : PERIODIC measurement window, seconds (uint16 LE)
         * ecgEnabled : 0=no ECG batches, 1=stream ECG alongside vitals
         * ----------------------------------------------------------- */
        case CMD_MODE_CFG:
        {
            if (len != 7)
            {
                NRF_LOG_WARNING("CMD_MODE_CFG: bad len %u (expect 7)", (unsigned)len);
                return false;
            }

            uint8_t  mode    = data[1];
            uint16_t period  = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
            uint16_t capture = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
            if (mode >= MODE_COUNT)
            {
                NRF_LOG_WARNING("CMD_MODE_CFG: bad mode %u", (unsigned)mode);
                return false;
            }

            /* Set period + capture first (clamped inside), then switch mode last
             * so the phase machine recomputes against the new values. */
            device_mode_set_period(period);
            device_mode_set_capture(capture);
            device_mode_set((device_mode_t)mode);

            g_ecg_stream_enabled = (data[6] != 0);
            flash_user_mark_dirty();

            NRF_LOG_INFO("CMD_MODE_CFG: mode=%u period=%u ms cap=%u ms ecg=%u",
                         (unsigned)mode, (unsigned)g_period_ms, (unsigned)g_capture_ms,
                         (unsigned)g_ecg_stream_enabled);
            return true;
        }

        default:
            NRF_LOG_WARNING("GW RX: unknown cmd=0x%02X", (unsigned)data[0]);
            return false;
    }
}
