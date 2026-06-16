#include "cmd.h"
#include "ble_gap.h"
#include "flash_user.h"
#include "device_mode.h"
#include <stdio.h>
#include <string.h>
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
volatile uint8_t  g_ppg_hr_source    = 0U;    /* 0=IR, 1=RED */

/* Vital reporting interval */
volatile bool     g_vital_cfg_pending = false;
volatile uint16_t g_vital_interval_ms = 1000U; /* ms */

/* LCD dashboard refresh interval (CONTINUOUS mode) */
volatile uint16_t g_lcd_interval_ms   = 1000U; /* ms */

/* ECG streaming flag — gates BLE ECG batch send (see cmd.h) */
volatile bool     g_ecg_stream_enabled = true;

/* Patient name — shown on LCD Row 1 line 2 instead of the BLE address
 * once the gateway sends it (CMD_NAME_CFG) */
volatile char     g_patient_name[16] = "";

/* Config-update splash notification (see cmd.h) */
volatile bool     g_cmd_update_pending = false;
volatile char     g_cmd_update_msg[20] = "";
volatile char     g_cmd_update_val[180] = "";

/* Copies a title + detail string into g_cmd_update_msg/_val and raises
 * g_cmd_update_pending. */
static void notify_update(const char *msg, const char *val)
{
    uint8_t i = 0;
    for (; msg[i] != '\0' && i < sizeof(g_cmd_update_msg) - 1; i++)
    {
        g_cmd_update_msg[i] = msg[i];
    }
    g_cmd_update_msg[i] = '\0';

    for (i = 0; val[i] != '\0' && i < sizeof(g_cmd_update_val) - 1; i++)
    {
        g_cmd_update_val[i] = val[i];
    }
    g_cmd_update_val[i] = '\0';

    g_cmd_update_pending = true;
}

/* PPG heart rate thresholds (bpm) */
volatile uint8_t  g_thr_ppg_norm_min = 60;
volatile uint8_t  g_thr_ppg_norm_max = 100;
volatile uint8_t  g_thr_ppg_warn_min = 50;
volatile uint8_t  g_thr_ppg_warn_max = 120;
volatile uint8_t  g_thr_ppg_dang_min = 40;
volatile uint8_t  g_thr_ppg_dang_max = 130;

/* ECG heart rate thresholds (bpm) */
volatile uint8_t  g_thr_ecg_norm_min = 60;
volatile uint8_t  g_thr_ecg_norm_max = 100;
volatile uint8_t  g_thr_ecg_warn_min = 50;
volatile uint8_t  g_thr_ecg_warn_max = 120;
volatile uint8_t  g_thr_ecg_dang_min = 40;
volatile uint8_t  g_thr_ecg_dang_max = 130;

/* SpO2 thresholds (%) */
volatile uint8_t  g_thr_spo2_norm_min = 95;
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
            {
                char val[180];
                snprintf(val, sizeof(val), "FS %uHz\nPKT %ums", (unsigned)freq_hz, (unsigned)interval_ms);
                notify_update("ECG Config", val);
            }
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

            uint16_t temp_norm_min = (uint16_t)data[19] | ((uint16_t)data[20] << 8);
            uint16_t temp_norm_max = (uint16_t)data[21] | ((uint16_t)data[22] << 8);
            uint16_t temp_warn_min = (uint16_t)data[23] | ((uint16_t)data[24] << 8);
            uint16_t temp_warn_max = (uint16_t)data[25] | ((uint16_t)data[26] << 8);
            uint16_t temp_dang_min = (uint16_t)data[27] | ((uint16_t)data[28] << 8);
            uint16_t temp_dang_max = (uint16_t)data[29] | ((uint16_t)data[30] << 8);

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
            g_thr_temp_norm_min = temp_norm_min;
            g_thr_temp_norm_max = temp_norm_max;
            g_thr_temp_warn_min = temp_warn_min;
            g_thr_temp_warn_max = temp_warn_max;
            g_thr_temp_dang_min = temp_dang_min;
            g_thr_temp_dang_max = temp_dang_max;

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
            {
                char val[180];
                snprintf(val, sizeof(val),
                         "PPG N%u-%u W%u-%u D%u-%u\n"
                         "ECG N%u-%u W%u-%u D%u-%u\n"
                         "SPO2 N%u-%u W%u-%u D%u-%u\n"
                         "TEMP N%u.%u-%u.%u\n"
                         "W%u.%u-%u.%u D%u.%u-%u.%u",
                         (unsigned)g_thr_ppg_norm_min,  (unsigned)g_thr_ppg_norm_max,
                         (unsigned)g_thr_ppg_warn_min,  (unsigned)g_thr_ppg_warn_max,
                         (unsigned)g_thr_ppg_dang_min,  (unsigned)g_thr_ppg_dang_max,
                         (unsigned)g_thr_ecg_norm_min,  (unsigned)g_thr_ecg_norm_max,
                         (unsigned)g_thr_ecg_warn_min,  (unsigned)g_thr_ecg_warn_max,
                         (unsigned)g_thr_ecg_dang_min,  (unsigned)g_thr_ecg_dang_max,
                         (unsigned)g_thr_spo2_norm_min, (unsigned)g_thr_spo2_norm_max,
                         (unsigned)g_thr_spo2_warn_min, (unsigned)g_thr_spo2_warn_max,
                         (unsigned)g_thr_spo2_dang_min, (unsigned)g_thr_spo2_dang_max,
                         (unsigned)(g_thr_temp_norm_min / 10), (unsigned)(g_thr_temp_norm_min % 10),
                         (unsigned)(g_thr_temp_norm_max / 10), (unsigned)(g_thr_temp_norm_max % 10),
                         (unsigned)(g_thr_temp_warn_min / 10), (unsigned)(g_thr_temp_warn_min % 10),
                         (unsigned)(g_thr_temp_warn_max / 10), (unsigned)(g_thr_temp_warn_max % 10),
                         (unsigned)(g_thr_temp_dang_min / 10), (unsigned)(g_thr_temp_dang_min % 10),
                         (unsigned)(g_thr_temp_dang_max / 10), (unsigned)(g_thr_temp_dang_max % 10));
                notify_update("Thresholds", val);
            }
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_PPG_CFG 0xCD  —  3 or 4 bytes
         * [CMD][freqLo][freqHi][hrSrc]
         * freq   : MAX30102 sample rate in Hz (uint16 LE)
         * hrSrc  : LED channel for HR peak detection — 0=IR, 1=RED (uint8, optional, default unchanged)
         * LED current is no longer configurable — it's adapted automatically
         * toward a target raw ADC level (see drivers/ppg/max.c).
         * ----------------------------------------------------------- */
        case CMD_PPG_CFG:
        {
            if (len != 3 && len != 4)
            {
                NRF_LOG_WARNING("CMD_PPG_CFG: bad len %u (expect 3 or 4)", (unsigned)len);
                return false;
            }

            uint16_t freq = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            if (freq == 0)
            {
                NRF_LOG_WARNING("CMD_PPG_CFG: zero sample freq");
                return false;
            }

            g_ppg_sample_freq  = freq;
            g_ppg_hr_source    = (len == 4) ? (data[3] ? 1U : 0U) : g_ppg_hr_source;
            g_ppg_cfg_pending  = true;
            flash_user_mark_dirty();

            NRF_LOG_INFO("CMD_PPG_CFG: OK %u Hz hrSrc=%s",
                         (unsigned)freq, g_ppg_hr_source ? "RED" : "IR");
            {
                char val[180];
                snprintf(val, sizeof(val), "FS %uHz\nHR %s",
                         (unsigned)freq, g_ppg_hr_source ? "RED" : "IR");
                notify_update("PPG Config", val);
            }
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_VITAL_CFG 0xCC  —  3 or 5 bytes
         * [CMD][intervalLo][intervalHi][lcdIntLo][lcdIntHi]
         * interval : vital reporting period in ms (uint16 LE)
         * lcdInterval (optional) : LCD dashboard refresh period in ms (uint16 LE)
         * ----------------------------------------------------------- */
        case CMD_VITAL_CFG:
        {
            if (len != 3 && len != 5)
            {
                NRF_LOG_WARNING("CMD_VITAL_CFG: bad len %u (expect 3 or 5)", (unsigned)len);
                return false;
            }

            uint16_t interval = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            if (interval == 0)
            {
                NRF_LOG_WARNING("CMD_VITAL_CFG: zero interval");
                return false;
            }

            uint16_t lcd_interval = g_lcd_interval_ms;
            if (len == 5)
            {
                lcd_interval = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
                if (lcd_interval == 0)
                {
                    NRF_LOG_WARNING("CMD_VITAL_CFG: zero lcd interval");
                    return false;
                }
            }

            g_vital_interval_ms  = interval;
            g_lcd_interval_ms    = lcd_interval;
            g_vital_cfg_pending  = true;
            flash_user_mark_dirty();

            NRF_LOG_INFO("CMD_VITAL_CFG: OK %u ms, lcd %u ms", (unsigned)interval, (unsigned)lcd_interval);
            {
                char val[24];
                snprintf(val, sizeof(val), "%u ms / %u ms", (unsigned)interval, (unsigned)lcd_interval);
                notify_update("Vital Interval", val);
            }
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
            {
                static const char *mode_name[MODE_COUNT] = { "Continuous", "Periodic", "ECG" };
                char val[180];
                if ((device_mode_t)mode == MODE_PERIODIC)
                {
                    snprintf(val, sizeof(val), "MODE %s\nPERIOD %us\nCAPTURE %us\nECG %s",
                             mode_name[mode], (unsigned)(g_period_ms / 1000), (unsigned)(g_capture_ms / 1000),
                             g_ecg_stream_enabled ? "ON" : "OFF");
                }
                else
                {
                    snprintf(val, sizeof(val), "MODE %s\nECG %s",
                             mode_name[mode], g_ecg_stream_enabled ? "ON" : "OFF");
                }
                notify_update("Mode Config", val);
            }
            return true;
        }

        /* -----------------------------------------------------------
         * CMD_NAME_CFG 0xC9  —  2-17 bytes
         * [CMD][len][name bytes...]
         * len : number of name bytes that follow (0-15)
         * ----------------------------------------------------------- */
        case CMD_NAME_CFG:
        {
            if (len < 2)
            {
                NRF_LOG_WARNING("CMD_NAME_CFG: bad len %u (expect >=2)", (unsigned)len);
                return false;
            }

            uint8_t name_len = data[1];
            if (name_len > 15) { name_len = 15; }
            if ((uint16_t)(2 + name_len) > len)
            {
                NRF_LOG_WARNING("CMD_NAME_CFG: name_len %u exceeds packet len %u",
                                (unsigned)name_len, (unsigned)len);
                return false;
            }

            for (uint8_t i = 0; i < name_len; i++)
            {
                g_patient_name[i] = (char)data[2 + i];
            }
            g_patient_name[name_len] = '\0';

            NRF_LOG_INFO("CMD_NAME_CFG: OK \"%s\"", (const char *)g_patient_name);
            notify_update("Patient Name", (const char *)g_patient_name);
            return true;
        }

        default:
            NRF_LOG_WARNING("GW RX: unknown cmd=0x%02X", (unsigned)data[0]);
            return false;
    }
}
