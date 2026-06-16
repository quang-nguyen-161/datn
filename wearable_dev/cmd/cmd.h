#ifndef CMD_H
#define CMD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Wire command bytes (gateway -> node).
 * conn_handle on the central identifies the target — no node_id byte needed.
 * ----------------------------------------------------------------------- */
#define CMD_ECG_CFG   0xCF  /* [CMD][freq_lo][freq_hi][int_lo][int_hi]               5 B  */
#define CMD_THR       0xCE  /* [CMD][ppg×6][ecg×6][spo2×6][temp×12]               31 B  */
#define CMD_PPG_CFG   0xCD  /* [CMD][freqLo][freqHi][hrSrc]                         4 B  (hrSrc optional, default 3 B) */
#define CMD_VITAL_CFG 0xCC  /* [CMD][intervalLo][intervalHi][lcdIntLo][lcdIntHi]     5 B  (lcd fields optional, default 3 B) */
#define CMD_MODE_CFG  0xCB  /* [CMD][mode][periodSecLo][periodSecHi][capSecLo][capSecHi][ecgEnabled]  7 B  */
#define CMD_NAME_CFG  0xC9  /* [CMD][len][name bytes...]                                     2-17 B  */

/* -----------------------------------------------------------------------
 * ECG reconfiguration  (CMD_ECG_CFG)
 * ----------------------------------------------------------------------- */
extern volatile bool     g_cmd_cfg_pending;
extern volatile uint32_t g_cmd_sample_us;
extern volatile uint16_t g_cmd_pkt_samples;

/* -----------------------------------------------------------------------
 * PPG (MAX30102) reconfiguration  (CMD_PPG_CFG)
 * ----------------------------------------------------------------------- */
extern volatile bool     g_ppg_cfg_pending;
extern volatile uint16_t g_ppg_sample_freq;  /* Hz  — default 100  */
extern volatile uint8_t  g_ppg_hr_source;    /* 0=IR, 1=RED — default 0 (IR) */

/* -----------------------------------------------------------------------
 * Vital reporting interval  (CMD_VITAL_CFG)
 * ----------------------------------------------------------------------- */
extern volatile bool     g_vital_cfg_pending;
extern volatile uint16_t g_vital_interval_ms; /* ms  — default 1000 */

/* -----------------------------------------------------------------------
 * LCD dashboard refresh interval (CONTINUOUS mode), set via CMD_VITAL_CFG
 * bytes 4-5. Independent of g_vital_interval_ms (BLE vitals send cadence).
 * ----------------------------------------------------------------------- */
extern volatile uint16_t g_lcd_interval_ms;   /* ms  — default 1000 */

/* -----------------------------------------------------------------------
 * ECG streaming flag  (CMD_MODE_CFG byte 6)
 *   true  → stream ECG batches over BLE alongside vitals
 *   false → process ECG (HR / LCD) but send no ECG batches
 * ----------------------------------------------------------------------- */
extern volatile bool     g_ecg_stream_enabled; /* default true */

/* -----------------------------------------------------------------------
 * Patient name  (CMD_NAME_CFG) — shown on the LCD BLE status row (line 2)
 * in place of the device's BLE address while connected.
 * ----------------------------------------------------------------------- */
extern volatile char     g_patient_name[16]; /* "" = not set, show address */

/* -----------------------------------------------------------------------
 * Config-update splash notification — set by cmd_rx_handle() whenever any
 * CMD_* config command is applied; consumed by main.c to show a brief
 * full-screen "Updated" splash on the LCD (dashboard_show_update_splash()).
 * ----------------------------------------------------------------------- */
extern volatile bool    g_cmd_update_pending;
extern volatile char    g_cmd_update_msg[20];   /* title, e.g. "ECG Config"        */
extern volatile char    g_cmd_update_val[180];  /* detail, '\n'-separated lines    */

/* -----------------------------------------------------------------------
 * Vital thresholds — 3 tiers per vital sign.
 *
 * HR / SpO2 : uint8,  raw bpm / percent
 * Temperature: uint16, value × 10  (e.g. 361 = 36.1 °C)
 *
 * Tier colour convention (matches ThingsBoard dashboard):
 *   norm  = green   — acceptable range
 *   warn  = orange  — outside normal, not yet critical
 *   dang  = red     — critical / dangerous
 * ----------------------------------------------------------------------- */

/* PPG heart rate (bpm) */
extern volatile uint8_t  g_thr_ppg_norm_min, g_thr_ppg_norm_max;
extern volatile uint8_t  g_thr_ppg_warn_min, g_thr_ppg_warn_max;
extern volatile uint8_t  g_thr_ppg_dang_min, g_thr_ppg_dang_max;

/* ECG heart rate (bpm) */
extern volatile uint8_t  g_thr_ecg_norm_min, g_thr_ecg_norm_max;
extern volatile uint8_t  g_thr_ecg_warn_min, g_thr_ecg_warn_max;
extern volatile uint8_t  g_thr_ecg_dang_min, g_thr_ecg_dang_max;

/* SpO2 (%) */
extern volatile uint8_t  g_thr_spo2_norm_min, g_thr_spo2_norm_max;
extern volatile uint8_t  g_thr_spo2_warn_min, g_thr_spo2_warn_max;
extern volatile uint8_t  g_thr_spo2_dang_min, g_thr_spo2_dang_max;

/* Temperature (×10 °C, uint16 LE) */
extern volatile uint16_t g_thr_temp_norm_min, g_thr_temp_norm_max;
extern volatile uint16_t g_thr_temp_warn_min, g_thr_temp_warn_max;
extern volatile uint16_t g_thr_temp_dang_min, g_thr_temp_dang_max;

/* -----------------------------------------------------------------------
 * cmd_rx_handle()
 * ----------------------------------------------------------------------- */
bool cmd_rx_handle(const uint8_t *data, uint16_t len, uint16_t max_samples);

#ifdef __cplusplus
}
#endif

#endif /* CMD_H */
