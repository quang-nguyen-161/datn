#ifndef CMD_H
#define CMD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Wire command bytes (gateway -> node).
 * ----------------------------------------------------------------------- */
#define CMD_ACK       0xA0  /* [CMD][addr_b2..b5][node_id]                         6 B  */
#define CMD_ECG_CFG   0xCF  /* [CMD][node_id][freq_lo][freq_hi][int_lo][int_hi]      6 B  */
#define CMD_THR       0xCE  /* [CMD][node_id][ppg×6][ecg×6][spo2×6][temp×12]       32 B  */
#define CMD_PPG_CFG   0xCD  /* [CMD][node_id][freqLo][freqHi][redMa][irMa]           6 B  */
#define CMD_VITAL_CFG 0xCC  /* [CMD][node_id][intervalLo][intervalHi]                4 B  */

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
extern volatile uint8_t  g_ppg_red_ma;       /* mA  — default 6    */
extern volatile uint8_t  g_ppg_ir_ma;        /* mA  — default 6    */

/* -----------------------------------------------------------------------
 * Vital reporting interval  (CMD_VITAL_CFG)
 * ----------------------------------------------------------------------- */
extern volatile bool     g_vital_cfg_pending;
extern volatile uint16_t g_vital_interval_ms; /* ms  — default 1000 */

/* -----------------------------------------------------------------------
 * Node identity.  0xFF = not yet assigned via CMD_ACK.
 * ----------------------------------------------------------------------- */
extern volatile uint8_t  g_node_id;

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
