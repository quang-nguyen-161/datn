#ifndef CMD_H
#define CMD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Wire command bytes (gateway -> node).
 * Byte layout and sizes are fixed by PAYLOAD_CONFIG.md §6.
 * ----------------------------------------------------------------------- */
#define CMD_ACK      0xA0   /* [CMD][addr_b2..b5][node_id]                   6 B */
#define CMD_ECG_CFG  0xCF   /* [CMD][node_id][freq_lo][freq_hi][int_lo][int_hi]  6 B */
#define CMD_THR      0xCE   /* [CMD][node_id][ppgHrMin..tempMax]             9 B */

/* -----------------------------------------------------------------------
 * ECG reconfiguration request.
 * Set by cmd_rx_handle() on CMD_ECG_CFG; consumed by ecg_config_apply()
 * in the main loop.
 * ----------------------------------------------------------------------- */
extern volatile bool     g_cmd_cfg_pending;
extern volatile uint32_t g_cmd_sample_us;    /* microseconds per sample */
extern volatile uint16_t g_cmd_pkt_samples;  /* samples per BLE notify  */

/* -----------------------------------------------------------------------
 * Node identity.
 * Assigned by the gateway via CMD_ACK.  0xFF = not yet assigned.
 * All subsequent commands are silently dropped until this is set.
 * ----------------------------------------------------------------------- */
extern volatile uint8_t  g_node_id;

/* -----------------------------------------------------------------------
 * Vital warning thresholds (uint8 units: bpm / % / °C, whole numbers).
 * Updated by CMD_THR.  Defaults match the gateway-side defaults.
 * ----------------------------------------------------------------------- */
extern volatile uint8_t  g_thr_ppg_hr_min;
extern volatile uint8_t  g_thr_ppg_hr_max;
extern volatile uint8_t  g_thr_ecg_hr_min;
extern volatile uint8_t  g_thr_ecg_hr_max;
extern volatile uint8_t  g_thr_spo2_min;
extern volatile uint8_t  g_thr_temp_min;
extern volatile uint8_t  g_thr_temp_max;

/* -----------------------------------------------------------------------
 * cmd_rx_handle()
 *
 * Parse a raw BLE RX write payload and update the shared state above.
 * Call from cus_data_handler() on BLE_CUS_EVT_RX_DATA.
 *
 * data        - raw bytes written to the RX characteristic.
 * len         - number of bytes.
 * max_samples - upper bound on packet sample count (= PACKET_SAMPLES_MAX
 *               from main.c, passed in to avoid a circular dependency).
 *
 * Returns true if the packet was recognised and processed.
 * ----------------------------------------------------------------------- */
bool cmd_rx_handle(const uint8_t *data, uint16_t len, uint16_t max_samples);

#ifdef __cplusplus
}
#endif

#endif /* CMD_H */
