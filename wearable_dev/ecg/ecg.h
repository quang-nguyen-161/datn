#ifndef ECG_H
#define ECG_H

#include <stdint.h>
#include <stdbool.h>

/* ── Result struct — ECG signal state (updated by ecg_process each sample) ── */
typedef struct {
    int16_t  raw;         /* raw 12-bit SAADC value            */
    float    filtered;    /* bandpass → ALE-NLMS → Savitzky-Golay output */
    bool     lead_off;    /* true when electrode is off         */
    bool     new_data;    /* cleared by consumer               */
} ecg_result_t;

extern ecg_result_t g_ecg;

/* ── Legacy ISR globals (still needed by ISR) ── */
extern volatile int16_t g_ecg_raw;
extern volatile bool    g_ecg_ready;

/* ── Init ── */
void ecg_init(void);        /* call after ble_app_init(); arms TIMER3→PPI→SAADC at 250 Hz */

/* ── Signal processing ── */
float ecg_process(int16_t raw);   /* bandpass(1–25 Hz) → ALE-NLMS → Savitzky-Golay; returns filtered sample */

/* ── BLE packet ── */
#define ECG_PACKET_SAMPLES  50    /* default samples per BLE notification (50 × int16 = 100 bytes) */
#define ECG_MAX_SAMPLES     100   /* max samples per BLE notification (100 × int16 = 200 bytes, fits ATT ~238 B) */
#define ECG_BUF_SAMPLES     500   /* max accumulation buffer; batches >ECG_MAX_SAMPLES split into multi-notify */

#endif /* ECG_H */
