#ifndef DASHBOARD_H__
#define DASHBOARD_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * Dashboard — GC9A01 240x240 display module (Layout V3)
 *
 * Layout:
 *   Row 1: BLE Status (device name/MAC + RSSI + signal bars) + battery icon
 *   Row 2: Temperature (left) + SpO2 (right)
 *   Row 3: Steps + Activity (left) + HR number & sweep (right)
 *   Row 4: ECG waveform (full width) + ON/OFF availability badge
 */

/* ── Sensor data struct for display ── */
typedef struct {
    float    hr;
    float    spo2;
    bool     hr_valid;
    bool     ppg_saturated;     /* true when the PPG HR-source channel (IR/RED, per
                                  * g_ppg_hr_source) is saturated — blinks "SAT" badge */
    bool     ppg_calibrating;   /* true while the adaptive LED current loop is
                                  * converging — HR/SpO2 show "Cal" instead of "--" */
    float    temperature;
    bool     temp_valid;
    uint32_t steps;
    bool     steps_valid;       /* false when accelerometer absent — shows "--" */
    float    cadence;
    uint32_t timestamp_ms;

    /* BLE status — Row 1 */
    char     device_name[16];   /* Patient name from gateway (CMD_NAME_CFG); shown on
                                  * line 2 when ble_connected, else "" => show mac */
    uint8_t  mac[6];            /* BLE address (line 2 fallback when no patient name) */
    int8_t   rssi;              /* RSSI dBm (-30 đến -100), 0=disconnected */
    bool     ble_connected;     /* true khi đã kết nối gateway */

    /* Battery — Row 1 (placeholder until ADC battery reading is wired up) */
    bool     battery_valid;     /* true once a real reading exists */
    uint8_t  battery_pct;       /* 0-100, only meaningful when battery_valid */

    /* ECG stream enable/disable — Row 4 badge + sweep gating */
    bool     ecg_enabled;       /* mirrors g_ecg_stream_enabled */
} dashboard_data_t;

/* ── API ── */

/** Show splash screen (blocks ~2s with delay) */
void dashboard_splash(void);

/** Draw static dashboard layout */
void dashboard_init_layout(void);

/** Full-screen "config updated" splash with bold centered title + value, ~1s (blocking) */
void dashboard_show_update_splash(const char *title, const char *val);

/** Update BLE status bar: device name/MAC + RSSI + signal icon */
void dashboard_update_ble_status(const dashboard_data_t *d);

/** Update HR + SpO2 display (number + sweep chart in Row 3 right) */
void dashboard_update_hr(const dashboard_data_t *d);

/** Update "SAT!" blink badge (call every PPG sample for smooth ~2.5Hz blink) */
void dashboard_update_sat_badge(const dashboard_data_t *d);

/** Update temperature display (number + sweep chart) */
void dashboard_update_temp(const dashboard_data_t *d);

/** Update ECG ON/OFF badge + sweep line (Row 4, call at ~8 FPS) */
void dashboard_update_ecg(const dashboard_data_t *d, uint16_t ecg_val);

/** Update steps + activity (Row 3 left, compact) */
void dashboard_update_steps(const dashboard_data_t *d, float ac_value);

/** Update battery icon (Row 1, outline placeholder until battery_valid) */
void dashboard_update_battery(const dashboard_data_t *d);

#endif /* DASHBOARD_H__ */
