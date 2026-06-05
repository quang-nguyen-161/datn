#ifndef DASHBOARD_H__
#define DASHBOARD_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * Dashboard — GC9A01 240x240 display module (Layout V2)
 *
 * Layout:
 *   Row 1: BLE Status (device name/MAC + RSSI + signal bars)
 *   Row 2: Temperature (left) + SpO2 (right)
 *   Row 3: ECG waveform (left 55%) + HR number & sweep (right 45%)
 *   Row 4: Steps + Activity + Distance + Progress
 */

/* ── Sensor data struct for display ── */
typedef struct {
    float    hr;
    float    spo2;
    bool     hr_valid;
    float    temperature;
    bool     temp_valid;
    uint32_t steps;
    float    cadence;
    uint32_t timestamp_ms;

    /* BLE status — Row 1 */
    char     device_name[16];   /* Tên từ gateway, hoặc "" nếu chưa có  */
    uint8_t  mac[6];            /* MAC address (fallback khi chưa có tên) */
    int8_t   rssi;              /* RSSI dBm (-30 đến -100), 0=disconnected */
    bool     ble_connected;     /* true khi đã kết nối gateway */
} dashboard_data_t;

/* ── API ── */

/** Show splash screen (blocks ~2s with delay) */
void dashboard_splash(void);

/** Draw static dashboard layout */
void dashboard_init_layout(void);

/** Update BLE status bar: device name/MAC + RSSI + signal icon */
void dashboard_update_ble_status(const dashboard_data_t *d);

/** Update HR + SpO2 display (number + sweep chart in Row 3 right) */
void dashboard_update_hr(const dashboard_data_t *d);

/** Update temperature display (number + sweep chart) */
void dashboard_update_temp(const dashboard_data_t *d);

/** Update ECG sweep line (Row 3 left, call at ~8 FPS) */
void dashboard_update_ecg(const dashboard_data_t *d, uint16_t ecg_val);

/** Update steps + activity + distance + progress bar */
void dashboard_update_steps(const dashboard_data_t *d, float ac_value);

#endif /* DASHBOARD_H__ */
