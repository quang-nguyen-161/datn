#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "cus_service.h"

/* ── Single-sensor test mode ──
 * Define exactly one to isolate that sensor for bring-up/debug
 * (BLE/advertising disabled); leave all undefined for normal operation. */
#define TEST_SENSOR_MAX
 //#define TEST_SENSOR_TMP
// #define TEST_SENSOR_ECG

#if defined(TEST_SENSOR_MAX) || defined(TEST_SENSOR_TMP) || defined(TEST_SENSOR_ECG)
    #define SENSOR_TEST_MODE 1
#else
    #define SENSOR_TEST_MODE 0
#endif

#if SENSOR_TEST_MODE
    #ifdef TEST_SENSOR_MAX
        #define ENABLE_MAX 1
    #else
        #define ENABLE_MAX 0
    #endif
    #ifdef TEST_SENSOR_TMP
        #define ENABLE_TMP 1
    #else
        #define ENABLE_TMP 0
    #endif
    #ifdef TEST_SENSOR_ECG
        #define ENABLE_ECG 1
    #else
        #define ENABLE_ECG 0
    #endif
#else
    #define ENABLE_MAX 1
    #define ENABLE_TMP 1
    #define ENABLE_ECG 1
#endif

/* ══════════════════════════════════════════════════════════════
 *  TWI (I2C) — shared bus
 *  Serves: MAX30102 (0x57)  MMA8452Q (0x1C)  TMP117 (0x48)
 *  Instance: TWI1   SCL = P0.29   SDA = P0.28   400 kHz
 * ════════════════════════════════════════════════════════════ */
extern nrf_drv_twi_t  m_twi;
extern volatile bool  m_xfer_done;   /* set true by twi_handler on any completion */
extern volatile bool  m_xfer_error;  /* set true by twi_handler on NACK/error     */

void twi_init(void);
bool twi_wait(void);  /* spin-wait with 200k-cycle timeout; resets bus on timeout */

/* ══════════════════════════════════════════════════════════════
 *  SPI — display bus
 *  Serves: GC9A01 240×240 LCD
 *  Instance: SPI0   8 MHz
 * ════════════════════════════════════════════════════════════ */
extern nrf_drv_spi_t    m_lcd_spi;

void lcd_spi_init(void);        /* defined in drivers/display/GC9A01.c */

/* ══════════════════════════════════════════════════════════════
 *  BLE — stack init, GAP/GATT, advertising, custom service
 *  (init chain called explicitly from main(), wearable_claude-style —
 *   no separate ble_app module)
 * ════════════════════════════════════════════════════════════ */
void ble_stack_init(void);
void gap_params_init(void);
void gatt_init(void);
void services_init(void);
void advertising_init(void);
void conn_params_init(void);
void advertising_start(void);

/* Thin accessors over the BLE connection state */
uint16_t ble_app_conn_handle(void);
bool     ble_app_is_connected(void);
bool     ble_app_ready_to_send(void);   /* connected AND MTU exchange completed */
uint32_t ble_app_send(uint8_t const *data, uint16_t len);

/* Change connection interval at runtime. min_ms / max_ms in milliseconds.
 * If connected, requests update immediately; otherwise takes effect on next connection. */
void     ble_app_set_conn_interval(uint16_t min_ms, uint16_t max_ms);

/* CUS service / connection state — used to call ble_cus_data_send() directly */
extern ble_cus_t m_cus;
extern uint16_t  m_conn_handle;
extern uint16_t  m_ble_max_data_len;

/* System helpers */
void log_init(void);
void timer_init(void);
void power_management_init(void);
void idle_state_handle(void);

/* ══════════════════════════════════════════════════════════════
 *  SAADC — ECG analog front-end (module-internal, no extern needed)
 *  250 Hz autonomous via TIMER3 → PPI → SAADC
 *  Fully owned by ecg/ecg.c, accessed via ecg_init() / ecg_process()
 * ════════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════════
 *  TIMER2 — free-running µs counter (owned by drivers/ppg/max.c)
 *  Accessed via timer2_init() / timer2_now() declared in max.h
 * ════════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════════
 *  PWM — reserved (not yet implemented)
 * ════════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════════
 *  Unified sensor data struct
 *  One global instance g_sensor holds all vital-sign results.
 *  Written by each driver; read by main loop and BLE packet builder.
 * ════════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t  hr_ppg;      /* heart rate (BPM) from MAX30102 PPG   */
    uint8_t  hr_ecg;      /* heart rate (BPM) from ECG R-peak     */
    uint8_t  spo2;        /* SpO2 (%) from MAX30102               */
    float    temp;        /* temperature (°C) from TMP117         */
    bool     temp_valid;  /* TMP117 has settled reading           */
    uint32_t steps;       /* pedometer step count (LCD only)      */
    float    cadence;     /* steps per minute (LCD only)          */
} sensor_data_t;

extern sensor_data_t g_sensor;

#endif /* MAIN_H */
