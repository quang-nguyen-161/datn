#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_gpio.h"       /* NRF_GPIO_PIN_MAP for board pin map     */
#include "nrf_saadc.h"      /* NRF_SAADC_INPUT_AIN0 for ECG_SAADC_INPUT */
#include "peripheral.h"     /* TWI/SPI/ADC/TIMER instances + init API  */

/* ── Single-sensor test mode ──
 * Define exactly one to isolate that sensor for bring-up/debug
 * (BLE/advertising disabled); leave all undefined for normal operation. */
//#define TEST_SENSOR_MAX
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
 *  Board pin map — all peripheral pin assignments
 *  nRF52840 (pca10056) has Port 1; nRF52832 (pca10040) is Port 0 only.
 *  Peripheral instances/init live in peripheral.c (TWI1, SPI0, SAADC,
 *  TIMER2/3). These macros are consumed by peripheral.c, GC9A01.c,
 *  and mma845.c.
 * ════════════════════════════════════════════════════════════ */
#if defined(NRF52840_XXAA)
/* ── nRF52840 (pca10056) custom board ── */
/* I2C (TWI1) — MAX30102 / MMA8452Q / TMP117, 400 kHz */
#define TWI_SCL_PIN         NRF_GPIO_PIN_MAP(0, 28)   /* TODO: confirm on 52840 board */
#define TWI_SDA_PIN         NRF_GPIO_PIN_MAP(0, 29)   /* TODO: confirm on 52840 board */
/* Display (SPI0 + GPIO) — GC9A01 240×240 LCD */
#define LCD_MOSI_PIN        NRF_GPIO_PIN_MAP(0, 7)    /* P0.07 SDA_LCD */
#define LCD_SCK_PIN         NRF_GPIO_PIN_MAP(0, 5)    /* P0.05 SCL_LCD */
#define LCD_CS_Pin          NRF_GPIO_PIN_MAP(1, 9)    /* P1.09 CS  */
#define LCD_DC_Pin          NRF_GPIO_PIN_MAP(1, 8)    /* P1.08 DC  */
#define LCD_RES_Pin         NRF_GPIO_PIN_MAP(0, 8)    /* P0.08 RES */
#define LCD_BLK_Pin         NRF_GPIO_PIN_MAP(0, 11)   /* P0.11 BLK */
/* Accelerometer INT1 (MMA8452Q) */
#define MMA8452Q_INT1_PIN   NRF_GPIO_PIN_MAP(0, 27)   /* TODO: confirm on 52840 board */
#else
/* ── nRF52832 (pca10040) — all pins must be Port 0 (0–31) ── */
/* I2C (TWI1) — MAX30102 / MMA8452Q / TMP117, 400 kHz */
#define TWI_SCL_PIN         NRF_GPIO_PIN_MAP(0, 29)   /* P0.29 SCL */
#define TWI_SDA_PIN         NRF_GPIO_PIN_MAP(0, 28)   /* P0.28 SDA */
/* Display (SPI0 + GPIO) — GC9A01 240×240 LCD */
#define LCD_MOSI_PIN        NRF_GPIO_PIN_MAP(0, 30)   /* P0.30 SDA_LCD */
#define LCD_SCK_PIN         NRF_GPIO_PIN_MAP(0, 31)   /* P0.31 SCL_LCD */
#define LCD_CS_Pin          NRF_GPIO_PIN_MAP(0, 26)   /* P0.26 CS  */
#define LCD_DC_Pin          NRF_GPIO_PIN_MAP(0, 27)   /* P0.27 DC  */
#define LCD_RES_Pin         NRF_GPIO_PIN_MAP(0, 3)    /* P0.03 RES */
#define LCD_BLK_Pin         NRF_GPIO_PIN_MAP(0, 25)   /* P0.25 BLK */
/* Accelerometer INT1 (MMA8452Q) */
#define MMA8452Q_INT1_PIN   NRF_GPIO_PIN_MAP(0, 27)   /* P0.27 */
#endif

#define LCD_MISO_PIN        NRF_SPI_PIN_NOT_CONNECTED

/* ECG analog front-end — SAADC AIN0 (P0.02) on both targets */
#define ECG_SAADC_INPUT     NRF_SAADC_INPUT_AIN0

bool lcd_spi_init(void);        /* defined in drivers/display/GC9A01.c; returns false if LCD absent */

/* System helpers */
void log_init(void);
void timer_init(void);
void power_management_init(void);
void idle_state_handle(void);

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
    bool     hr_ppg_valid;/* MAX30102 has produced a PPG HR        */
    bool     spo2_valid;  /* MAX30102 has produced an SpO2 reading */
    bool     hr_ecg_valid;/* ECG R-peak has produced a HR          */
    uint32_t steps;       /* pedometer step count (LCD only)      */
    float    cadence;     /* steps per minute (LCD only)          */
} sensor_data_t;

extern sensor_data_t g_sensor;

#endif /* MAIN_H */
