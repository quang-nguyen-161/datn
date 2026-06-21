#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_gpio.h"       /* NRF_GPIO_PIN_MAP for board pin map     */
#include "nrf_saadc.h"      /* NRF_SAADC_INPUT_AIN0 for ECG_SAADC_INPUT */
#include "peripheral.h"     /* TWI/SPI/ADC/TIMER instances + init API  */

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
#define MMA8452Q_INT1_PIN   NRF_GPIO_PIN_MAP(0, 31)   /* TODO: confirm on 52840 board */
/* SPI — SPIM3 is the only nRF52840 instance capable of >8 Mbps */
#define LCD_SPI_INSTANCE    3
#define LCD_SPI_FREQ        NRF_SPIM_FREQ_32M
#else
/* ── nRF52832 (pca10040) — all pins must be Port 0 (0–31) ── */
/* I2C (TWI1) — MAX30102 / MMA8452Q / TMP117, 400 kHz */
#define TWI_SCL_PIN         NRF_GPIO_PIN_MAP(0, 29)   /* P0.29 SCL */
#define TWI_SDA_PIN         NRF_GPIO_PIN_MAP(0, 28)   /* P0.28 SDA */
/* Display (SPI0 + GPIO) — GC9A01 240×240 LCD */
#define LCD_MOSI_PIN        13
#define LCD_SCK_PIN         12
#define LCD_CS_Pin          16
#define LCD_DC_Pin          15
#define LCD_RES_Pin         14
#define LCD_BLK_Pin         17
/* Accelerometer INT1 (MMA8452Q) */
#define MMA8452Q_INT1_PIN   NRF_GPIO_PIN_MAP(0, 30)   /* P0.27 */
/* SPI — nRF52832 has SPIM0/1/2 only, max 8 MHz */
#define LCD_SPI_INSTANCE    0
#define LCD_SPI_FREQ        NRF_SPIM_FREQ_4M
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
