#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"

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
