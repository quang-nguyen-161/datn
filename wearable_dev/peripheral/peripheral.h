#ifndef PERIPHERAL_H
#define PERIPHERAL_H

/* ══════════════════════════════════════════════════════════════
 *  peripheral.h — centralized low-level hardware module
 *
 *  Owns every on-chip peripheral the application uses: the shared
 *  TWI1 I2C bus, the SPI0 display bus, the SAADC + PPI + TIMER3
 *  ECG sampling chain, the TIMER2 µs counter, and the (reserved)
 *  PWM. Each peripheral exposes an init function, its IRQ callback
 *  lives here, and the shared driver-instance handles are defined
 *  here (declared extern below).
 *
 *  Board pin assignments live in main.h (board pin map).
 * ════════════════════════════════════════════════════════════ */

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"

/* ══════════════════════════════════════════════════════════════
 *  TWI (I2C) — shared bus
 *  Serves: MAX30102 (0x57)  MMA8452Q (0x1C)  TMP117 (0x48)
 *  Instance: TWI1   SCL/SDA from main.h   400 kHz
 * ════════════════════════════════════════════════════════════ */
extern nrf_drv_twi_t  m_twi;
extern volatile bool  m_xfer_done;   /* set true by twi_handler on any completion */
extern volatile bool  m_xfer_error;  /* set true by twi_handler on NACK/error     */

void twi_init(void);
bool twi_wait(void);  /* spin-wait with 200k-cycle timeout; resets bus on timeout */

/* ══════════════════════════════════════════════════════════════
 *  SPI — display bus
 *  Serves: GC9A01 240×240 LCD   Instance: SPI0   4 MHz
 *  spi_init() brings up the bus only; GC9A01.c configures LCD GPIO.
 * ════════════════════════════════════════════════════════════ */
extern nrf_drv_spi_t  m_lcd_spi;

bool spi_init(void);  /* returns false if nrf_drv_spi_init fails */

/* ══════════════════════════════════════════════════════════════
 *  SAADC + PPI + TIMER3 — ECG analog front-end
 *  Autonomous 250 Hz sampling: TIMER3 compare → PPI → SAADC sample.
 *  saadc_callback() stashes the raw sample for the main loop.
 * ════════════════════════════════════════════════════════════ */
extern volatile int16_t g_ecg_raw;    /* latest raw SAADC sample  */
extern volatile bool    g_ecg_ready;  /* set by SAADC ISR, cleared by main loop */

void adc_init(void);                  /* SAADC + PPI + TIMER3 bring-up */
void adc_set_sample_us(uint32_t us);  /* live TIMER3 compare reconfigure (4000 = 250 Hz) */

/* ══════════════════════════════════════════════════════════════
 *  TIMER2 — free-running 1 MHz µs counter
 * ════════════════════════════════════════════════════════════ */
void     timer2_init(void);
uint32_t timer2_now(void);

/* ══════════════════════════════════════════════════════════════
 *  PWM — LCD backlight dimming on LCD_BLK_Pin (PWM0)
 * ════════════════════════════════════════════════════════════ */
void pwm_init(void);                       /* init PWM0, sets backlight to 100% */

/* Set LCD backlight brightness. `percent` is clamped to 0–100
 * (0 = off, 100 = full). Safe to call any time after pwm_init();
 * a no-op if PWM init failed. */
void lcd_set_brightness(uint8_t percent);

#endif /* PERIPHERAL_H */
