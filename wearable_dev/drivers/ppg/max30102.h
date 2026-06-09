#ifndef _MAX30102_H
#define _MAX30102_H

#include <stdint.h>
#include <stdbool.h>

/* ── I2C address ─────────────────────────────── */
#define I2C_ADDR_MAX30102             0x57

/* ── Register map ────────────────────────────── */
#define INTERRUPT_STATUS_1_REGISTER 0x00
#define INTERRUPT_STATUS_2_REGISTER 0x01
#define INTERRUPT_ENABLE_1_REGISTER 0x02
#define INTERRUPT_ENABLE_2_REGISTER 0x03
#define FIFO_WRITE_POINTER_REGISTER   0x04
#define OVER_FLOW_COUNTER_REGISTER    0x05
#define FIFO_READ_POINTER_REGISTER    0x06
#define FIFO_DATA_REGISTER            0x07
#define FIFO_CONFIG_REGISTER          0x08
#define MODE_CONFIG_REGISTER          0x09
#define SPO2_CONFIG_REGISTER          0x0A
#define LED_CURRENT_REGISTER_1        0x0C
#define LED_CURRENT_REGISTER_2        0x0D

/* ── LED current presets ─────────────────────── */
#define LED_CURRENT_OFF               0x00
#define LED_CURRENT_LOW               0x1F   /* ~6.4 mA  */
#define LED_CURRENT_HIGH              0x7F   /* ~25.4 mA */

/* ── Public API ──────────────────────────────── */

/* Initialize sensor. Returns false if device does not respond (absent). */
bool max30102_init(void);
void max30102_shutdown(void);
void max30102_wakeup(void);

/* Read available FIFO samples into caller-supplied buffers.
 * Returns number of samples read (0 on I2C error or empty FIFO). */
int  max30102_read_samples(uint32_t *ir_buf, uint32_t *red_buf, int max_samples);

bool max30102_read_1_sample(uint32_t *ir, uint32_t *red);

/* Compute HR and SpO2 from a filled sample buffer.
 * Returns true and fills *hr_out / *spo2_out when result is valid. */
bool max30102_compute(uint32_t *ir_buf, uint32_t *red_buf, int count,
                      float *hr_out, float *spo2_out);

/* Timer2 — free-running 1 MHz counter (µs resolution) */
void     timer2_init(void);
uint32_t timer2_now(void);

#endif /* _MAX30102_H */
