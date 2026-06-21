#ifndef _MAX30102_H
#define _MAX30102_H

#include <stdint.h>
#include <stdbool.h>
#include "filter.h"   /* dc_filter_t (used by spo2_count) */

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

/* ── Register bit positions (used by the config setters) ── */
#define FIFO_SMP_AVE_SHIFT      5   /* FIFO_CONFIG_REGISTER  */
#define FIFO_ROLL_OVER_SHIFT    4
#define FIFO_A_FULL_SHIFT       0
#define SPO2_ADC_RGE_SHIFT      5   /* SPO2_CONFIG_REGISTER  */
#define SPO2_SR_SHIFT           2
#define SPO2_LED_PW_SHIFT       0
#define INTERRUPT_A_FULL_BIT    7   /* INTERRUPT_ENABLE_1    */

/* ── LED current presets ─────────────────────── */
#define LED_CURRENT_OFF               0x00
#define LED_CURRENT_LOW               0x1F   /* ~6.4 mA  */
#define LED_CURRENT_HIGH              0x7F   /* ~25.4 mA */

/* ── Config enums (mirror legacy driver) ─────── */
typedef enum {
    max30102_heart_rate = 0x02,
    max30102_spo2       = 0x03,
    max30102_multi_led  = 0x07
} max30102_mode_t;

typedef enum {
    max30102_smp_ave_1,
    max30102_smp_ave_2,
    max30102_smp_ave_4,
    max30102_smp_ave_8,
    max30102_smp_ave_16,
    max30102_smp_ave_32
} max30102_smp_ave_t;

typedef enum {
    max30102_sr_50,
    max30102_sr_100,
    max30102_sr_200,
    max30102_sr_400,
    max30102_sr_800,
    max30102_sr_1000,
    max30102_sr_1600,
    max30102_sr_3200
} max30102_sr_t;

typedef enum {
    max30102_pw_15_bit,
    max30102_pw_16_bit,
    max30102_pw_17_bit,
    max30102_pw_18_bit
} max30102_led_pw_t;

typedef enum {
    max30102_adc_2048,
    max30102_adc_4096,
    max30102_adc_8192,
    max30102_adc_16384
} max30102_adc_t;

/* ── Float ring buffer (HR/SpO2 algorithms) ──── */
#define RB_SIZE 100
typedef struct {
    float   buff[RB_SIZE];
    uint8_t rb_head;
} rb_t;

/* ── Public API ──────────────────────────────── */

/* Initialize sensor. Returns false if device does not respond (absent). */
bool max30102_init(void);
void max30102_shutdown(void);
void max30102_wakeup(void);
/* Reset all filter and algorithm state, recomputing coefficients for fs Hz.
 * Call after changing the sample rate or LED current so the filter cutoffs
 * and update period stay correct at the new Fs. */
void max30102_reset_filters(float fs);

/* Read available FIFO samples into caller-supplied buffers.
 * Returns number of samples read (0 on I2C error or empty FIFO). */
int  max30102_read_samples(uint32_t *ir_buf, uint32_t *red_buf, int max_samples);

bool max30102_read_1_sample(uint32_t *ir, uint32_t *red);

/* Read one sample (red, ir) from FIFO (legacy-style single-sample read). */
void max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);

/* Streaming HR/SpO2: push one raw sample, process incrementally.
 * Returns true and fills *hr_out / *spo2_out when a fresh valid result is ready
 * (roughly once per PPG_UPDATE_PERIOD samples). */
bool max30102_process(uint32_t ir, uint32_t red, float *hr_out, float *spo2_out);

/* True when the HR-source channel (IR or RED, per g_ppg_hr_source) is above
 * the saturation bound — updated every call to max30102_process(). */
bool max30102_hr_saturated(void);

/* True while a finger is on the sensor but the adaptive LED current loop
 * hasn't yet converged on PPG_TARGET_ADC — HR/SpO2 are 0 during this window. */
bool max30102_is_calibrating(void);

/* ── Register-config setters (translated from legacy driver) ── */
void max30102_reset(void);
void max30102_turnon(void);
void max30102_turnoff(void);
void max30102_clear_fifo(void);
void max30102_set_led_pulse_width(max30102_led_pw_t pw);
void max30102_set_fifo_config(max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full);
void max30102_set_adc_resolution(max30102_adc_t adc);
void max30102_set_sampling_rate(max30102_sr_t sr);
void max30102_set_led_current_1(float ma);
void max30102_set_led_current_2(float ma);
void max30102_set_mode(max30102_mode_t mode);
void max30102_set_a_full(uint8_t enable);

/* ── Float ring-buffer HR/SpO2 algorithm (used by max30102_process) ── */
void     rb_push(rb_t *rb, float value);
bool     find_peak(rb_t *rb);
bool     find_peak_thresh(rb_t *rb, float threshold);
void     peak_time_push(rb_t *signal_rb, rb_t *dt_rb);
uint32_t hr_count(rb_t *dt_rb, rb_t *w_rb, uint32_t peak_count);
uint32_t spo2_count(rb_t *ir_acdc_rb, rb_t *red_acdc_rb,
                    rb_t *ir_raw_rb, rb_t *red_raw_rb);

/* ── Raw-uint32 ring-buffer HR/SpO2 algorithm (legacy variant, fallback) ── */
void     ring_buffer_push(uint32_t *buff, uint32_t value, uint32_t size, uint32_t *head);
bool     find_peak1(uint32_t *buff, uint32_t size, uint32_t head);
void     peak_time_push1(uint32_t *buff, uint32_t *delta_t_buff, uint32_t size,
                         uint32_t head, uint32_t *dt_head, uint32_t delta_size);
uint32_t hr_count1(uint32_t *delta_t_buff, uint32_t delta_size,
                   uint32_t peak, uint32_t dt_head);
uint32_t spo2_count1(uint32_t *ir_buff, uint32_t *red_buff,
                     uint32_t size, uint32_t head);

/* Timer2 (µs counter) now lives in peripheral.h — timer2_init()/timer2_now(). */

/* ══════════════════════════════════════════════════════════
 *  Adaptive Derivative-Envelope peak detector
 *  Ported from Pan-Tompkins-inspired ECG approach:
 *    derivative → square → fast-attack/slow-decay envelope → 40% threshold
 *  Works well for PPG because it self-calibrates to signal amplitude.
 * ══════════════════════════════════════════════════════════ */

#define ADENV_RR_SIZE       8        /* RR history depth for median HR           */
#define ADENV_REFRACT       35       /* 350 ms refractory @ 100 Hz               */
#define ADENV_ENV_DECAY     0.992f   /* slow envelope decay between beats        */
#define ADENV_THRESH_RATIO  0.4f     /* fire when sq > 40 % of envelope peak     */
#define ADENV_RR_SEED       750000U  /* nominal 80 BPM @ 1 MHz timer (µs)       */
#define ADENV_RR_MIN        300000U  /* 200 BPM upper bound                      */
#define ADENV_RR_MAX        2000000U /* 30  BPM lower bound                      */

typedef struct {
    float    last_val;                  /* previous filtered sample               */
    float    env_peak;                  /* running envelope of squared derivative */
    uint32_t cooldown;                  /* refractory countdown (samples)         */
    uint32_t rr_buf[ADENV_RR_SIZE];     /* RR intervals (µs), seeded at init      */
    uint8_t  rr_head;                   /* next write index (circular)            */
    uint32_t last_peak_time;            /* µs timestamp of last accepted peak     */
    uint32_t peak_count;                /* total accepted peaks since init        */
} adenv_t;

/* Initialise state; seeds rr_buf with ADENV_RR_SEED for stable startup BPM. */
void  adenv_init(adenv_t *s);

/* Push one filtered sample; returns true when a peak is accepted this call.
 * Caller must gate on finger-present before calling (no signal → no peaks). */
bool  adenv_push(adenv_t *s, float sample);

/* Compute median BPM from the RR history in *s.
 * Returns 0.0 if fewer than 2 peaks have been accepted. */
float adenv_hr(const adenv_t *s);

/* Compute SpO2 (%) from AC/DC ratios of the filtered ring buffers.
 * Equivalent to spo2_count() but returns float directly. */
float adenv_spo2(dc_filter_t *dc_ir, dc_filter_t *dc_red,
                 rb_t *ir_rb, rb_t *red_rb);

/* Streaming HR/SpO2 using the adenv pipeline.
 * Drop-in replacement for max30102_process(); uses its own independent state.
 * Returns true and fills *hr_out/*spo2_out once per ~PPG_UPDATE_PERIOD samples. */
bool  max30102_process2(uint32_t ir_raw, uint32_t red_raw,
                        float *hr_out, float *spo2_out);

#endif /* _MAX30102_H */
