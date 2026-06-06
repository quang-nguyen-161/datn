#include "max30102.h"
#include "filter.h"
#include "main.h"           /* m_twi, m_xfer_done, m_xfer_error, twi_wait() */
#include "nrf_drv_twi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "app_error.h"
#include <math.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════
 *  TIMER2 — free-running 1 MHz counter
 * ══════════════════════════════════════════════════════════ */

static const nrf_drv_timer_t TIMER2 = NRF_DRV_TIMER_INSTANCE(2);

static void timer2_handler(nrf_timer_event_t e, void *ctx) { (void)e; (void)ctx; }

void timer2_init(void)
{
    nrf_drv_timer_config_t cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    cfg.frequency = NRF_TIMER_FREQ_1MHz;
    cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    ret_code_t err = nrf_drv_timer_init(&TIMER2, &cfg, timer2_handler);
    APP_ERROR_CHECK(err);
    nrf_drv_timer_enable(&TIMER2);
}

uint32_t timer2_now(void)
{
    nrf_drv_timer_capture(&TIMER2, NRF_TIMER_CC_CHANNEL0);
    return nrf_drv_timer_capture_get(&TIMER2, NRF_TIMER_CC_CHANNEL0);
}

/* ══════════════════════════════════════════════════════════
 *  I2C helpers  (return false on NACK / bus error)
 * ══════════════════════════════════════════════════════════ */

static bool ppg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_tx(&m_twi, I2C_ADDR_MAX30102, buf, 2, false) != NRF_SUCCESS)
        return false;
    return twi_wait();
}

static bool ppg_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_tx(&m_twi, I2C_ADDR_MAX30102, &reg, 1, true) != NRF_SUCCESS)
        return false;
    if (!twi_wait()) return false;

    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_rx(&m_twi, I2C_ADDR_MAX30102, data, (uint8_t)len) != NRF_SUCCESS)
        return false;
    return twi_wait();
}

/* ══════════════════════════════════════════════════════════
 *  PPG DSP filter state (IR + RED channels, Fs = 100 Hz)
 *  Pipeline: HP 0.5 Hz → LP 5 Hz → ALE-NLMS → Savitzky-Golay
 * ══════════════════════════════════════════════════════════ */

static iir1_df2t_t s_ir_hp;    /* HP 0.5 Hz — baseline wander removal  */
static iir1_df2t_t s_ir_lp;    /* LP 5.0 Hz — high-freq noise removal  */
static nlms_t      s_ir_nlms;  /* NLMS adaptive filter (32 taps)        */
static delay_t     s_ir_dly;   /* ALE decorrelation delay               */
static sg_filter_t s_ir_sg;    /* Savitzky-Golay smoothing (window=11)  */

static iir1_df2t_t s_red_hp;
static iir1_df2t_t s_red_lp;
static nlms_t      s_red_nlms;
static delay_t     s_red_dly;
static sg_filter_t s_red_sg;

static void ppg_filters_init(void)
{
    butter1_hp_coeffs(100.0f, 0.5f, &s_ir_hp);
    butter1_lp_coeffs(100.0f, 5.0f, &s_ir_lp);
    nlms_init(&s_ir_nlms, 0.005f, 1e-6f);
    delay_init(&s_ir_dly);
    sg_init(&s_ir_sg);

    butter1_hp_coeffs(100.0f, 0.5f, &s_red_hp);
    butter1_lp_coeffs(100.0f, 5.0f, &s_red_lp);
    nlms_init(&s_red_nlms, 0.005f, 1e-6f);
    delay_init(&s_red_dly);
    sg_init(&s_red_sg);
}

/* ══════════════════════════════════════════════════════════
 *  Sensor control
 * ══════════════════════════════════════════════════════════ */

bool max30102_init(void)
{
    /* Software reset — if NACK, sensor is absent */
    if (!ppg_write(MODE_CONFIG_REGISTER, 0x40)) return false;
    nrf_delay_ms(10);

    /* Verify device is alive after reset */
    uint8_t mode = 0;
    if (!ppg_read(MODE_CONFIG_REGISTER, &mode, 1)) return false;

    /* SpO2 mode, 100 Hz, 411 µs pulse, 16384 ADC range */
    ppg_write(FIFO_CONFIG_REGISTER,   0x00);
    ppg_write(MODE_CONFIG_REGISTER,   0x03);
    ppg_write(SPO2_CONFIG_REGISTER,   (0x01 << 5) | (0x03 << 2) | 0x03);
    ppg_write(LED_CURRENT_REGISTER_1, LED_CURRENT_LOW);
    ppg_write(LED_CURRENT_REGISTER_2, LED_CURRENT_LOW);

    /* Clear FIFO */
    ppg_write(FIFO_WRITE_POINTER_REGISTER, 0x00);
    ppg_write(FIFO_READ_POINTER_REGISTER,  0x00);
    ppg_write(OVER_FLOW_COUNTER_REGISTER,  0x00);

    ppg_filters_init();

    NRF_LOG_INFO("[MAX30102] Init OK");
    return true;
}

void max30102_shutdown(void)
{
    ppg_write(MODE_CONFIG_REGISTER,   0x80 | 0x03);
    ppg_write(LED_CURRENT_REGISTER_1, LED_CURRENT_OFF);
    ppg_write(LED_CURRENT_REGISTER_2, LED_CURRENT_OFF);
}

void max30102_wakeup(void)
{
    ppg_write(MODE_CONFIG_REGISTER, 0x03);
    nrf_delay_ms(10);
    ppg_write(FIFO_WRITE_POINTER_REGISTER, 0x00);
    ppg_write(FIFO_READ_POINTER_REGISTER,  0x00);
    ppg_write(OVER_FLOW_COUNTER_REGISTER,  0x00);
}

/* ══════════════════════════════════════════════════════════
 *  FIFO read
 * ══════════════════════════════════════════════════════════ */

#define MAX_FIFO_DEPTH 32

int max30102_read_samples(uint32_t *ir_buf, uint32_t *red_buf, int max_samples)
{
    uint8_t wptr = 0, rptr = 0;
    if (!ppg_read(FIFO_WRITE_POINTER_REGISTER, &wptr, 1) ||
        !ppg_read(FIFO_READ_POINTER_REGISTER,  &rptr, 1))
        return 0;

    int avail = ((MAX_FIFO_DEPTH + (int)wptr) - (int)rptr) % MAX_FIFO_DEPTH;
    if (avail <= 0) return 0;
    if (avail > max_samples)    avail = max_samples;
    if (avail > MAX_FIFO_DEPTH) avail = MAX_FIFO_DEPTH;

    uint8_t raw[MAX_FIFO_DEPTH * 6];
    if (!ppg_read(FIFO_DATA_REGISTER, raw, (uint16_t)(6 * avail))) return 0;

    for (int i = 0; i < avail; i++) {
        red_buf[i] = (((uint32_t)raw[6*i]   << 16) |
                      ((uint32_t)raw[6*i+1]  << 8)  |
                       (uint32_t)raw[6*i+2]) & 0x03FFFFUL;
        ir_buf[i]  = (((uint32_t)raw[6*i+3] << 16) |
                      ((uint32_t)raw[6*i+4]  << 8)  |
                       (uint32_t)raw[6*i+5]) & 0x03FFFFUL;
    }
    return avail;
}

/* ══════════════════════════════════════════════════════════
 *  DSP — HR and SpO2 from accumulated raw buffer
 *
 *  Pipeline applied inside max30102_compute():
 *    raw uint32 → HP 0.5 Hz → LP 5 Hz → ALE-NLMS → Savitzky-Golay
 *  DC for SpO2 R-ratio is computed from the raw buffer before filtering.
 * ══════════════════════════════════════════════════════════ */

#define SAMPLE_RATE_HZ  100.0f
#define EPSILON         0.001f
#define HR_MIN          30.0f
#define HR_MAX          220.0f
#define SPO2_MIN        70.0f
#define SPO2_MAX        100.0f
#define PPG_MAX_BUF     100

static int detect_peaks_f(float *buf, int n, int *peaks, int min_dist)
{
    int count = 0;
    for (int i = 1; i < n - 1; i++) {
        if (buf[i] > buf[i-1] && buf[i] > buf[i+1]) {
            if (count == 0 || (i - peaks[count-1]) > min_dist)
                peaks[count++] = i;
        }
    }
    return count;
}

static float calc_hr(int *peaks, int n_peaks, float fs)
{
    if (n_peaks < 2) return 0.0f;
    float rr_sum = 0.0f;
    for (int i = 1; i < n_peaks; i++)
        rr_sum += (float)(peaks[i] - peaks[i-1]) / fs;
    float rr_avg = rr_sum / (n_peaks - 1);
    return (rr_avg > EPSILON) ? (60.0f / rr_avg) : 0.0f;
}

/* SpO2 via peak-valley ratio on filtered signal, DC from raw */
static float calc_spo2_f(float *ir, float *red, int n,
                          int *peaks, int n_peaks,
                          float dc_ir, float dc_red)
{
    if (n_peaks < 2 || n < 4 || dc_ir < EPSILON || dc_red < EPSILON) return 0.0f;

    float ac_ir = 0.0f, ac_red = 0.0f;
    int   beats = 0;
    for (int b = 0; b < n_peaks - 1; b++) {
        int s = peaks[b], e = peaks[b+1];
        if (e >= n) e = n - 1;
        float min_ir = ir[s], min_red = red[s];
        for (int j = s + 1; j <= e; j++) {
            if (ir[j]  < min_ir)  min_ir  = ir[j];
            if (red[j] < min_red) min_red = red[j];
        }
        float ai = ir[peaks[b]]  - min_ir;
        float ar = red[peaks[b]] - min_red;
        if (ai > 0.5f && ar > 0.5f) { ac_ir += ai; ac_red += ar; beats++; }
    }
    if (beats == 0) return 0.0f;

    float r = (ac_red / beats / dc_red) / (ac_ir / beats / dc_ir);
    if (r > 0.02f && r < 1.84f)
        return (-45.06f * r + 30.35f) * r + 94.845f;
    return 0.0f;
}

static float s_hr_hist[3] = {0.0f, 0.0f, 0.0f};

bool max30102_compute(uint32_t *ir_buf, uint32_t *red_buf, int count,
                      float *hr_out, float *spo2_out)
{
    if (count < 20) return false;

    /* 1. DC from raw before HP filtering removes it (needed for SpO2 R-ratio) */
    uint64_t ir_sum = 0, red_sum = 0;
    for (int i = 0; i < count; i++) { ir_sum += ir_buf[i]; red_sum += red_buf[i]; }
    float dc_ir  = (float)ir_sum  / count;
    float dc_red = (float)red_sum / count;
    if (dc_ir < EPSILON || dc_red < EPSILON) return false;

    /* 2. Apply DSP pipeline sample-by-sample: HP → LP → ALE-NLMS → Savitzky-Golay */
    float ir_filt[PPG_MAX_BUF];
    float red_filt[PPG_MAX_BUF];
    for (int i = 0; i < count; i++) {
        float ir  = (float)ir_buf[i];
        float red = (float)red_buf[i];

        ir  = iir1_step(&s_ir_hp,  ir);
        ir  = iir1_step(&s_ir_lp,  ir);
        ir  = ale_process(&s_ir_nlms, &s_ir_dly, ir);
        ir  = sg_step(&s_ir_sg, ir);

        red = iir1_step(&s_red_hp, red);
        red = iir1_step(&s_red_lp, red);
        red = ale_process(&s_red_nlms, &s_red_dly, red);
        red = sg_step(&s_red_sg, red);

        ir_filt[i]  = ir;
        red_filt[i] = red;
    }

    /* 3. Peak detection on filtered IR — min distance = 0.5 s at 100 Hz */
    int peaks[PPG_MAX_BUF];
    int n_peaks = detect_peaks_f(ir_filt, count, peaks,
                                  (int)(SAMPLE_RATE_HZ * 0.5f));
    if (n_peaks < 2) return false;

    /* 4. HR from RR intervals */
    float hr = calc_hr(peaks, n_peaks, SAMPLE_RATE_HZ);

    /* 5. SpO2 from filtered peak-valley AC, normalised by raw DC */
    float spo2 = calc_spo2_f(ir_filt, red_filt, count, peaks, n_peaks,
                               dc_ir, dc_red);

    if (hr < HR_MIN || hr > HR_MAX)         return false;
    if (spo2 < SPO2_MIN || spo2 > SPO2_MAX) return false;

    /* 6. 3-sample HR history smoothing */
    s_hr_hist[0] = s_hr_hist[1];
    s_hr_hist[1] = s_hr_hist[2];
    s_hr_hist[2] = hr;
    float hr_sum = 0.0f; int hr_n = 0;
    for (int i = 0; i < 3; i++) {
        if (s_hr_hist[i] > 0.0f) { hr_sum += s_hr_hist[i]; hr_n++; }
    }
    *hr_out   = (hr_n > 0) ? (hr_sum / hr_n) : hr;
    *spo2_out = spo2;
    return true;
}
