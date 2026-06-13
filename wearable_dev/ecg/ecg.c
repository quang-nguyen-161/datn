#include "ecg.h"
#include "../dsp/filter.h"
#include "main.h"          /* g_sensor */
#include "peripheral.h"    /* adc_init(): SAADC + PPI + TIMER3 sampling chain */
#include "cmd.h"           /* g_ecg_stream_enabled */

#include <string.h>        /* memcpy (RR-interval median buffer) */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* ================================================================
 *  Shared result struct (filled by ecg_process, consumed in main loop)
 *  The SAADC capture globals g_ecg_raw / g_ecg_ready and the whole
 *  SAADC + PPI + TIMER3 sampling chain live in peripheral.c.
 * ================================================================ */
ecg_result_t     g_ecg       = {0};

/* ================================================================
 *  ECG filter states (private to this module)
 *  Pipeline: 4th-order notch (2× Q=10 @ 50 Hz)
 *            → bandpass (4th-order Butter, 1–25 Hz)
 *            → ALE-NLMS (adaptive line enhancer)
 *            → Savitzky-Golay (window=11, poly=3)
 * ================================================================ */
static biquad_df2t_t  s_notch1, s_notch2;         /* 2× 2nd-order notch, Q=10, 50 Hz → 4th-order */
static biquad_df2t_t  s_hpf1, s_hpf2;             /* 2× 2nd-order HP at 1 Hz  */
static biquad_df2t_t  s_lpf1_ecg, s_lpf2_ecg;     /* 2× 2nd-order LP at 25 Hz */
static nlms_t         s_nlms;
static delay_t        s_ale_delay;
static sg_filter_t    s_sg;

/* ================================================================
 *  Public API
 * ================================================================ */
void ecg_init(void)
{
    /* 4th-order notch @ 50 Hz: two cascaded 2nd-order sections, Q=10 */
    notch2_coeffs(250.0f, 50.0f, 10.0f, &s_notch1);
    notch2_coeffs(250.0f, 50.0f, 10.0f, &s_notch2);

    /* Bandpass: 4th-order Butterworth 1–25 Hz (2× HP + 2× LP biquads) */
    butter2_hp_coeffs(250.0f, 1.0f,  &s_hpf1);
    butter2_hp_coeffs(250.0f, 1.0f,  &s_hpf2);
    butter2_lp_coeffs(250.0f, 25.0f, &s_lpf1_ecg);
    butter2_lp_coeffs(250.0f, 25.0f, &s_lpf2_ecg);

    nlms_init(&s_nlms, 0.01f, 1e-6f);
    delay_init(&s_ale_delay);
    sg_init(&s_sg);

    adc_init();    /* SAADC + PPI + TIMER3 — 250 Hz autonomous sampling */
    NRF_LOG_INFO("ECG: filters ready");
    NRF_LOG_FLUSH();
}

/* ================================================================
 *  R-peak detector v1 — simplified Pan-Tompkins adaptive threshold
 *  Adaptive threshold: decays toward 0 when no beat, spikes up on detection.
 *  Returns non-zero BPM on beat, 0 otherwise.
 * ================================================================ */
static uint8_t rpeak_detect(float x)
{
    static float    threshold = 200.0f;
    static float    peak_max  = 0.0f;
    static uint32_t last_beat = 0;  /* sample counter */
    static uint32_t sample_n  = 0;

    sample_n++;

    /* rising phase — track peak */
    if (x > threshold && x > peak_max) { peak_max = x; }

    /* falling edge crosses half-threshold — beat detected */
    if (peak_max > threshold && x < threshold * 0.5f)
    {
        uint32_t rr_samples = sample_n - last_beat;
        last_beat = sample_n;
        peak_max  = 0.0f;

        /* update threshold adaptively */
        threshold = threshold * 0.7f + (x + 50.0f) * 0.3f;

        /* physiologically valid: 30–220 BPM at 250 Hz */
        if (rr_samples > 68 && rr_samples < 500)
        {
            return (uint8_t)(15000u / rr_samples); /* 60s × 250Hz / rr */
        }
    }

    /* threshold decay when quiet */
    if (peak_max == 0.0f) { threshold *= 0.9999f; }
    if (threshold < 50.0f) { threshold = 50.0f; }

    return 0;
}

/* ================================================================
 *  R-peak detector v2 — derivative-square + adaptive envelope + median RR
 *  Scaled to 250 Hz. Runs on the same SG-filtered output as v1.
 *  Takes precedence over v1 when it fires (more noise-robust BPM).
 * ================================================================ */
#define RPEAK2_REFRACT   62          /* 250 ms refractory @ 250 Hz      */
#define RR2_BUF_SIZE      8
#define RR2_MIN_SAMPLES  75          /* 200 BPM max @ 250 Hz            */
#define RR2_MAX_SAMPLES  375         /* 40 BPM min  @ 250 Hz            */

static float    s_env_peak2  = 0.0f;
static uint32_t s_rr2_buf[RR2_BUF_SIZE] = {200,200,200,200,200,200,200,200};
static int      s_rr2_head   = 0;
static uint32_t s_last_peak2 = 0;
static uint32_t s_sample2    = 0;
static int      s_cooldown2  = 0;

static uint8_t rpeak_detect_v2(float val)
{
    s_sample2++;

    static float last_val = 0.0f;
    float diff = val - last_val;
    last_val   = val;
    float sq   = diff * diff;

    /* Adaptive envelope: instant attack, slow 0.992/sample decay */
    if (sq > s_env_peak2) s_env_peak2 = sq;
    else                  s_env_peak2 *= 0.992f;

    float thr = s_env_peak2 * 0.4f;
    if (thr < 1.0f) thr = 1.0f;    /* floor: ~1 (ADC-unit/sample)^2 */

    if (s_cooldown2 > 0) { s_cooldown2--; return 0; }

    if (sq > thr)
    {
        uint32_t rr = s_sample2 - s_last_peak2;
        s_last_peak2 = s_sample2;
        s_cooldown2  = RPEAK2_REFRACT;

        if (rr >= RR2_MIN_SAMPLES && rr <= RR2_MAX_SAMPLES)
        {
            s_rr2_buf[s_rr2_head] = rr;
            s_rr2_head = (s_rr2_head + 1) % RR2_BUF_SIZE;

            /* Median of 8 RR intervals — bubble sort on local copy */
            uint32_t tmp[RR2_BUF_SIZE];
            memcpy(tmp, s_rr2_buf, sizeof(tmp));
            for (int i = 0; i < RR2_BUF_SIZE - 1; i++)
                for (int j = 0; j < RR2_BUF_SIZE - 1 - i; j++)
                    if (tmp[j] > tmp[j+1]) {
                        uint32_t t = tmp[j]; tmp[j] = tmp[j+1]; tmp[j+1] = t;
                    }

            uint32_t med_rr = (tmp[3] + tmp[4]) / 2;
            if (med_rr > 0)
                return (uint8_t)(15000u / med_rr);  /* 60 × 250 / rr */
        }
    }
    return 0;
}

float ecg_process(int16_t raw)
{
    float x = (float)raw;

    /* 4th-order notch @ 50 Hz (2 cascaded biquads, Q=10) */
    x = biquad_step(&s_notch1,    x);
    x = biquad_step(&s_notch2,    x);

    /* Bandpass: 4th-order Butterworth 1–25 Hz */
    x = biquad_step(&s_hpf1,      x);
    x = biquad_step(&s_hpf2,      x);
    x = biquad_step(&s_lpf1_ecg,  x);
    x = biquad_step(&s_lpf2_ecg,  x);

    /* ALE-NLMS: adaptive line enhancer — attenuates narrowband interference */
    x = ale_process(&s_nlms, &s_ale_delay, x);

    /* Savitzky-Golay: smooth transients while preserving QRS peak shape */
    x = sg_step(&s_sg, x);

    /* R-peak v1: Pan-Tompkins adaptive threshold */
    uint8_t bpm = rpeak_detect(x);
    if (g_ecg_stream_enabled) {
        if (bpm > 0) { g_sensor.hr_ecg = bpm; g_sensor.hr_ecg_valid = true; }
    } else {
        g_sensor.hr_ecg_valid = false;
    }

    g_ecg.raw      = raw;
    g_ecg.filtered = x;
    g_ecg.new_data = true;

    return x;
}
