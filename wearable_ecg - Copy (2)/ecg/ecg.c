#include "ecg.h"
#include "filter.h"
#include "main.h"    /* g_sensor */

#include "app_error.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* ================================================================
 *  Shared globals (set in ISR, read in main loop)
 * ================================================================ */
volatile int16_t g_ecg_raw   = 0;
volatile bool    g_ecg_ready = false;
ecg_result_t     g_ecg       = {0};

/* ================================================================
 *  Hardware resources
 *  TIMER3: dedicated to SAADC PPI (TIMER0=SD, TIMER1=PWM, TIMER2=µs counter)
 * ================================================================ */
#define SAADC_SAMPLE_US  4000       /* 250 Hz */
#define SAMPLES_IN_BUFFER 1

static const nrf_drv_timer_t m_saadc_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t     m_buf[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_chan;

/* ================================================================
 *  ECG filter states (private to this module)
 * ================================================================ */
static dc_filter_t    s_dc_ecg;

static biquad_df2t_t  s_notch50 = {
    .b0 =  0.98351943f, .b1 = -1.05399213f, .b2 = 0.98351943f,
    .a1 = -1.05399213f, .a2 =  0.96703886f,
    .s1 = 0.0f, .s2 = 0.0f
};

static biquad_df2t_t  s_lpf1 = {
    .b0 = 0.06745527f, .b1 = 0.13491055f, .b2 = 0.06745527f,
    .a1 = -1.14298050f, .a2 = 0.41280160f,
    .s1 = 0.0f, .s2 = 0.0f
};

static biquad_df2t_t  s_lpf2 = {
    .b0 = 0.06745038f, .b1 = 0.13490076f, .b2 = 0.06745038f,
    .a1 = -1.14289760f, .a2 = 0.41269904f,
    .s1 = 0.0f, .s2 = 0.0f
};

/* ================================================================
 *  DC-removal (sliding-mean)
 * ================================================================ */
static float dc_remove(dc_filter_t *f, float x)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = x;
    f->sum += x;
    f->idx = (f->idx + 1) % DC_WINDOW;
    f->dc_comp = f->sum / DC_WINDOW;
    return x - f->dc_comp;
}

/* ================================================================
 *  SAADC callback — fires every 4 ms via PPI, minimal work here
 * ================================================================ */
static void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type != NRF_DRV_SAADC_EVT_DONE) { return; }

    g_ecg_raw   = p_event->data.done.p_buffer[0];
    g_ecg_ready = true;

    ret_code_t err = nrf_drv_saadc_buffer_convert(
        p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err);
}

/* ================================================================
 *  SAADC init — 12-bit, AIN0, GAIN1_6, internal 0.6 V ref
 *  AD8232 output 0.5–2.5 V fits inside 3.6 V full-scale range
 * ================================================================ */
static void saadc_init(void)
{
    nrf_saadc_channel_config_t ch = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    ch.gain      = NRF_SAADC_GAIN1_6;
    ch.reference = NRF_SAADC_REFERENCE_INTERNAL;
    ch.acq_time  = NRF_SAADC_ACQTIME_20US;
    ch.mode      = NRF_SAADC_MODE_SINGLE_ENDED;
    ch.burst     = NRF_SAADC_BURST_ENABLED;

    nrf_drv_saadc_config_t cfg = NRF_DRV_SAADC_DEFAULT_CONFIG;
    cfg.resolution = NRF_SAADC_RESOLUTION_12BIT;
    cfg.oversample = NRF_SAADC_OVERSAMPLE_4X;

    ret_code_t err;
    err = nrf_drv_saadc_init(&cfg, saadc_callback);      APP_ERROR_CHECK(err);
    err = nrf_drv_saadc_channel_init(0, &ch);             APP_ERROR_CHECK(err);
    err = nrf_drv_saadc_buffer_convert(m_buf[0], SAMPLES_IN_BUFFER); APP_ERROR_CHECK(err);
    err = nrf_drv_saadc_buffer_convert(m_buf[1], SAMPLES_IN_BUFFER); APP_ERROR_CHECK(err);
    nrf_drv_saadc_calibrate_offset();
}

/* ================================================================
 *  PPI init — TIMER3 compare → SAADC sample task at 250 Hz
 * ================================================================ */
static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
    (void)event_type; (void)p_context;
}

static void ppi_init(void)
{
    ret_code_t err;

    err = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err);

    nrf_drv_timer_config_t tcfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    tcfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err = nrf_drv_timer_init(&m_saadc_timer, &tcfg, timer_handler);
    APP_ERROR_CHECK(err);

    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_saadc_timer, SAADC_SAMPLE_US);
    nrf_drv_timer_extended_compare(&m_saadc_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_saadc_timer);

    uint32_t timer_evt  = nrf_drv_timer_compare_event_address_get(&m_saadc_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_task = nrf_drv_saadc_sample_task_get();

    err = nrf_drv_ppi_channel_alloc(&m_ppi_chan);     APP_ERROR_CHECK(err);
    err = nrf_drv_ppi_channel_assign(m_ppi_chan, timer_evt, saadc_task); APP_ERROR_CHECK(err);
    err = nrf_drv_ppi_channel_enable(m_ppi_chan);     APP_ERROR_CHECK(err);
}

/* ================================================================
 *  Public API
 * ================================================================ */
void ecg_init(void)
{
    saadc_init();
    ppi_init();
    NRF_LOG_INFO("ECG: 250 Hz PPI sampling started");
    NRF_LOG_FLUSH();
}

/* ================================================================
 *  Simplified Pan-Tompkins R-peak detector
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

float ecg_process(int16_t raw)
{
    float x = (float)raw;
    x = dc_remove(&s_dc_ecg, x);
    x = biquad_step(&s_notch50, x);
    x = biquad_step(&s_lpf1,    x);
    x = biquad_step(&s_lpf2,    x);

    uint8_t bpm = rpeak_detect(x);
    if (bpm > 0) { g_sensor.hr_ecg = bpm; }

    g_ecg.raw      = raw;
    g_ecg.filtered = x;
    g_ecg.new_data = true;

    return x;
}
