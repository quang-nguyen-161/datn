#include "max30102.h"
#include "filter.h"
#include "main.h"           /* m_twi, m_xfer_done, m_xfer_error, twi_wait() */
#include "nrf_drv_twi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
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

/* ── Tunable PPG filter/algorithm parameters (~1–2 Hz pulse @ 100 Hz) ── */
#define PPG_NLMS_TAPS     16
#define PPG_ALE_DELAY     6          /* 60 ms — shorter than dicrotic (~300 ms) */
#define PPG_SG_WINDOW     15
#define PPG_NLMS_MU       0.15f      /* was 0.005 — correct adaptation rate for PPG levels */
#define PPG_NLMS_EPS      0.01f      /* was 1e-6 — prevents div/0 artifact at startup */
#define PPG_UPDATE_PERIOD 100      /* emit HR/SpO2 once per ~1 s @ 100 Hz */
#define PPG_FINGER_THRESHOLD  30000U /* raw ADC count; ambient (no finger) ≈ 500 */
#define PPG_FINGER_DEBOUNCE   5      /* consecutive sub-threshold samples → off   */

static iir1_df2t_t s_ir_hp;    /* HP 0.5 Hz — baseline wander removal  */
static iir1_df2t_t s_ir_lp;    /* LP 5.0 Hz — high-freq noise removal  */
static nlms_t      s_ir_nlms;  /* NLMS adaptive filter                  */
static delay_t     s_ir_dly;   /* ALE decorrelation delay               */
static sg_filter_t s_ir_sg;    /* Savitzky-Golay smoothing              */

static iir1_df2t_t s_red_hp;
static iir1_df2t_t s_red_lp;
static nlms_t      s_red_nlms;
static delay_t     s_red_dly;
static sg_filter_t s_red_sg;

/* ── Streaming HR/SpO2 ring-buffer state (used by max30102_process) ── */
static rb_t        s_ir_rb;    /* filtered IR AC signal       */
static rb_t        s_red_rb;   /* filtered RED AC signal       */
static rb_t        s_dt_rb;    /* peak-to-peak intervals (µs)  */
static dc_filter_t s_dc_ir;    /* raw IR DC (sliding mean)     */
static dc_filter_t s_dc_red;   /* raw RED DC (sliding mean)    */
static uint32_t    s_peak_count;
static uint32_t    s_last_peak_time;
static uint32_t    s_sample_ctr;
static uint32_t    s_dc_warmup;    /* samples remaining until DC window is full */
static float       s_hr_hist[3];
static float       s_spo2_hist[3];
static uint8_t     s_no_finger_cnt;
static bool        s_finger_on;

static void ppg_filters_init(void)
{
    butter1_hp_coeffs(100.0f, 0.5f, &s_ir_hp);
    butter1_lp_coeffs(100.0f, 5.0f, &s_ir_lp);
    nlms_init_n(&s_ir_nlms, PPG_NLMS_TAPS, PPG_NLMS_MU, PPG_NLMS_EPS);
    delay_init_d(&s_ir_dly, PPG_ALE_DELAY);
    sg_init_n(&s_ir_sg, PPG_SG_WINDOW);

    butter1_hp_coeffs(100.0f, 0.5f, &s_red_hp);
    butter1_lp_coeffs(100.0f, 5.0f, &s_red_lp);
    nlms_init_n(&s_red_nlms, PPG_NLMS_TAPS, PPG_NLMS_MU, PPG_NLMS_EPS);
    delay_init_d(&s_red_dly, PPG_ALE_DELAY);
    sg_init_n(&s_red_sg, PPG_SG_WINDOW);

    /* reset streaming state */
    memset(&s_ir_rb,  0, sizeof(s_ir_rb));
    memset(&s_red_rb, 0, sizeof(s_red_rb));
    memset(&s_dt_rb,  0, sizeof(s_dt_rb));
    memset(&s_dc_ir,  0, sizeof(s_dc_ir));
    memset(&s_dc_red, 0, sizeof(s_dc_red));
    s_peak_count      = 0;
    s_last_peak_time  = 0;
    s_sample_ctr      = 0;
    s_dc_warmup       = DC_WINDOW;
    s_hr_hist[0]      = 0.0f;
    s_hr_hist[1]      = 0.0f;
    s_hr_hist[2]      = 0.0f;
    s_spo2_hist[0]    = 0.0f;
    s_spo2_hist[1]    = 0.0f;
    s_spo2_hist[2]    = 0.0f;
    s_no_finger_cnt   = 0;
    s_finger_on       = false;
}

/* ══════════════════════════════════════════════════════════
 *  Sensor control
 * ══════════════════════════════════════════════════════════ */

bool max30102_init(void)
{
    uint8_t reg;
    uint8_t dummy;

    NRF_LOG_INFO("[MAX30102] Reset");
    NRF_LOG_FLUSH();

    /* Match old driver exactly */
    ppg_write(MODE_CONFIG_REGISTER, 0x00);
    ppg_write(MODE_CONFIG_REGISTER, 0x40);

    nrf_delay_ms(1000);

    /* Old driver cleared interrupt status here */
    ppg_read(INTERRUPT_STATUS_1_REGISTER, &dummy, 1);

    /* Interrupt configuration */
    ppg_write(INTERRUPT_ENABLE_1_REGISTER, 0xC0);
    ppg_write(INTERRUPT_ENABLE_2_REGISTER, 0x00);

    /* FIFO pointers */
    ppg_write(FIFO_READ_POINTER_REGISTER, 0x00);
    ppg_write(FIFO_WRITE_POINTER_REGISTER, 0x00);
    ppg_write(OVER_FLOW_COUNTER_REGISTER, 0x00);

    /* Same FIFO config as old driver */
    ppg_write(FIFO_CONFIG_REGISTER, 0x0F);

    /* SpO2 mode */
    ppg_write(MODE_CONFIG_REGISTER, 0x03);

    /* Exact value from previous debug version */
    ppg_write(SPO2_CONFIG_REGISTER, 0x27);

    /* Increase LED current for debugging */
    ppg_write(LED_CURRENT_REGISTER_1, 0x1F);
    ppg_write(LED_CURRENT_REGISTER_2, 0x1F);

    /* Read back everything */
    ppg_read(MODE_CONFIG_REGISTER, &reg, 1);
    NRF_LOG_INFO("MODE=0x%02X", reg);

    ppg_read(SPO2_CONFIG_REGISTER, &reg, 1);
    NRF_LOG_INFO("SPO2=0x%02X", reg);

    ppg_read(FIFO_CONFIG_REGISTER, &reg, 1);
    NRF_LOG_INFO("FIFO=0x%02X", reg);

    ppg_read(LED_CURRENT_REGISTER_1, &reg, 1);
    NRF_LOG_INFO("LED1=0x%02X", reg);

    ppg_read(LED_CURRENT_REGISTER_2, &reg, 1);
    NRF_LOG_INFO("LED2=0x%02X", reg);

    NRF_LOG_FLUSH();

    nrf_delay_ms(1000);

    uint8_t wptr, rptr;

    ppg_read(FIFO_WRITE_POINTER_REGISTER, &wptr, 1);
    ppg_read(FIFO_READ_POINTER_REGISTER, &rptr, 1);

    NRF_LOG_INFO("FIFO_WR=%u", wptr);
    NRF_LOG_INFO("FIFO_RD=%u", rptr);
    NRF_LOG_INFO("FIFO_CNT=%d", ((32 + wptr) - rptr) % 32);

    /* Dump first FIFO sample */
    for(int i=0;i<10;i++)
{
    uint8_t fifo[6];

    ppg_read(FIFO_DATA_REGISTER, fifo, 6);

    NRF_LOG_INFO("%02X %02X %02X %02X %02X %02X",
                 fifo[0], fifo[1], fifo[2],
                 fifo[3], fifo[4], fifo[5]);

    nrf_delay_ms(100);
}


    NRF_LOG_FLUSH();

    ppg_filters_init();

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
 *  Register-config setters (translated from legacy driver:
 *  max30102_write/read → ppg_write/ppg_read, MAX30102_* → *_REGISTER).
 *  Standalone helpers; max30102_init() does not use them.
 * ══════════════════════════════════════════════════════════ */

void max30102_reset(void)
{
    ppg_write(MODE_CONFIG_REGISTER, 0x40);
}

void max30102_turnon(void)
{
    uint8_t config;
    ppg_read(MODE_CONFIG_REGISTER, &config, 1);
    config &= ~0x80;
    ppg_write(MODE_CONFIG_REGISTER, config);
}

void max30102_turnoff(void)
{
    uint8_t config;
    ppg_read(MODE_CONFIG_REGISTER, &config, 1);
    config |= 0x80;
    ppg_write(MODE_CONFIG_REGISTER, config);
}

void max30102_clear_fifo(void)
{
    ppg_write(FIFO_WRITE_POINTER_REGISTER, 0x00);
    ppg_write(FIFO_READ_POINTER_REGISTER,  0x00);
    ppg_write(OVER_FLOW_COUNTER_REGISTER,  0x00);
}

void max30102_set_led_pulse_width(max30102_led_pw_t pw)
{
    uint8_t config;
    ppg_read(SPO2_CONFIG_REGISTER, &config, 1);
    config = (config & 0x7c) | (pw << SPO2_LED_PW_SHIFT);
    ppg_write(SPO2_CONFIG_REGISTER, config);
}

void max30102_set_fifo_config(max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full)
{
    uint8_t config = 0x00;
    config |= smp_ave << FIFO_SMP_AVE_SHIFT;
    config |= ((roll_over_en & 0x01) << FIFO_ROLL_OVER_SHIFT);
    config |= ((fifo_a_full & 0x0f) << FIFO_A_FULL_SHIFT);
    ppg_write(FIFO_CONFIG_REGISTER, config);
}

void max30102_set_adc_resolution(max30102_adc_t adc)
{
    uint8_t config;
    ppg_read(SPO2_CONFIG_REGISTER, &config, 1);
    config = (config & 0x1f) | (adc << SPO2_ADC_RGE_SHIFT);
    ppg_write(SPO2_CONFIG_REGISTER, config);
}

void max30102_set_sampling_rate(max30102_sr_t sr)
{
    uint8_t config;
    ppg_read(SPO2_CONFIG_REGISTER, &config, 1);
    config = (config & 0x63) << SPO2_SR_SHIFT;
    ppg_write(SPO2_CONFIG_REGISTER, config);
}

void max30102_set_led_current_1(float ma)
{
    uint8_t pa = ma / 0.2;
    ppg_write(LED_CURRENT_REGISTER_1, pa);
}

void max30102_set_led_current_2(float ma)
{
    uint8_t pa = ma / 0.2;
    ppg_write(LED_CURRENT_REGISTER_2, pa);
}

void max30102_set_mode(max30102_mode_t mode)
{
    uint8_t config;
    ppg_read(MODE_CONFIG_REGISTER, &config, 1);
    config = (config & 0xf8) | mode;
    ppg_write(MODE_CONFIG_REGISTER, config);
    max30102_clear_fifo();
}

void max30102_set_a_full(uint8_t enable)
{
    uint8_t reg = 0;
    ppg_read(INTERRUPT_ENABLE_1_REGISTER, &reg, 1);
    reg &= ~(0x01 << INTERRUPT_A_FULL_BIT);
    reg |= ((enable & 0x01) << INTERRUPT_A_FULL_BIT);
    ppg_write(INTERRUPT_ENABLE_1_REGISTER, reg);
}

/* ══════════════════════════════════════════════════════════
 *  FIFO read
 * ══════════════════════════════════════════════════════════ */

#define MAX_FIFO_DEPTH 32

int max30102_read_samples(uint32_t *ir_buf, uint32_t *red_buf, int max_samples)
{
    if (max_samples <= 0)
        return 0;

    if (max_samples > MAX_FIFO_DEPTH)
        max_samples = MAX_FIFO_DEPTH;

    uint8_t raw[MAX_FIFO_DEPTH * 6];

    if (!ppg_read(FIFO_DATA_REGISTER, raw, 6 * max_samples))
        return 0;

    for (int i = 0; i < max_samples; i++)
    {
        red_buf[i] =
            (((uint32_t)raw[6*i]   << 16) |
             ((uint32_t)raw[6*i+1] << 8)  |
              (uint32_t)raw[6*i+2]) & 0x03FFFFUL;

        ir_buf[i] =
            (((uint32_t)raw[6*i+3] << 16) |
             ((uint32_t)raw[6*i+4] << 8)  |
              (uint32_t)raw[6*i+5]) & 0x03FFFFUL;
    }

    return max_samples;
}

bool max30102_read_1_sample(uint32_t *ir, uint32_t *red)
{	
    if (ir == NULL || red == NULL)
        return false;

    uint8_t raw[6];

    if (!ppg_read(FIFO_DATA_REGISTER, raw, sizeof(raw)))
        return false;

    /* LED1 (bytes 0-2) is physically IR on this PCB; LED2 (bytes 3-5) is RED */
    *ir  = (((uint32_t)raw[0] << 16) |
            ((uint32_t)raw[1] << 8)  |
             (uint32_t)raw[2]) & 0x03FFFFUL;

    *red = (((uint32_t)raw[3] << 16) |
            ((uint32_t)raw[4] << 8)  |
             (uint32_t)raw[5]) & 0x03FFFFUL;
		
    return true;
}
/* ══════════════════════════════════════════════════════════
 *  Single-sample FIFO read (legacy-style)
 * ══════════════════════════════════════════════════════════ */

void max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
    uint8_t uch_temp;
    uint8_t sample[6];

    ppg_read(INTERRUPT_STATUS_1_REGISTER, &uch_temp, 1);
    ppg_read(INTERRUPT_STATUS_2_REGISTER, &uch_temp, 1);

    ppg_read(FIFO_DATA_REGISTER, sample, 6);
    uint32_t ir_sample  = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3ffff;
    uint32_t red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3ffff;

    *pun_ir_led  = ir_sample;
    *pun_red_led = red_sample;
}

/* ══════════════════════════════════════════════════════════
 *  DC removal — sliding-window mean (populates dc_filter_t::dc_comp,
 *  needed by spo2_count). Ported from the legacy main.c dc_remove().
 * ══════════════════════════════════════════════════════════ */

static float dc_remove(dc_filter_t *f, float x)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = x;
    f->sum += x;
    f->idx = (f->idx + 1) % DC_WINDOW;
    f->dc_comp = f->sum / DC_WINDOW;
    return x - f->dc_comp;
}

/* ══════════════════════════════════════════════════════════
 *  Float ring-buffer HR/SpO2 algorithm (used by max30102_process)
 * ══════════════════════════════════════════════════════════ */

void rb_push(rb_t *rb, float value)
{
    rb->buff[rb->rb_head] = value;
    rb->rb_head = (rb->rb_head + 1) % RB_SIZE;
}

static inline float rb_get(rb_t *rb, int32_t idx)
{
    while (idx < 0)
        idx += RB_SIZE;
    idx %= RB_SIZE;
    return rb->buff[idx];
}

bool find_peak(rb_t *rb)
{
    /* candidate = newest written sample minus 5 (needs 4 neighbours each side) */
    int32_t curr_idx = (rb->rb_head + RB_SIZE - 5) % RB_SIZE;
    float   curr     = rb_get(rb, curr_idx);

    for (int i = 1; i <= 4; i++)
    {
        if (curr <= rb_get(rb, curr_idx - i))
            return false;
        if (curr <= rb_get(rb, curr_idx + i))
            return false;
    }
    return true;
}

void peak_time_push(rb_t *signal_rb, rb_t *dt_rb)
{
    static uint32_t last_peak_time = 0;

    if (find_peak(signal_rb))
    {
        uint32_t now = timer2_now();
        if (last_peak_time != 0)
        {
            uint32_t dt = now - last_peak_time;
            if (dt > 0)
                rb_push(dt_rb, (float)dt);
        }
        last_peak_time = now;
    }
}

uint32_t hr_count(rb_t *dt_rb, uint32_t peak_count)
{
    if (peak_count < 2)
        return 0;

    const float MIN_DT = 300000.0f;
    const float MAX_DT = 2000000.0f;

    uint32_t valid_intervals = 0;
    float    total_dt = 0.0f;

    uint32_t intervals = peak_count - 1;
    if (intervals > RB_SIZE)
        intervals = RB_SIZE;

    for (uint32_t i = 0; i < intervals; i++)
    {
        int32_t pos = (dt_rb->rb_head + RB_SIZE - 1 - i) % RB_SIZE;
        float   dt  = dt_rb->buff[pos];

        if (dt < MIN_DT) continue;
        if (dt > MAX_DT) continue;

        total_dt += dt;
        valid_intervals++;
    }

    if (valid_intervals == 0 || total_dt <= 0.0f)
        return 0;

    float bpm = (60.0f * 1e6f * valid_intervals) / total_dt;
    return (uint32_t)bpm;
}

uint32_t spo2_count(dc_filter_t *dc_ir, dc_filter_t *dc_red,
                    rb_t *ir_rb, rb_t *red_rb)
{
    /* AC = peak-to-peak over the filtered ring buffers */
    float ir_min = 1e9f, ir_max = -1e9f;
    float red_min = 1e9f, red_max = -1e9f;

    for (int i = 0; i < RB_SIZE; i++)
    {
        float ir  = ir_rb->buff[i];
        float red = red_rb->buff[i];

        if (ir  > ir_max)  ir_max  = ir;
        if (ir  < ir_min)  ir_min  = ir;
        if (red > red_max) red_max = red;
        if (red < red_min) red_min = red;
    }

    float ir_ac  = ir_max  - ir_min;
    float red_ac = red_max - red_min;

    /* DC from the raw sliding-mean filters */
    float ir_dc  = dc_ir->dc_comp;
    float red_dc = dc_red->dc_comp;

    if (ir_ac <= 0.0f || red_ac <= 0.0f ||
        ir_dc <= 0.0f || red_dc <= 0.0f)
        return 0;

    float R = (red_ac / red_dc) / (ir_ac / ir_dc);

    NRF_LOG_INFO("SPO2_DBG: R=%d.%03d ir_ac=%d ir_dc=%d red_ac=%d red_dc=%d",
                 (int)R, (int)((R - (int)R) * 1000),
                 (int)ir_ac, (int)ir_dc, (int)red_ac, (int)red_dc);

    float spo2 = 119.0f - 25.0f * R;
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 <   0.0f) spo2 =   0.0f;

    return (uint32_t)spo2;
}

/* ══════════════════════════════════════════════════════════
 *  Raw-uint32 ring-buffer HR/SpO2 algorithm (legacy variant,
 *  ported for parity / fallback — not wired into max30102_process)
 * ══════════════════════════════════════════════════════════ */

void ring_buffer_push(uint32_t *buff, uint32_t value, uint32_t size, uint32_t *head)
{
    buff[*head] = value;
    *head = (*head + 1) % size;
}

static inline uint32_t rb_get1(uint32_t *buff, uint32_t size, int32_t idx)
{
    if (idx < 0)            idx += size;
    if (idx >= (int32_t)size) idx -= size;
    return buff[idx];
}

bool find_peak1(uint32_t *buff, uint32_t size, uint32_t head)
{
    int32_t curr_idx1 = (head + size - 1) % size;

    uint32_t curr  = buff[curr_idx1];
    uint32_t prev1 = rb_get1(buff, size, curr_idx1 - 1);
    uint32_t prev2 = rb_get1(buff, size, curr_idx1 - 2);
    uint32_t prev3 = rb_get1(buff, size, curr_idx1 - 3);
    uint32_t prev4 = rb_get1(buff, size, curr_idx1 - 4);
    uint32_t prev5 = rb_get1(buff, size, curr_idx1 - 5);
    uint32_t prev6 = rb_get1(buff, size, curr_idx1 - 6);
    uint32_t prev7 = rb_get1(buff, size, curr_idx1 - 7);
    uint32_t prev8 = rb_get1(buff, size, curr_idx1 - 8);
    uint32_t next1 = rb_get1(buff, size, curr_idx1 + 1);
    uint32_t next2 = rb_get1(buff, size, curr_idx1 + 2);
    uint32_t next3 = rb_get1(buff, size, curr_idx1 + 3);
    uint32_t next4 = rb_get1(buff, size, curr_idx1 + 4);
    uint32_t next5 = rb_get1(buff, size, curr_idx1 + 5);
    uint32_t next6 = rb_get1(buff, size, curr_idx1 + 6);
    uint32_t next7 = rb_get1(buff, size, curr_idx1 + 7);
    uint32_t next8 = rb_get1(buff, size, curr_idx1 + 8);

    if (curr > prev1 && curr > prev2 && curr > prev3 && curr > prev4 &&
        curr > prev5 && curr > prev6 && curr > prev7 && curr > prev8 &&
        curr > next1 && curr > next2 && curr > next3 && curr > next4 &&
        curr > next5 && curr > next6 && curr > next7 && curr > next8)
    {
        return true;
    }
    return false;
}

void peak_time_push1(uint32_t *buff, uint32_t *delta_t_buff, uint32_t size,
                     uint32_t head, uint32_t *dt_head, uint32_t delta_size)
{
    static uint32_t last_peak_time = 0;

    uint32_t peak_idx = (head + size - 8 - 1) % size;

    if (find_peak1(buff, size, peak_idx))
    {
        uint32_t now = timer2_now();
        if (last_peak_time != 0)
        {
            uint32_t dt = now - last_peak_time;
            if (dt > 0)
            {
                delta_t_buff[*dt_head] = dt;
                *dt_head = (*dt_head + 1) % delta_size;
            }
        }
        last_peak_time = now;
    }
}

uint32_t hr_count1(uint32_t *delta_t_buff, uint32_t delta_size,
                   uint32_t peak, uint32_t dt_head)
{
    if (peak < 2) return 0;

    uint32_t valid_intervals = 0;
    uint64_t total_dt = 0;

    const uint32_t MIN_DT = 400000;
    const uint32_t MAX_DT = 2000000;

    uint32_t intervals = peak - 1;
    if (intervals > delta_size) intervals = delta_size;

    for (uint32_t i = 0; i < intervals; i++)
    {
        int32_t pos = (int32_t)dt_head - 1 - i;
        if (pos < 0) pos += delta_size;

        uint32_t dt = delta_t_buff[pos];
        if (dt < MIN_DT) continue;
        if (dt > MAX_DT) continue;

        total_dt += dt;
        valid_intervals++;
    }

    if (valid_intervals == 0 || total_dt == 0)
        return 0;

    float bpm = (60.0f * 1e6f * valid_intervals) / ((float)total_dt);
    return (uint32_t)bpm;
}

uint32_t spo2_count1(uint32_t *ir_buff, uint32_t *red_buff,
                     uint32_t size, uint32_t head)
{
    uint32_t idx = (head + 1) % size;

    uint32_t ir_min = 0xFFFFFFFF, ir_max = 0;
    uint32_t red_min = 0xFFFFFFFF, red_max = 0;
    uint64_t ir_sum = 0, red_sum = 0;

    for (uint32_t i = 0; i < size; i++)
    {
        uint32_t ir  = ir_buff[idx];
        uint32_t red = red_buff[idx];

        ir_sum  += ir;
        red_sum += red;

        if (ir  > ir_max)  ir_max  = ir;
        if (ir  < ir_min)  ir_min  = ir;
        if (red > red_max) red_max = red;
        if (red < red_min) red_min = red;

        idx++;
        if (idx >= size) idx = 0;
    }

    float ir_dc  = (float)ir_sum  / size;
    float red_dc = (float)red_sum / size;

    float ir_ac  = (float)(ir_max  - ir_min);
    float red_ac = (float)(red_max - red_min);

    if (ir_ac <= 0 || red_ac <= 0 || ir_dc <= 0 || red_dc <= 0)
        return 0;

    float slope = (red_ac / red_dc) / (ir_ac / ir_dc);

    if (slope <= 2.5f && slope >= 0.8f)
        slope = (slope - 0.8f) / (2.5f - 0.8f) * (0.294118f - 0.241176f) + 0.241176f;
    else if (slope < 0.8f && slope >= 0.0f)
        slope = slope / 0.8f * (0.235294f - 0.182353f) + 0.182353f;
    else if (slope > 2.5f && slope <= 10.0f)
        slope = (slope - 2.5f) / (10.0f - 2.5f) * (0.470588f - 0.3f) + 0.3f;
    else if (slope > 10.0f)
        slope = 0.529f;
    else
        slope = 0.24f;

    float spo2 = 104.0f - 17.0f * slope;
    if (spo2 > 100.0f) spo2 = 99.0f;

    return (uint32_t)spo2;
}

/* ══════════════════════════════════════════════════════════
 *  Streaming HR/SpO2 — push one raw sample, process incrementally.
 *
 *  Per sample: raw DC (sliding mean) for SpO2; DSP pipeline
 *    raw → HP 0.5 Hz → LP 5 Hz → ALE-NLMS → Savitzky-Golay → ring buffer;
 *  peaks detected on the filtered IR ring, RR intervals timed via timer2.
 *  A result is emitted once every PPG_UPDATE_PERIOD samples.
 * ══════════════════════════════════════════════════════════ */

#define HR_MIN     30.0f
#define HR_MAX     220.0f
#define SPO2_MIN   70.0f
#define SPO2_MAX   100.0f

bool max30102_process(uint32_t ir_raw, uint32_t red_raw,
                      float *hr_out, float *spo2_out)
{
    /* 0. Finger-on/off detection */
    if (ir_raw < PPG_FINGER_THRESHOLD)
    {
        if (++s_no_finger_cnt >= PPG_FINGER_DEBOUNCE && s_finger_on)
            ppg_filters_init();
        if (!s_finger_on)
        {
            *hr_out   = 0.0f;
            *spo2_out = 0.0f;
            return true;
        }
        return false;
    }
    s_no_finger_cnt = 0;
    if (!s_finger_on)
    {
        /* Preload HP filter state so the DC-removal transient starts at zero
         * instead of decaying from ~raw_value over 4+ seconds. */
        s_ir_hp.s1  = -s_ir_hp.b0  * (float)ir_raw;
        s_red_hp.s1 = -s_red_hp.b0 * (float)red_raw;
        s_finger_on = true;
    }

    /* 1. Raw DC (sliding mean) for the SpO2 R-ratio */
    dc_remove(&s_dc_ir,  (float)ir_raw);
    dc_remove(&s_dc_red, (float)red_raw);

    /* 2. DSP pipeline: HP → LP → ALE-NLMS → Savitzky-Golay */
    float ir = (float)ir_raw;
    ir = iir1_step(&s_ir_hp, ir);
    ir = iir1_step(&s_ir_lp, ir);
    //ir = ale_process(&s_ir_nlms, &s_ir_dly, ir);
    ir = sg_step(&s_ir_sg, ir);

    float red = (float)red_raw;
    red = iir1_step(&s_red_hp, red);
    red = iir1_step(&s_red_lp, red);
    //red = ale_process(&s_red_nlms, &s_red_dly, red);
    red = sg_step(&s_red_sg, red);
		
    rb_push(&s_ir_rb,  ir);
    rb_push(&s_red_rb, red);
		
		
    /* 3. Peak detection on filtered IR + RR interval accumulation.
     * 350 ms refractory period suppresses dicrotic notch (~250-350 ms after
     * systolic peak), which would otherwise double the apparent HR. */
    uint8_t peak_accepted = 0;
    if (find_peak(&s_ir_rb))
    {
        uint32_t now = timer2_now();
        uint32_t dt  = now - s_last_peak_time;

        if (s_last_peak_time == 0 || dt >= 350000U)
        {
            if (s_last_peak_time != 0 && dt > 0)
            {
                rb_push(&s_dt_rb, (float)dt);
                s_peak_count++;
            }
            s_last_peak_time = now;
            peak_accepted    = 1;
        }
    }

    /* RT stream for ppg_rt_stream.py — one line per sample at 100 Hz */
    //NRF_LOG_INFO("ppg:%d %d", (int)ir, (int)peak_accepted);

    /* 4. Emit a result once per PPG_UPDATE_PERIOD samples */
    if (s_dc_warmup > 0) s_dc_warmup--;

    if (++s_sample_ctr < PPG_UPDATE_PERIOD)
        return false;
    s_sample_ctr = 0;

    float hr   = (float)hr_count(&s_dt_rb, s_peak_count);
    /* Block SpO2 while DC window is still filling after a reset — dc_comp is
     * underestimated during warmup, which makes R wrong and SpO2 garbage. */
    float spo2 = (s_dc_warmup == 0)
               ? (float)spo2_count(&s_dc_ir, &s_dc_red, &s_ir_rb, &s_red_rb)
               : 0.0f;

    NRF_LOG_INFO("PPG: hr=%d spo2=%d peaks=%d dc_ir=%d",
                 (int)hr, (int)spo2, (int)s_peak_count, (int)s_dc_ir.dc_comp);

    if (hr < HR_MIN || hr > HR_MAX)         return false;
    if (spo2 < SPO2_MIN || spo2 > SPO2_MAX) return false;

    /* 5. 3-sample HR history smoothing (median — one bad second doesn't pull output) */
    s_hr_hist[0] = s_hr_hist[1];
    s_hr_hist[1] = s_hr_hist[2];
    s_hr_hist[2] = hr;

    float a = s_hr_hist[0], b = s_hr_hist[1], c = s_hr_hist[2];
    if (a == 0.0f || b == 0.0f)
    {
        /* fewer than 3 valid readings yet — use latest */
        *hr_out = hr;
    }
    else
    {
        /* sort three values, pick middle */
        float t;
        if (a > b) { t = a; a = b; b = t; }
        if (b > c) { t = b; b = c; c = t; }
        if (a > b) { t = a; a = b; b = t; }
        (void)a; (void)c;
        *hr_out = b;
    }

    /* 6. SpO2 3-sample median smoothing */
    s_spo2_hist[0] = s_spo2_hist[1];
    s_spo2_hist[1] = s_spo2_hist[2];
    s_spo2_hist[2] = spo2;
    float sa = s_spo2_hist[0], sb = s_spo2_hist[1], sc = s_spo2_hist[2];
    if (sa == 0.0f || sb == 0.0f)
    {
        *spo2_out = spo2;
    }
    else
    {
        float st;
        if (sa > sb) { st = sa; sa = sb; sb = st; }
        if (sb > sc) { st = sb; sb = sc; sc = st; }
        if (sa > sb) { st = sa; sa = sb; sb = st; }
        (void)sa; (void)sc;
        *spo2_out = sb;
    }
    return true;
}

/* ══════════════════════════════════════════════════════════
 *  Adaptive Derivative-Envelope (adenv) peak detector
 *  — derivative → square → fast-attack/slow-decay envelope → 40 % threshold
 *  — 8-sample median RR for HR (robust to single missed/extra beats)
 *  — SpO2 same R-ratio formula, float return
 * ══════════════════════════════════════════════════════════ */

void adenv_init(adenv_t *s)
{
    s->last_val       = 0.0f;
    s->env_peak       = 0.0f;
    s->cooldown       = 0;
    s->rr_head        = 0;
    s->last_peak_time = 0;
    s->peak_count     = 0;
    for (int i = 0; i < ADENV_RR_SIZE; i++)
        s->rr_buf[i] = ADENV_RR_SEED;
}

bool adenv_push(adenv_t *s, float sample)
{
    float diff = sample - s->last_val;
    s->last_val = sample;
    float sq = diff * diff;

    if (sq > s->env_peak)
        s->env_peak = sq;
    else
        s->env_peak *= ADENV_ENV_DECAY;

    float threshold = s->env_peak * ADENV_THRESH_RATIO;
    if (threshold < 1.0f) threshold = 1.0f;

    if (s->cooldown > 0) {
        s->cooldown--;
        return false;
    }

    if (sq <= threshold)
        return false;

    uint32_t now = timer2_now();
    if (s->last_peak_time != 0) {
        uint32_t dt = now - s->last_peak_time;
        if (dt >= ADENV_RR_MIN && dt <= ADENV_RR_MAX) {
            s->rr_buf[s->rr_head] = dt;
            s->rr_head = (s->rr_head + 1) % ADENV_RR_SIZE;
        }
    }
    s->last_peak_time = now;
    s->peak_count++;
    s->cooldown = ADENV_REFRACT;
    return true;
}

float adenv_hr(const adenv_t *s)
{
    if (s->peak_count < 2)
        return 0.0f;

    uint32_t tmp[ADENV_RR_SIZE];
    for (int i = 0; i < ADENV_RR_SIZE; i++)
        tmp[i] = s->rr_buf[i];

    for (int i = 0; i < ADENV_RR_SIZE - 1; i++)
        for (int j = 0; j < ADENV_RR_SIZE - 1 - i; j++)
            if (tmp[j] > tmp[j + 1]) {
                uint32_t t = tmp[j]; tmp[j] = tmp[j + 1]; tmp[j + 1] = t;
            }

    uint32_t med = (tmp[ADENV_RR_SIZE / 2 - 1] + tmp[ADENV_RR_SIZE / 2]) / 2;
    if (med == 0)
        return 0.0f;

    return 60.0f * 1e6f / (float)med;
}

float adenv_spo2(dc_filter_t *dc_ir, dc_filter_t *dc_red,
                 rb_t *ir_rb, rb_t *red_rb)
{
    float ir_min = 1e9f,  ir_max  = -1e9f;
    float red_min = 1e9f, red_max = -1e9f;

    for (int i = 0; i < RB_SIZE; i++) {
        float ir  = ir_rb->buff[i];
        float red = red_rb->buff[i];
        if (ir  > ir_max)  ir_max  = ir;
        if (ir  < ir_min)  ir_min  = ir;
        if (red > red_max) red_max = red;
        if (red < red_min) red_min = red;
    }

    float ir_ac  = ir_max  - ir_min;
    float red_ac = red_max - red_min;
    float ir_dc  = dc_ir->dc_comp;
    float red_dc = dc_red->dc_comp;

    if (ir_ac <= 0.0f || red_ac <= 0.0f || ir_dc <= 0.0f || red_dc <= 0.0f)
        return 0.0f;

    float R    = (red_ac / red_dc) / (ir_ac / ir_dc);
    float spo2 = 119.0f - 25.0f * R;
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 <   0.0f) spo2 =   0.0f;
    return spo2;
}

/* ── Private state for max30102_process2 ─────────────────── */

static iir1_df2t_t s2_ir_hp,  s2_ir_lp;
static nlms_t      s2_ir_nlms;
static delay_t     s2_ir_dly;
static sg_filter_t s2_ir_sg;

static iir1_df2t_t s2_red_hp, s2_red_lp;
static nlms_t      s2_red_nlms;
static delay_t     s2_red_dly;
static sg_filter_t s2_red_sg;

static rb_t        s2_ir_rb,  s2_red_rb;
static dc_filter_t s2_dc_ir,  s2_dc_red;
static adenv_t     s2_ir_adenv;
static uint32_t    s2_sample_ctr;
static uint32_t    s2_dc_warmup;
static float       s2_hr_hist[3];
static float       s2_spo2_hist[3];
static uint8_t     s2_no_finger_cnt;
static bool        s2_finger_on;

static void ppg2_filters_init(void)
{
    butter1_hp_coeffs(100.0f, 0.5f, &s2_ir_hp);
    butter1_lp_coeffs(100.0f, 5.0f, &s2_ir_lp);
    nlms_init_n(&s2_ir_nlms, PPG_NLMS_TAPS, PPG_NLMS_MU, PPG_NLMS_EPS);
    delay_init_d(&s2_ir_dly, PPG_ALE_DELAY);
    sg_init_n(&s2_ir_sg, PPG_SG_WINDOW);

    butter1_hp_coeffs(100.0f, 0.5f, &s2_red_hp);
    butter1_lp_coeffs(100.0f, 5.0f, &s2_red_lp);
    nlms_init_n(&s2_red_nlms, PPG_NLMS_TAPS, PPG_NLMS_MU, PPG_NLMS_EPS);
    delay_init_d(&s2_red_dly, PPG_ALE_DELAY);
    sg_init_n(&s2_red_sg, PPG_SG_WINDOW);

    memset(&s2_ir_rb,   0, sizeof(s2_ir_rb));
    memset(&s2_red_rb,  0, sizeof(s2_red_rb));
    memset(&s2_dc_ir,   0, sizeof(s2_dc_ir));
    memset(&s2_dc_red,  0, sizeof(s2_dc_red));
    adenv_init(&s2_ir_adenv);
    s2_sample_ctr     = 0;
    s2_dc_warmup      = DC_WINDOW;
    s2_hr_hist[0]     = 0.0f;
    s2_hr_hist[1]     = 0.0f;
    s2_hr_hist[2]     = 0.0f;
    s2_spo2_hist[0]   = 0.0f;
    s2_spo2_hist[1]   = 0.0f;
    s2_spo2_hist[2]   = 0.0f;
    s2_no_finger_cnt  = 0;
    s2_finger_on      = false;
}

void max30102_reset_filters(void)
{
    ppg_filters_init();
    ppg2_filters_init();
}

bool max30102_process2(uint32_t ir_raw, uint32_t red_raw,
                       float *hr_out, float *spo2_out)
{
    if (ir_raw < PPG_FINGER_THRESHOLD) {
        if (++s2_no_finger_cnt >= PPG_FINGER_DEBOUNCE && s2_finger_on)
            ppg2_filters_init();
        if (!s2_finger_on) {
            *hr_out   = 0.0f;
            *spo2_out = 0.0f;
            return true;
        }
        return false;
    }
    s2_no_finger_cnt = 0;
    s2_finger_on     = true;

    dc_remove(&s2_dc_ir,  (float)ir_raw);
    dc_remove(&s2_dc_red, (float)red_raw);

    float ir = (float)ir_raw;
    ir = iir1_step(&s2_ir_hp, ir);
    ir = iir1_step(&s2_ir_lp, ir);
    float ir_spo2 = ir;                              /* pre-ALE: ratio preserved */
    ir = ale_process(&s2_ir_nlms, &s2_ir_dly, ir);
    ir = sg_step(&s2_ir_sg, ir);

    float red = (float)red_raw;
    red = iir1_step(&s2_red_hp, red);
    red = iir1_step(&s2_red_lp, red);
    float red_spo2 = red;                            /* pre-ALE: ratio preserved */
    red = ale_process(&s2_red_nlms, &s2_red_dly, red);
    red = sg_step(&s2_red_sg, red);

    rb_push(&s2_ir_rb,  ir_spo2);
    rb_push(&s2_red_rb, red_spo2);

    adenv_push(&s2_ir_adenv, ir);

    if (s2_dc_warmup > 0) s2_dc_warmup--;

    if (++s2_sample_ctr < PPG_UPDATE_PERIOD)
        return false;
    s2_sample_ctr = 0;

    float hr   = adenv_hr(&s2_ir_adenv);
    float spo2 = (s2_dc_warmup == 0)
               ? adenv_spo2(&s2_dc_ir, &s2_dc_red, &s2_ir_rb, &s2_red_rb)
               : 0.0f;

    NRF_LOG_INFO("PPG2: hr=%d spo2=%d peaks=%d",
                 (int)hr, (int)spo2, (int)s2_ir_adenv.peak_count);

    if (hr < HR_MIN || hr > HR_MAX)         return false;
    if (spo2 < SPO2_MIN || spo2 > SPO2_MAX) return false;

    s2_hr_hist[0] = s2_hr_hist[1];
    s2_hr_hist[1] = s2_hr_hist[2];
    s2_hr_hist[2] = hr;

    float a = s2_hr_hist[0], b = s2_hr_hist[1], c = s2_hr_hist[2];
    if (a == 0.0f || b == 0.0f) {
        *hr_out = hr;
    } else {
        float t;
        if (a > b) { t = a; a = b; b = t; }
        if (b > c) { t = b; b = c; c = t; }
        if (a > b) { t = a; a = b; b = t; }
        (void)a; (void)c;
        *hr_out = b;
    }

    s2_spo2_hist[0] = s2_spo2_hist[1];
    s2_spo2_hist[1] = s2_spo2_hist[2];
    s2_spo2_hist[2] = spo2;
    float sa = s2_spo2_hist[0], sb = s2_spo2_hist[1], sc = s2_spo2_hist[2];
    if (sa == 0.0f || sb == 0.0f) {
        *spo2_out = spo2;
    } else {
        float st;
        if (sa > sb) { st = sa; sa = sb; sb = st; }
        if (sb > sc) { st = sb; sb = sc; sc = st; }
        if (sa > sb) { st = sa; sa = sb; sb = st; }
        (void)sa; (void)sc;
        *spo2_out = sb;
    }
    return true;
}
