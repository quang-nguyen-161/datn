/* ══════════════════════════════════════════════════════════════
 *  peripheral.c — centralized low-level hardware module
 *
 *  All on-chip peripheral init, IRQ callbacks, and shared driver
 *  instances. See peripheral.h for the public API. Board pin
 *  assignments come from main.h (board pin map).
 * ════════════════════════════════════════════════════════════ */

#include "peripheral.h"
#include "main.h"           /* board pin map: TWI_SCL_PIN, ECG_SAADC_INPUT, ... */

#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_pwm.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* ══════════════════════════════════════════════════════════════
 *  TWI (I2C) — TWI1 shared bus
 * ════════════════════════════════════════════════════════════ */
nrf_drv_twi_t    m_twi        = NRF_DRV_TWI_INSTANCE(1);
volatile bool    m_xfer_done  = false;
volatile bool    m_xfer_error = false;

void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    if (p_event->type == NRF_DRV_TWI_EVT_ADDRESS_NACK ||
        p_event->type == NRF_DRV_TWI_EVT_DATA_NACK) {
        m_xfer_error = true;
    }
    m_xfer_done = true;
}

/* Spin-waits for the in-flight IRQ-driven transfer to complete.
 * Timeout ~5 ms at 64 MHz. Resets peripheral if IRQ never fires. */
bool twi_wait(void)
{
    uint32_t timeout = 200000;
    while (!m_xfer_done && --timeout);
    if (!timeout) {
        nrf_drv_twi_disable(&m_twi);
        nrf_drv_twi_enable(&m_twi);
    }
    bool ok = m_xfer_done && !m_xfer_error;
    m_xfer_done  = false;
    m_xfer_error = false;
    return ok;
}

void twi_init(void)
{
    const nrf_drv_twi_config_t cfg = {
        .scl                = TWI_SCL_PIN,
        .sda                = TWI_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };
    ret_code_t err = nrf_drv_twi_init(&m_twi, &cfg, twi_handler, NULL);
    APP_ERROR_CHECK(err);
    nrf_drv_twi_enable(&m_twi);
}

/* ══════════════════════════════════════════════════════════════
 *  SPI — SPI0 display bus (GC9A01)
 *  Bus only; LCD GPIO (RES/CS/DC) is configured in GC9A01.c.
 * ════════════════════════════════════════════════════════════ */
nrf_drv_spi_t    m_lcd_spi    = NRF_DRV_SPI_INSTANCE(0);

bool spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.miso_pin  = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.mosi_pin  = LCD_MOSI_PIN;
    spi_config.sck_pin   = LCD_SCK_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    ret_code_t err = nrf_drv_spi_init(&m_lcd_spi, &spi_config, NULL, NULL);
    if (err != NRF_SUCCESS) {
        NRF_LOG_WARNING("[SPI] init failed: 0x%08X", err);
        return false;
    }
    NRF_LOG_INFO("[SPI] init OK.");
    return true;
}

/* ══════════════════════════════════════════════════════════════
 *  SAADC + PPI + TIMER3 — ECG analog front-end (250 Hz)
 *  TIMER3 dedicated to SAADC PPI (TIMER0=SD, TIMER1=PWM, TIMER2=µs).
 * ════════════════════════════════════════════════════════════ */
#define SAADC_SAMPLE_US   4000       /* 250 Hz */
#define SAMPLES_IN_BUFFER 1

volatile int16_t g_ecg_raw   = 0;
volatile bool    g_ecg_ready = false;

static const nrf_drv_timer_t m_saadc_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t      m_buf[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t      m_ppi_chan;

void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type != NRF_DRV_SAADC_EVT_DONE) { return; }

    g_ecg_raw   = p_event->data.done.p_buffer[0];
    g_ecg_ready = true;

    /* Re-queue the same buffer; matches SAMPLES_IN_BUFFER (1) */
    ret_code_t err = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err);
}

/* SAADC: 12-bit, AIN0, GAIN1_6, internal 0.6 V ref.
 * AD8232 output 0.5–2.5 V fits inside 3.6 V full-scale range. */
static void saadc_init(void)
{
    nrf_saadc_channel_config_t ch = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ECG_SAADC_INPUT);
    ch.gain      = NRF_SAADC_GAIN1_6;
    ch.reference = NRF_SAADC_REFERENCE_INTERNAL;
    ch.acq_time  = NRF_SAADC_ACQTIME_20US;
    ch.mode      = NRF_SAADC_MODE_SINGLE_ENDED;
    ch.burst     = NRF_SAADC_BURST_DISABLED;

    nrf_drv_saadc_config_t cfg = NRF_DRV_SAADC_DEFAULT_CONFIG;
    cfg.resolution = NRF_SAADC_RESOLUTION_12BIT;
    cfg.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;

    ret_code_t err;
    err = nrf_drv_saadc_init(&cfg, saadc_callback);                  APP_ERROR_CHECK(err);
    err = nrf_drv_saadc_channel_init(0, &ch);                        APP_ERROR_CHECK(err);
    err = nrf_drv_saadc_buffer_convert(m_buf[0], SAMPLES_IN_BUFFER); APP_ERROR_CHECK(err);
    err = nrf_drv_saadc_buffer_convert(m_buf[1], SAMPLES_IN_BUFFER); APP_ERROR_CHECK(err);
    nrf_drv_saadc_calibrate_offset();
}

static void saadc_timer_handler(nrf_timer_event_t event_type, void *p_context)
{
    (void)event_type; (void)p_context;
}

/* PPI: TIMER3 compare → SAADC sample task at 250 Hz */
static void ppi_init(void)
{
    ret_code_t err;

    err = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err);

    nrf_drv_timer_config_t tcfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    tcfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err = nrf_drv_timer_init(&m_saadc_timer, &tcfg, saadc_timer_handler);
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

    err = nrf_drv_ppi_channel_alloc(&m_ppi_chan);                        APP_ERROR_CHECK(err);
    err = nrf_drv_ppi_channel_assign(m_ppi_chan, timer_evt, saadc_task); APP_ERROR_CHECK(err);
    err = nrf_drv_ppi_channel_enable(m_ppi_chan);                        APP_ERROR_CHECK(err);
}

void adc_init(void)
{
    saadc_init();
    ppi_init();
    NRF_LOG_INFO("ADC: 250 Hz PPI sampling");
    NRF_LOG_FLUSH();
}

void adc_set_sample_us(uint32_t us)
{
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_saadc_timer, us);
    nrf_drv_timer_extended_compare(&m_saadc_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    NRF_LOG_INFO("ADC sample period set to %u us (%u ticks)", (unsigned)us, (unsigned)ticks);
}

/* ══════════════════════════════════════════════════════════════
 *  TIMER2 — free-running 1 MHz µs counter
 * ════════════════════════════════════════════════════════════ */
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

/* ══════════════════════════════════════════════════════════════
 *  PWM — LCD backlight dimming on LCD_BLK_Pin (PWM0 hardware peripheral)
 *
 *  Single-channel PWM at ~1 kHz. Duty cycle 0–LCD_PWM_TOP maps to
 *  0–100 % brightness (backlight assumed active-high: pin HIGH = lit).
 *  Uses the dedicated PWM0 peripheral — does not consume a TIMER.
 * ════════════════════════════════════════════════════════════ */
#define LCD_PWM_TOP   1000u    /* 1 MHz base / 1000 → 1 kHz PWM frequency */

static nrf_drv_pwm_t            m_bl_pwm   = NRF_DRV_PWM_INSTANCE(0);
static nrf_pwm_values_common_t m_bl_duty[1];      /* live duty (EasyDMA source) */
static bool                    m_bl_ready = false;

static const nrf_pwm_sequence_t m_bl_seq =
{
    .values.p_common = m_bl_duty,
    .length          = NRF_PWM_VALUES_LENGTH(m_bl_duty),
    .repeats         = 0,
    .end_delay       = 0,
};

void pwm_init(void)
{
    nrf_drv_pwm_config_t const cfg =
    {
        .output_pins =
        {
            LCD_BLK_Pin,                 /* channel 0 → backlight */
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = LCD_PWM_TOP,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO,
    };

    if (nrf_drv_pwm_init(&m_bl_pwm, &cfg, NULL) != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("[PWM] backlight init failed");
        return;
    }
    m_bl_ready = true;
    lcd_set_brightness(100);            /* full brightness by default */
}

void lcd_set_brightness(uint8_t percent)
{
    if (!m_bl_ready) { return; }
    if (percent > 100) { percent = 100; }

    /* polarity bit (0x8000) clear → output HIGH for `duty` ticks each period,
     * so duty/LCD_PWM_TOP == brightness fraction (active-high backlight). */
    m_bl_duty[0] = (nrf_pwm_values_common_t)
                   ((uint32_t)LCD_PWM_TOP * percent / 100u);

    nrf_drv_pwm_simple_playback(&m_bl_pwm, &m_bl_seq, 1,
                                NRF_DRV_PWM_FLAG_LOOP);
}
