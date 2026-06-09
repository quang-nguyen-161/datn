/**
 * LCD Power Management — Auto-dim/off + Wake-on-Motion
 *
 * Uses GC9A01 SLPIN/SLPOUT commands to control display power.
 * When LCD is off, all SPI display traffic is skipped in main loop.
 *
 * Savings: ~15–20 mA when LCD is in sleep mode.
 *
 * Anti-flicker: Debounce prevents LCD from toggling rapidly.
 *   - After LCD turns OFF, must wait LCD_DEBOUNCE_MS before allowing wake
 *   - Wake requires WAKE_CONFIRM_COUNT consecutive readings above threshold
 */

#include "lcd_power.h"
#include "gc9a01.h"
#include "nrf_log.h"
#include "nrf_delay.h"

/* ── Internal state ── */
static bool     s_lcd_on          = true;   /* LCD starts ON after boot        */
static uint32_t s_last_activity   = 0;      /* Last motion/event time          */
static bool     s_needs_redraw    = false;  /* Flag: dashboard needs full redraw */
static uint32_t s_off_timestamp   = 0;      /* When LCD was turned off         */
static uint8_t  s_wake_count      = 0;      /* Consecutive above-threshold cnt */

/* ── Debounce config ── */
#define LCD_DEBOUNCE_MS       2000   /* Min time LCD stays OFF before allowing wake */
#define WAKE_CONFIRM_COUNT    3      /* Need 3 consecutive readings above threshold */

/* ── Debug log throttle ── */
static uint32_t s_dbg_counter = 0;
#define DBG_LOG_EVERY         50     /* Log ac value every 50th check (~5s at 10Hz) */

/* ================================================================
 *  PUBLIC API
 * ================================================================ */

void lcd_power_init(void)
{
    s_lcd_on        = true;
    s_last_activity = 0;
    s_needs_redraw  = false;
    s_off_timestamp = 0;
    s_wake_count    = 0;
    s_dbg_counter   = 0;
    NRF_LOG_INFO("[LCD_PWR] Init — LCD ON, timeout=%d ms, debounce=%d ms",
                 LCD_TIMEOUT_MS, LCD_DEBOUNCE_MS);
}

void lcd_power_on(void)
{
    if (!s_lcd_on) {
        /* Wake LCD from sleep — frame buffer already has current data
         * (we continue SPI writes during SLPIN) */
        GC9A01A_sleep_mode(0);   /* SLPOUT */
        nrf_delay_ms(120);       /* GC9A01 cần 120ms sau SLPOUT mới hiển thị */
        s_lcd_on       = true;
        s_wake_count   = 0;
        NRF_LOG_INFO("[LCD_PWR] LCD ON (wake — charts preserved)");
    }
    /* Reset timeout regardless */
    s_last_activity = 0;  /* Will be set properly on next check_timeout call */
}

void lcd_power_off(void)
{
    if (s_lcd_on) {
        GC9A01A_sleep_mode(1);   /* SLPIN */
        s_lcd_on       = false;
        s_off_timestamp = 0;     /* Will be seeded on first wake_check call */
        s_wake_count   = 0;
        NRF_LOG_INFO("[LCD_PWR] LCD OFF (sleep)");
    }
}

bool lcd_is_on(void)
{
    return s_lcd_on;
}

void lcd_power_check_timeout(uint32_t now_ms)
{
    if (!s_lcd_on) {
        /* Seed off_timestamp if LCD just turned off */
        if (s_off_timestamp == 0) {
            s_off_timestamp = now_ms;
        }
        return;
    }

    /* First call after power_on — seed the activity timestamp */
    if (s_last_activity == 0) {
        s_last_activity = now_ms;
        return;
    }

    if ((now_ms - s_last_activity) >= LCD_TIMEOUT_MS) {
        lcd_power_off();
    }
}

void lcd_power_wake_check(float ac, uint32_t now_ms)
{
    /* Debug: log ac value periodically */
    s_dbg_counter++;
    if (s_dbg_counter >= DBG_LOG_EVERY) {
        s_dbg_counter = 0;
        NRF_LOG_INFO("[LCD_PWR] ac=" NRF_LOG_FLOAT_MARKER " lcd=%s wk=%d",
                     NRF_LOG_FLOAT(ac),
                     s_lcd_on ? "ON" : "OFF",
                     s_wake_count);
    }

    if (s_lcd_on) {
        /* LCD is on — reset timeout on any significant motion */
        if (ac > WAKE_AC_THRESHOLD) {
            s_last_activity = 0;  /* Will be re-seeded → extends timeout */
        }
        return;
    }

    /* LCD is off — check debounce first */
    if (s_off_timestamp == 0 || (now_ms - s_off_timestamp) < LCD_DEBOUNCE_MS) {
        /* Too soon after turning off — ignore motion */
        return;
    }

    /* Check threshold with confirmation counter */
    if (ac > WAKE_AC_THRESHOLD) {
        s_wake_count++;
        if (s_wake_count >= WAKE_CONFIRM_COUNT) {
            NRF_LOG_INFO("[LCD_PWR] Wake confirmed: ac=" NRF_LOG_FLOAT_MARKER
                         " count=%d", NRF_LOG_FLOAT(ac), s_wake_count);
            lcd_power_on();
        }
    } else {
        /* Below threshold — reset counter */
        s_wake_count = 0;
    }
}

bool lcd_power_needs_redraw(void)
{
    if (s_needs_redraw) {
        s_needs_redraw = false;
        return true;
    }
    return false;
}
