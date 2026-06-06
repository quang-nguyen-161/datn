#ifndef LCD_POWER_H__
#define LCD_POWER_H__

/**
 * LCD Power Management — Auto-dim/off + Wake-on-Motion
 *
 * Controls LCD (GC9A01) power state to save ~15–20 mA when display is off.
 *
 * Flow:
 *   Boot → LCD ON → Dashboard
 *     ├─ Sau timeout (10s) không motion → lcd_power_off()
 *     ├─ ac > threshold → lcd_power_on() + redraw dashboard
 *     ├─ HR finger detected → lcd_power_on() + giữ sáng khi đo
 *     └─ Alert → lcd_power_on()
 */

#include <stdbool.h>
#include <stdint.h>

/* ── Configuration ── */
#define LCD_TIMEOUT_MS      10000   /* Tắt LCD sau 10s không tương tác */
#define WAKE_AC_THRESHOLD   1.5f    /* Ngưỡng gia tốc (g) để bật LCD  */

/* ── API ── */

/**
 * Initialize LCD power management.
 * LCD starts in ON state.
 */
void lcd_power_init(void);

/**
 * Turn LCD ON and reset the auto-off timer.
 * If LCD was off, calls GC9A01 sleep-out + triggers dashboard redraw flag.
 */
void lcd_power_on(void);

/**
 * Turn LCD OFF (sleep-in command).
 * SPI traffic to display is skipped while LCD is off.
 */
void lcd_power_off(void);

/**
 * @return true if LCD is currently ON.
 */
bool lcd_is_on(void);

/**
 * Check timeout — call in main loop.
 * If LCD has been on for > LCD_TIMEOUT_MS without activity, turn it off.
 * @param now_ms  Current timestamp in milliseconds.
 */
void lcd_power_check_timeout(uint32_t now_ms);

/**
 * Check accelerometer magnitude for wake trigger.
 * Includes debounce: ignores motion for LCD_DEBOUNCE_MS after LCD turns off.
 * Requires WAKE_CONFIRM_COUNT consecutive above-threshold readings.
 * @param ac      AC component from accel_filter_get_ac().
 * @param now_ms  Current timestamp in milliseconds (for debounce).
 */
void lcd_power_wake_check(float ac, uint32_t now_ms);

/**
 * @return true if LCD just transitioned from OFF → ON (needs dashboard redraw).
 *         Cleared after first read.
 */
bool lcd_power_needs_redraw(void);

#endif /* LCD_POWER_H__ */
