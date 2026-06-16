#ifndef DEVICE_MODE_H
#define DEVICE_MODE_H

#include <stdint.h>
#include <stdbool.h>

/* ══════════════════════════════════════════════════════════════
 *  Operating modes
 * ════════════════════════════════════════════════════════════ */
typedef enum {
    MODE_CONTINUOUS = 0,    /* 10 ms timer: MAX30102 + MMA8452Q + TMP117 alert */
    MODE_PERIODIC   = 1,    /* same sensors, user-configured interval           */
    MODE_ECG        = 2,    /* ECG only, 250 Hz stream, LCD waveform            */
    MODE_COUNT
} device_mode_t;

/* ── Current state ── */
extern device_mode_t g_device_mode;
extern uint16_t      g_period_ms;    /* MODE_PERIODIC wake-to-wake interval (ms), 5000–60000 */
extern uint16_t      g_capture_ms;   /* MODE_PERIODIC measurement window (ms), ≥5000 ≤period  */

/* ── Sensor-tick flag — set by app_timer, consumed in main loop ── */
extern volatile bool g_sensor_tick;

/* ── PERIODIC: set true at end of each capture window — tells main loop to
 *    emit one averaged vital packet for this cycle. Consumed in main loop. ── */
extern volatile bool g_periodic_send_due;

/* ── PERIODIC: set true on capture↔sleep phase transitions — tells main loop
 *    to power down/restore MAX30102 + LCD backlight. Consumed in main loop. ── */
extern volatile bool g_periodic_enter_sleep;
extern volatile bool g_periodic_enter_capture;

/* ── PERIODIC: true while in the sleep phase (between enter_sleep and
 *    enter_capture) — tells main loop to skip LCD redraws. ── */
extern volatile bool g_periodic_sleeping;

/* ── API ── */
void     device_mode_init(void);                    /* call once, reads FDS */
void     device_mode_set(device_mode_t mode);       /* switches mode, saves to FDS */
void     device_mode_set_period(uint16_t seconds);  /* periodic interval (5–60 s), saves */
void     device_mode_set_capture(uint16_t seconds); /* periodic capture window (≥5 s), saves */
uint16_t device_mode_get_period(void);

/* ── BLE RX command protocol (called from cus_service RX handler) ──
 *   Byte 0 = command type:
 *     0x01  set mode       byte 1 = mode enum
 *     0x02  set period     bytes 1-2 = interval seconds, big-endian
 *     0x03  request status (device replies with 0x12 status packet)
 */
void device_mode_on_ble_rx(uint8_t const *data, uint16_t len);

#endif /* DEVICE_MODE_H */
