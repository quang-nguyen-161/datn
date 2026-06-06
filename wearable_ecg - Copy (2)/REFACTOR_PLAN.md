# Refactor Plan — Full Scope

Review every section. Reply **"go"** when ready or ask to drop/change anything.

---

## New Folder Structure

```
wearable_ecg/
├── main.h              ← NEW: central hardware manifest
├── main.c              ← updated
├── app/                ← NEW: application-level logic
│   ├── device_mode.c
│   └── device_mode.h
├── ble/
│   ├── ble_app.c       ← updated: connection interval per mode
│   ├── ble_app.h
│   ├── cus_service.c   ← updated: extended RX packet protocol
│   └── cus_service.h
├── ecg/
│   ├── ecg.c           ← updated: lead-off detection
│   └── ecg.h           ← updated: add LOD pin defines
├── dsp/                ← NEW: moved from ecg/
│   ├── filter.c
│   └── filter.h
├── drivers/
│   ├── ppg/
│   │   ├── max.c       ← updated: 25 sps, uint16 buffers, one algorithm
│   │   ├── max.h       ← updated: #include main.h, uint16 buffers
│   │   ├── algorithm_by_RF.c   ← KEEP (chosen algorithm)
│   │   ├── algorithm_by_RF.h
│   │   ├── spo2_algorithm.c    ← REMOVE (duplicate, replaced by RF)
│   │   └── spo2_algorithm.h    ← REMOVE
│   ├── accel/
│   │   ├── mma845.c    ← updated: #include main.h, type fix
│   │   ├── mma845.h    ← updated: #include main.h
│   │   ├── accel_filter.c
│   │   └── accel_filter.h
│   ├── temp/
│   │   ├── tmp117.c    ← updated: bug fix + ALERT interrupt
│   │   └── tmp117.h    ← updated: #include main.h
│   └── display/
│       ├── GC9A01.c    ← updated: SPI instance lifted to main.h
│       ├── GC9A01.h    ← updated
│       ├── dashboard.c ← updated: ECG waveform page
│       ├── dashboard.h
│       ├── lcd_power.c
│       └── lcd_power.h
└── storage/
    ├── flash_user.c    ← updated: persist mode + interval + name
    └── flash_user.h    ← updated: extend configuration_t
```

---

## Part 1 — Code Architecture (original plan)

### 1.1 `main.h` — Central Hardware Manifest

One file that declares every peripheral instance, init, and handler.
Every driver `#include "main.h"` instead of writing its own `extern` lines.

```c
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"

/* ── TWI (I2C) shared bus ──────────────────────────────────────
 *  Serves: MAX30102 (0x57)  MMA8452Q (0x1C)  TMP117 (0x48)
 *  Instance: TWI1  SCL=P0.29  SDA=P0.28  400 kHz            */
extern nrf_drv_twi_t    m_twi;
extern volatile bool     m_xfer_done;
void twi_init(void);            /* defined in main.c */

/* ── SPI — display bus ─────────────────────────────────────────
 *  Serves: GC9A01 240×240 LCD
 *  Instance: SPI0  8 MHz                                      */
extern nrf_drv_spi_t    m_lcd_spi;
void lcd_spi_init(void);        /* defined in drivers/display/GC9A01.c */

/* ── SAADC — ECG (module-internal, listed for reference) ───────
 *  250 Hz autonomous via TIMER3 → PPI → SAADC
 *  Fully owned by ecg/ecg.c via ecg_init()                   */

/* ── PWM — reserved (not yet implemented) ──────────────────── */

#endif /* MAIN_H */
```

Notes:
- `twi_handler` stays `static` in `main.c` — nothing outside needs to call it
- `m_lcd_spi` must be non-`const` in `main.c` because `nrf_drv_spi_init()` writes to it
- SAADC has no instance handle in the SDK (singleton), so no extern needed

### 1.2 Per-File Changes

**`main.c`**
- Add `#include "main.h"`
- Change TWI to 400 kHz: `.frequency = NRF_DRV_TWI_FREQ_400K`
- Call `lcd_spi_init()` before `ble_app_init()`
- Add sensor-tick flag and polling guard (see Part 2)

**`ecg/ecg.c`**
- `#include "filter.h"` path unchanged — compiler finds `dsp/filter.h` via new include path
- Add lead-off detection (see Part 3-D)

**`drivers/ppg/max.h` + `max.c`**
- Replace `extern nrf_drv_twi_t m_twi` → `#include "main.h"`
- Change buffer types: `uint32_t irBuffer[100]` → `uint16_t irBuffer[50]` (saves 600 bytes RAM)
- Remove `maxim_heart_rate_and_oxygen_saturation()` call — use RF algorithm only
- Change sample rate to 25 sps via `set_sampling_rate()`
- Remove `spo2_algorithm.c/.h` from project (and from .uvprojx groups)

**`drivers/accel/mma845.h` + `mma845.c`**
- Remove `extern nrf_drv_twi_t m_twi;` (line 8)
- Remove `extern volatile uint8_t m_xfer_done;` (line 10) — type was wrong (`uint8_t` vs `bool`)
- Add `#include "main.h"` in mma845.h

**`drivers/temp/tmp117.h` + `tmp117.c`** (bug fix + enhancement)
- Remove `static const nrf_drv_twi_t m_twi_tmp117 = NRF_DRV_TWI_INSTANCE(1)` — shadow TWI1 handle
- Remove both duplicate `extern volatile bool m_xfer_done` declarations (lines 11 and 19)
- Replace all `m_twi_tmp117` → `m_twi` (2 occurrences)
- Add `#include "main.h"` via tmp117.h
- Add TMP117 ALERT interrupt (see Part 3-H)

**`drivers/display/GC9A01.h` + `GC9A01.c`**
- Remove `static const nrf_drv_spi_t lcd_spi = NRF_DRV_SPI_INSTANCE(0)`
- Add `#include "main.h"` → provides `extern nrf_drv_spi_t m_lcd_spi`
- Rename all `lcd_spi` → `m_lcd_spi` throughout file
- Add `void lcd_spi_init(void)` declaration to GC9A01.h

### 1.3 `dsp/` Folder — Filter Move

- Copy `ecg/filter.c` → `dsp/filter.c`
- Copy `ecg/filter.h` → `dsp/filter.h`
- Delete `ecg/filter.c` and `ecg/filter.h`
- All `#include "filter.h"` lines unchanged — compiler finds it via new `dsp/` include path

### 1.4 Sensor Structs (§7 from original)

Every sensor module gets a result struct for clean data passing:

```c
/* drivers/ppg/max.h */
typedef struct {
    uint8_t  hr;          /* BPM */
    uint8_t  spo2;        /* % */
    bool     hr_valid;
    bool     spo2_valid;
    uint32_t timestamp_ms;
} ppg_result_t;
extern ppg_result_t g_ppg;

/* drivers/accel/mma845.h */
typedef struct {
    float    ax, ay, az;  /* g */
    float    magnitude;   /* m/s² */
    float    ac;          /* AC component (gravity removed) */
    bool     new_data;
    uint32_t timestamp_ms;
} accel_result_t;
extern accel_result_t g_accel;

/* drivers/temp/tmp117.h */
typedef struct {
    float    temp_c;
    bool     new_data;
    bool     alert;       /* threshold crossed */
    uint32_t timestamp_ms;
} temp_result_t;
extern temp_result_t g_temp;

/* ecg/ecg.h */
typedef struct {
    int16_t  raw;
    float    filtered;
    uint8_t  hr_bpm;      /* on-device R-peak detector result */
    bool     lead_off;    /* LOD+ or LOD- high */
    bool     new_data;
} ecg_result_t;
extern ecg_result_t g_ecg;
```

### 1.5 Keil `.uvprojx` Changes (all 4 files)

| Change | Detail |
|--------|--------|
| Add include path `..\..\..\dsp` | Front of IncludePath |
| Add include path `..\..\..\app` | Front of IncludePath |
| Add group **DSP** | Contains `dsp\filter.c` |
| Add group **App** | Contains `app\device_mode.c` |
| Remove `filter.c` from ECG group | Moved to DSP |
| Update FilePath: `ecg\filter.c` → `dsp\filter.c` | |
| Add `main.h` to Application group | Visible in project tree |
| Remove `spo2_algorithm.c` from PPG group | Replaced by RF algorithm |
| Remove `spo2_algorithm.h` from PPG group | |

---

## Part 2 — Three Operating Modes (§8)

### 2.1 Mode Definitions

New file: `app/device_mode.h`

```c
#ifndef DEVICE_MODE_H
#define DEVICE_MODE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MODE_CONTINUOUS,   /* 10 ms timer: all sensors, HR_PPG + SpO2 + temp + fall detect */
    MODE_PERIODIC,     /* same sensors, user-set interval via BLE                       */
    MODE_ECG           /* ECG only: 250 Hz stream, on-device HR, LCD waveform           */
} device_mode_t;

extern device_mode_t g_device_mode;

void device_mode_init(void);
void device_mode_set(device_mode_t mode);
void device_mode_set_period_ms(uint16_t ms);
uint16_t device_mode_get_period_ms(void);

#endif
```

New file: `app/device_mode.c`

- `device_mode_set()` starts/stops the sensor timer and calls `ble_app_set_conn_interval()`
- Saves new mode to FDS immediately on change
- Calls `lcd_power_on()` and triggers a full dashboard redraw on mode switch

### 2.2 Sensor Polling — Timer + Flag (no ISR reads)

TWI is not ISR-safe in legacy mode. The timer only sets a flag:

```c
/* in main.c */
static volatile bool     g_sensor_tick = false;
static app_timer_id_t    m_sensor_timer;

static void sensor_timer_cb(void *ctx)
{
    g_sensor_tick = true;   /* flag only — no I2C here */
}

/* main loop: */
if (g_sensor_tick && g_device_mode != MODE_ECG)
{
    g_sensor_tick = false;
    if (m_xfer_done)        /* guard: previous transfer must be done */
    {
        max30102_poll();    /* reads FIFO, updates g_ppg */
        MMA8452Q_read();    /* updates g_accel */
        /* TMP117: read only on ALERT interrupt, not here */
        dashboard_update(); /* refresh LCD */
        ble_send_vitals();  /* send one structured BLE packet */
    }
}
```

### 2.3 BLE RX Packet Protocol

Extend `cus_service.c` RX handler to parse command packets:

```
Byte 0 — command type:
  0x01  set mode       byte 1 = mode enum (0=continuous, 1=periodic, 2=ecg)
  0x02  set period     bytes 1-2 = interval ms, big-endian (periodic mode only)
  0x03  request status (device replies with one status notification)

Byte 0 — TX notification type (first byte of every outgoing packet):
  0x10  ECG stream     bytes 1-N = int16_t samples (N = up to 100 bytes = 50 samples)
  0x11  vitals report  HR, SpO2, temp, steps, activity (structured)
  0x12  status reply   current mode, period, firmware version
```

### 2.4 Mode-Specific Behavior

**MODE_CONTINUOUS**
- `app_timer` at 10 ms (100 Hz tick)
- Each tick: read MAX30102 FIFO (25 sps hardware), MMA8452Q (reads all 3 axes)
- TMP117 reads only on ALERT interrupt
- BLE: send vitals packet every 1 s (100 ticks)
- LCD: full dashboard, updates at ~8 FPS

**MODE_PERIODIC**
- `app_timer` at user period (min 1 s, max 3600 s, stored in FDS)
- Same sensor reads as CONTINUOUS but at the lower rate
- BLE: send one vitals packet per period
- LCD: update on each read

**MODE_ECG**
- Stop sensor timer (MAX30102, MMA8452Q not polled)
- ECG SAADC already runs at 250 Hz via hardware PPI — no change needed
- On-device R-peak detector gives live HR for LCD
- LCD: full-width ECG waveform page (replaces dashboard)
- BLE: 50-sample ECG packets (100 bytes) at 200 ms intervals (5 packets/sec × 50 = 250 Hz)
- Connection interval: switch to 20 ms for this mode

### 2.5 Mode via Button

Use one GPIO with `nrf_drv_gpiote` sense (zero-power between presses):
- Short press (< 1 s): cycle MODE_CONTINUOUS → MODE_PERIODIC → MODE_ECG → ...
- Long press (≥ 2 s): toggle LCD on/off (backlight save)

---

## Part 3 — Signal Quality + Power Improvements

### 3-A: I2C Bus Guard (Critical for Modes)

Add to every sensor read function:
```c
if (!m_xfer_done) return false;   /* previous transfer in flight */
```
And replace all `while (!m_xfer_done);` spin-waits with a timeout counter:
```c
uint32_t timeout = 10000;
while (!m_xfer_done && --timeout);
if (!timeout) { /* log error, reset TWI */ return false; }
```

### 3-B: Remove Duplicate HR Algorithm

`max.c` currently calls both `maxim_heart_rate_and_oxygen_saturation()` (Maxim) and the RF algorithm. Remove the Maxim call. Keep only `rf_heart_rate_and_oxygen_saturation()`. Delete `spo2_algorithm.c/.h` from the project.

### 3-C: BLE Connection Interval for ECG Mode

In `ble_app.c`, add a function to change connection interval at runtime:
```c
void ble_app_set_conn_interval(uint16_t min_ms, uint16_t max_ms);
```
Call in `device_mode_set()`:
- CONTINUOUS / PERIODIC: min=100 ms, max=200 ms (power-friendly)
- ECG: min=20 ms, max=50 ms (throughput for 250 Hz stream)

### 3-D: Lead-Off Detection (AD8232 LOD Pins)

Add to `ecg/ecg.h`:
```c
#define ECG_LOD_PLUS_PIN   XX   /* set to your GPIO pin */
#define ECG_LOD_MINUS_PIN  XX
```

In `ecg_init()`: configure both pins as GPIO inputs.

In `ecg_process()` (or main loop before calling it):
```c
g_ecg.lead_off = nrf_gpio_pin_read(ECG_LOD_PLUS_PIN) ||
                 nrf_gpio_pin_read(ECG_LOD_MINUS_PIN);
if (g_ecg.lead_off) {
    /* skip filter (avoids DC window corruption), send invalid flag */
    return;
}
```

### 3-E: On-Device R-Peak Detector (Pan-Tompkins simplified)

Add to `ecg/ecg.c` — gives local HR for LCD without needing server:

```c
/* Called once per filtered ECG sample */
static uint8_t rpeak_detect(float x)
{
    static float   threshold = 500.0f;
    static uint32_t last_peak_tick = 0;
    static float   peak_max = 0.0f;
    uint32_t now = tick_ms();   /* app_timer ticks */

    if (x > threshold && x > peak_max) { peak_max = x; }

    if (peak_max > threshold && x < threshold * 0.5f) {
        uint32_t rr_ms = now - last_peak_tick;
        last_peak_tick = now;
        peak_max = 0.0f;
        threshold = threshold * 0.7f + peak_max * 0.3f;  /* adaptive */
        if (rr_ms > 300 && rr_ms < 2000)                 /* 30–200 BPM */
            return (uint8_t)(60000 / rr_ms);
    }
    return 0;   /* 0 = no beat this sample */
}
```

Result stored in `g_ecg.hr_bpm`. Dashboard shows it in ECG mode.

### 3-F: TWI Speed — 400 kHz

One-line change in `twi_init()` in `main.c`:
```c
.frequency = NRF_DRV_TWI_FREQ_400K,   /* was FREQ_100K */
```
All three sensors support 400 kHz. Reduces per-sensor read from ~600 µs to ~150 µs.
This is critical for fitting MAX + MMA reads inside the 10 ms polling window.

### 3-G: Persist Mode + Interval in FDS

Extend `configuration_t` in `storage/flash_user.h`:
```c
typedef struct {
    uint32_t  boot_count;
    uint8_t   last_mode;         /* restore after power cycle */
    uint16_t  period_ms;         /* periodic mode interval    */
    char      device_name[16];   /* BLE advertised name       */
    uint8_t   padding[1];        /* keep struct 4-byte aligned */
} configuration_t;
```
Call `m_record_update()` in `device_mode_set()` and `device_mode_set_period_ms()`.
On boot, read saved mode and apply it before `ble_app_advertising_start()`.

### 3-H: TMP117 ALERT Interrupt

TMP117 has a hardware ALERT pin that fires when temperature crosses a programmed limit.
Instead of polling TMP117 every 10 ms, configure it once and only read on interrupt:

```c
/* in tmp117_init(): */
tmp117_set_HighLimit(38.0f);   /* fever threshold */
tmp117_set_LowLimit(35.0f);    /* hypothermia threshold */
tmp117_continuous_mode();

/* configure GPIO sense on ALERT pin: */
nrf_drv_gpiote_in_config_t alert_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
nrf_drv_gpiote_in_init(TMP117_ALERT_PIN, &alert_cfg, tmp117_alert_handler);

/* handler: */
static void tmp117_alert_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    g_temp.alert = true;   /* read in main loop, not here */
}
```

In main loop:
```c
if (g_temp.alert) {
    g_temp.alert = false;
    g_temp.temp_c = tmp117_get_temp();
    g_temp.new_data = true;
}
```

### 3-I: MAX30102 Sample Rate — 25 sps

In `max30102_init()` or `max30102_setup()`, change:
```c
set_sampling_rate(MAX30102_SAMPLERATE_25);   /* was 100 sps */
```
The SpO2 algorithm needs only 25 sps (100-sample buffer = 4 s). This:
- Cuts MAX30102 current from ~600 µA to ~200 µA (3× improvement)
- Reduces I2C read size per poll by 4×
- Halves FIFO interrupt frequency

Buffer size can drop from `[100]` to `[50]` (2 s @ 25 sps — enough for the algorithm).
Change `uint32_t irBuffer[100]` → `uint16_t irBuffer[50]` (saves 600 bytes RAM).
MAX30102 ADC is 18-bit; after right-shifting 2 bits, values fit in `uint16_t`.

---

## Part 4 — Bug Fixes (must-fix)

| File | Bug | Fix |
|------|-----|-----|
| `tmp117.c` | `static const nrf_drv_twi_t m_twi_tmp117 = NRF_DRV_TWI_INSTANCE(1)` — private shadow TWI1 handle, never initialized | Remove; use `m_twi` from `main.h` |
| `tmp117.c` | `extern volatile bool m_xfer_done` declared twice (lines 11 and 19) | Remove both; use `main.h` |
| `mma845.c` | `extern volatile uint8_t m_xfer_done` — wrong type (should be `bool`) | Remove; use `main.h` |
| `tmp117.c` + `mma845.c` + `max.c` | `while (!m_xfer_done);` — hangs on sensor disconnect | Replace with timeout counter |
| `max.c` | Both Maxim and RF algorithms called on same buffer | Remove Maxim call; keep RF only |

---

## Part 5 — Files NOT Changed

- `ble/ble_app.h` — only additions (new `ble_app_set_conn_interval()`)
- `ble/cus_service.h` — only additions
- `ecg/ecg.h` — only additions (LOD pins, `ecg_result_t`)
- `drivers/ppg/algorithm_by_RF.c/.h` — unchanged (pure math)
- `drivers/accel/accel_filter.c/.h` — unchanged (pure math)
- `drivers/display/dashboard.c/.h` — minor addition for ECG waveform page
- `drivers/display/lcd_power.c/.h` — unchanged
- `storage/flash_user.c` — minor change to `configuration_t` size only

---

## Implementation Order

If you say "go" I will do this in order. Each step compiles before the next starts.

```
Step 1  dsp/ folder — move filter files, update .uvprojx, verify build
Step 2  main.h — create file, update all drivers (externs removed), verify build
Step 3  Bug fixes — tmp117 TWI handle, mma845 type, timeout guards
Step 4  Sensor structs — add result structs to each driver header
Step 5  TWI 400 kHz — one line, verify sensors still respond
Step 6  Remove Maxim algorithm — delete spo2_algorithm, update max.c
Step 7  MAX30102 25 sps + uint16 buffers
Step 8  TMP117 ALERT interrupt
Step 9  Lead-off detection (AD8232 LOD pins) — requires you to tell me the GPIO pin numbers
Step 10 On-device R-peak detector
Step 11 FDS — extend configuration_t, save/restore mode
Step 12 Device mode system — app/device_mode.c/.h, 3-mode state machine
Step 13 BLE protocol extension — RX command parsing, TX packet types
Step 14 Button GPIO for mode switching — requires you to tell me the GPIO pin number
Step 15 BLE connection interval switching per mode
Step 16 ECG mode — LCD waveform page, 50-sample BLE packets
```

**Before step 9** I need: LOD+ pin and LOD− pin numbers from your schematic.
**Before step 14** I need: button GPIO pin number.

---

Reply **"go"** to start from Step 1, or tell me which steps to skip or reorder.
