# Wearable ECG Device — Firmware Description

Firmware for a wrist-worn health monitor built on the Nordic nRF52832/nRF52840 SoC.
Acquires ECG at 250 Hz via PPI-driven SAADC (zero CPU polling), applies digital filtering,
and streams processed samples over BLE to a host application. Simultaneously reads PPG
(HR + SpO2), temperature, and motion from onboard I2C sensors, and renders a live
dashboard on a 240×240 circular LCD.

---

## Hardware Platform

| Target | SoC | RAM | Flash | Softdevice |
|--------|-----|-----|-------|------------|
| pca10040 | nRF52832 | 64 KB | 512 KB | S112 / S132 |
| pca10056 | nRF52840 | 256 KB | 1 MB | S113 / S140 |

### Connected Peripherals

| Peripheral | Interface | Address / Pin | Role |
|------------|-----------|---------------|------|
| AD8232 ECG front-end | AIN0 (SAADC) | AIN0 | ECG electrode signal |
| MAX30102 | TWI1 400 kHz (I2C) | 0x57 | PPG — heart rate + SpO2 |
| MMA8452Q | TWI1 400 kHz (I2C) | 0x1C | 3-axis accelerometer |
| TMP117 | TWI1 400 kHz (I2C) | 0x48 | High-accuracy temperature |
| GC9A01 | SPI0 8 MHz | — | 240×240 circular LCD display |

#### Hardware Pins (nRF52832 / pca10040)
```
TWI1 SCL  P0.29    TWI1 SDA  P0.28
SPI0 SCK  P0.5     SPI0 MOSI P0.7
LCD CS    P1.9     LCD DC    P1.8
LCD RST   P0.8     LCD BLK   P0.11  (backlight)
```

---

## Firmware Architecture

### Central Hardware Manifest (`main.h`)

All peripheral instance handles and shared flags live in `main.h` / `main.c`.
No driver file declares its own `extern nrf_drv_twi_t m_twi` — they all `#include "main.h"`.

```c
extern nrf_drv_twi_t  m_twi;          /* TWI1 — shared bus (MAX30102 + MMA8452Q + TMP117) */
extern volatile bool  m_xfer_done;    /* TWI transfer-done flag */
extern nrf_drv_spi_t  m_lcd_spi;      /* SPI0 — GC9A01 LCD */

/* Safe replacement for while(!m_xfer_done) — times out in ~2.3 ms @ 64 MHz */
#define TWI_WAIT() \
    do { uint32_t _t = 50000; while (!m_xfer_done && --_t); } while (0)
```

### ECG Signal Chain (FS=250 Hz, hardware-autonomous)

```
TIMER3 (250 Hz tick)
    └─► PPI channel ─► SAADC trigger         ← zero CPU involvement in sampling
             └─► SAADC callback (ISR)
                     └─► raw 12-bit ADC → g_ecg.raw
                         └─► × scale (mV)
                             └─► DC removal  (sliding mean)
                                 └─► 50 Hz notch IIR
                                     └─► 2-stage biquad LPF (40 Hz corner)
                                         └─► g_ecg.filtered
                                             └─► Pan-Tompkins R-peak → g_ecg.hr_bpm
```

TIMER3 is used exclusively for SAADC PPI to avoid conflict with APP_PWM_INSTANCE (TIMER1).

### Operating Modes (`app/device_mode.c`)

Three modes controlled via BLE command or FDS-restored on boot:

| Mode | Enum | Sensor tick | BLE conn interval | ECG packet |
|------|------|-------------|-------------------|------------|
| Continuous | `MODE_CONTINUOUS` | every 10 ms | 100–200 ms | 10 samples |
| Periodic | `MODE_PERIODIC` | user-configured seconds | 100–200 ms | 10 samples |
| ECG only | `MODE_ECG` | disabled | 20–50 ms | 50 samples |

A 10 ms `app_timer` fires `sensor_timer_cb()`. In PERIODIC mode the callback counts
ticks until `g_period_ms` has elapsed, then sets `g_sensor_tick = true`. In ECG mode
the callback returns immediately — SAADC drives itself autonomously.

Selected mode and period are saved to FDS flash on every change and restored at boot.

### Sensor Result Structs

Each sensor module owns a global result struct that the main loop reads:

```c
ecg_result_t  g_ecg;   /* raw, filtered, hr_bpm, lead_off, new_data */
ppg_result_t  g_ppg;   /* hr, spo2, new_data                        */
temp_result_t g_temp;  /* temp_c, new_data                          */
accel_result_t g_accel;/* x, y, z (mg), activity, new_data          */
```

### PPG Sensor Pipeline (MAX30102)

- Sampling rate: `max30102_sr_400` (400 ADC sps) with 4× FIFO averaging → **100 FIFO sps (FS=100)**.
- Algorithm: Robert Fraczkiewicz `rf_heart_rate_and_oxygen_saturation()` — autocorrelation
  peak detection with Pearson correlation quality gate. Valid range 40–180 bpm.
  Buffer: 400 samples (ST=4 s × FS=100); `sum_X2 = 5 333 300` (precalculated for these parameters).
- Result stored in `g_ppg`; MAXIM `spo2_algorithm` sources are retained but not called.

### Accelerometer Pipeline (MMA8452Q)

- ±2 g range, ODR 100 Hz over TWI1.
- **Freefall detection** — hardware FF_MT engine configured on-chip:
  - Threshold: ~0.19 g (3 counts × 0.063 g/count at 2g scale)
  - Debounce: 60 ms (6 counts × 10 ms at ODR_100)
  - Interrupt routed to INT1 pin → falling-edge GPIOTE on `MMA8452Q_INT1_PIN` (P0.27)
  - Handler sets `g_accel.fall_detected = true`; `MMA8452Q_read()` clears the FF_MT_SRC latch
  - Override `MMA8452Q_INT1_PIN` in `mma845.h` if your schematic uses a different pin
- 4-stage software filter in `drivers/accel/accel_filter.c`:
  1. Median(3) — spike removal
  2. EMA (α = 0.5) — noise smoothing
  3. Rate limiter (±2 m/s²) — anti-glitch
  4. High-pass IIR — gravity removal, yields AC motion component
- Activity states: `REST` → `MOVING` → `WALK` → `FAST_WALK` → `RUN`

### Temperature Sensor (TMP117)

- Resolution: 0.0078 °C LSB; 8×/32×/64× averaging selectable.
- Polled on every sensor tick; result stored in `g_temp.temp_c`.
- High/low alert limits configured in registers (22 °C low, 60 °C high) but threshold
  crossing is not interrupt-driven — temperature is read unconditionally each tick.

### Display Dashboard (GC9A01 240×240 LCD)

Layout — four row zones on circular screen:
```
┌──────────────────────────────┐
│  [BLE] device / MAC  RSSI ▌▌ │  Row 1 – BLE status
├───────────────┬──────────────┤
│  Temp  36.8°C │  SpO2  98%   │  Row 2 – temperature + SpO2
├───────────────┴──────────────┤
│  ECG waveform  │  ♥ 72 bpm   │  Row 3 – live ECG + HR sweep
├──────────────────────────────┤
│  👟 1024 steps  ████░ 2.1 km │  Row 4 – steps + activity + distance
└──────────────────────────────┘
```
Font sizes: 8, 12, 16, 20, 24 pt (separate `.c` font tables).

### Flash Storage (FDS)

File ID `0x8010`, Record key `0x7010`. `configuration_t` stores:

| Field | Type | Description |
|-------|------|-------------|
| `boot_count` | uint32_t | incremented each reset |
| `device_name` | char[16] | BLE device name |
| `config1_on` | bool | reserved |
| `config2_on` | bool | reserved |
| `last_mode` | uint8_t | saved `device_mode_t` (0/1/2) |
| `period_ms_lo` | uint16_t | periodic interval low 16 bits |
| `period_ms_hi` | uint16_t | periodic interval high 16 bits |

API: `m_record_init()`, `m_record_write()`, `m_record_update()`, `m_record_read()`, `m_record_gc()`

---

## Project Layout

```
wearable_ecg/
│
├── main.c                      Entry point, TWI/SPI init, main loop (mode-aware)
├── main.h                      Central hardware manifest — all extern declarations,
│                               TWI_WAIT() macro, peripheral init prototypes
│
├── app/
│   ├── device_mode.c           3-mode state machine, 10 ms app_timer, BLE command parser
│   └── device_mode.h           device_mode_t enum, g_device_mode/g_period_ms/g_sensor_tick
│
├── ble/
│   ├── ble_app.c               BLE stack init, GAP/GATT/advertising, ble_app_send(),
│   │                           ble_app_set_conn_interval(), RX → device_mode_on_ble_rx()
│   ├── ble_app.h
│   ├── cus_service.c           Custom GATT service (TX/RX characteristics)
│   └── cus_service.h
│
├── dsp/
│   ├── filter.c                IIR1/IIR2 biquad, 50 Hz notch, 40 Hz LPF, NLMS, DC removal
│   └── filter.h                Shared by ecg/ and drivers/ — not ECG-specific
│
├── ecg/
│   ├── ecg.c                   SAADC + PPI init, ecg_process(), Pan-Tompkins R-peak,
│   │                           g_ecg result struct
│   └── ecg.h                   ecg_result_t, g_ecg extern, g_ecg_ready / g_ecg_raw
│
├── drivers/
│   ├── ppg/
│   │   ├── max.c               MAX30102 driver — I2C, FIFO, RF algorithm call, g_ppg
│   │   ├── max.h               ppg_result_t, g_ppg extern, timer2_init(), max30102_setup()
│   │   ├── algorithm_by_RF.c   Robert Fraczkiewicz HR/SpO2 algorithm (FS=25)
│   │   ├── algorithm_by_RF.h
│   │   ├── spo2_algorithm.c    MAXIM Integrated SpO2 algorithm (retained, not called)
│   │   └── spo2_algorithm.h
│   │
│   ├── accel/
│   │   ├── mma845.c            MMA8452Q driver — I2C, MMA8452Q_init(), MMA8452Q_read()
│   │   ├── mma845.h            accel_result_t, g_accel extern
│   │   ├── accel_filter.c      4-stage accelerometer filter pipeline
│   │   └── accel_filter.h
│   │
│   ├── temp/
│   │   ├── tmp117.c            TMP117 driver — I2C, ALERT GPIO interrupt, g_temp
│   │   └── tmp117.h            temp_result_t, g_temp extern, TMP117_ALERT_PIN
│   │
│   └── display/
│       ├── GC9A01.c            GC9A01 SPI LCD driver (uses m_lcd_spi from main.h)
│       ├── GC9A01.h
│       ├── dashboard.c         UI layout, widget draw, activity classification
│       ├── dashboard.h
│       ├── lcd_power.c         Backlight PWM control
│       ├── lcd_power.h
│       ├── font.c/h  fonts.h   Font base structures
│       └── font8/12/16/20/24.c Font data tables
│
└── storage/
    ├── flash_user.c            FDS wrapper — init, read, write, update, GC
    ├── flash_user.h
    └── fds_example.h           File/record ID defines, configuration_t struct
```

Keil MDK projects live in:
```
pca10040/s112/arm5_no_packs/   nRF52832 + S112
pca10040/s132/arm5_no_packs/   nRF52832 + S132  ← primary target
pca10056/s113/arm5_no_packs/   nRF52840 + S113
pca10056/s140/arm5_no_packs/   nRF52840 + S140
```

---

## Build & Flash

### Prerequisites

- **Keil MDK 5** with nRF52 pack
- **nRF5 SDK 17.1.0** (project lives inside SDK tree)
- **J-Link** programmer/debugger

### Steps (Keil)

1. Open target `.uvprojx`:
   - nRF52832: `pca10040/s132/arm5_no_packs/ble_app_uart_pca10040_s132.uvprojx`
   - nRF52840: `pca10056/s140/arm5_no_packs/ble_app_uart_pca10056_s140.uvprojx`
2. Build (`F7`).
3. Flash the matching SoftDevice first (from `nRF5_SDK_17.1.0/components/softdevice/`).
4. Flash the application (`F8`).

### Flash Pre-built Hex (nrfjprog)

```sh
nrfjprog --family NRF52 --eraseall
nrfjprog --family NRF52 --program hex/ble_app_uart_pca10040_s132.hex --verify
nrfjprog --family NRF52 --reset
```

---

## BLE Communication Protocol

Connect with any BLE central (nRF Connect app, custom Android/iOS app) and enable
notifications on the TX characteristic.

### TX (device → host)

| Byte 0 | Name | Payload |
|--------|------|---------|
| `0x10` | ECG stream | bytes 1…N: `int16_t` samples × 10 (CONTINUOUS/PERIODIC) or × 50 (ECG mode) |
| `0x11` | Vitals report | [1] HR bpm · [2] SpO2 % · [3-4] temp×100 big-endian · [5] ECG HR · [6] mode |
| `0x12` | Status reply | [1] mode · [2-3] period_ms big-endian |

### RX (host → device)

| Byte 0 | Command | Payload |
|--------|---------|---------|
| `0x01` | Set mode | [1] mode enum (0=CONTINUOUS, 1=PERIODIC, 2=ECG) |
| `0x02` | Set period | [1-2] interval seconds, big-endian |
| `0x03` | Request status | — (device replies with `0x12`) |

---

## Dependencies (nRF5 SDK 17.1.0 modules)

| Module | Purpose |
|--------|---------|
| `nrf_drv_saadc` | 12-bit ADC for ECG |
| `nrf_drv_ppi` | Hardware PPI for autonomous SAADC triggering |
| `nrf_drv_timer` | TIMER3 at 250 Hz for PPI tick |
| `nrf_drv_twi` | I2C bus shared by MAX30102, MMA8452Q, TMP117 |
| `nrf_drv_spi` | SPI0 for GC9A01 LCD |
| `nrf_drv_gpiote` | TMP117 ALERT interrupt, (future) button |
| `app_timer` | 10 ms sensor base timer + BLE timers |
| `app_scheduler` | Deferred events from ISR |
| `ble_advertising` | BLE advertising |
| `ble_conn_params` | Connection parameter negotiation |
| `nrf_ble_gatt` | GATT layer |
| `nrf_ble_qwr` | Queued Writes |
| `fds` | Flash Data Storage — config + mode persistence |
| `bsp_btn_ble` | Board button support |
| `nrf_pwr_mgmt` | Power management (WFE sleep) |
| `nrf_log` | RTT/UART logging |

---

## Authors

- Firmware: xuaaan
- HR/SpO2 algorithm: Robert Fraczkiewicz (`algorithm_by_RF`)
- SpO2 algorithm (alternative, retained): MAXIM Integrated (`spo2_algorithm`)
- nRF5 SDK 17.1.0: Nordic Semiconductor
