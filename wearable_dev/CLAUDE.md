# Wearable ECG / Health Monitor — nRF5 SDK 17.1.0

## Project overview
Firmware for the nRF52832 wearable health monitor node (PCA10040, s132 SoftDevice).
Sensors: AD8232 ECG, MAX30102 PPG/SpO2, MMA8452Q accelerometer, TMP117 temperature, GC9A01 round LCD.
Streams ECG waveform + vitals over BLE to an ESP32 gateway, which publishes to ThingsBoard → VitalSync Next.js dashboard.

---

## Hardware

| Component | Interface | Pin(s) |
|-----------|-----------|--------|
| AD8232 ECG | SAADC AIN0 | P0.02 |
| MAX30102 PPG/SpO2 | I2C TWI1 | SCL P0.29 / SDA P0.28 |
| MMA8452Q accel | I2C TWI1 | shared bus |
| TMP117 temp | I2C TWI1 | shared bus (addr 0x48) |
| GC9A01 LCD 240×240 | SPI0 | CLK P0.27 / MOSI P0.26 / CS P0.25 / DC P0.24 / RST P0.23 |
| MAX30102 INT | GPIO input | P0.11 |

---

## Timer Allocation — do not change without resolving all conflicts

| Timer | Owner |
|-------|-------|
| TIMER0 | SoftDevice (reserved) |
| TIMER1 | unused (reserved) |
| TIMER2 | `timer2_now()` µs counter (peripheral.c) |
| TIMER3 | SAADC PPI 250 Hz ECG (peripheral.c) — **never reassign** |

LCD backlight is plain GPIO on/off on `LCD_BLK_Pin` (`pwm_init()` / `lcd_set_brightness()` in peripheral.c — names kept for call-site compatibility, no PWM/TIMER1 involved).

---

## Source files

| Path | Purpose |
|------|---------|
| `main.c / main.h` | Top-level init, main loop, vitals BLE send; main.h holds the **board pin map** (per-target `NRF52840_XXAA`/nRF52832 macros) + `sensor_data_t` |
| `peripheral/peripheral.c/.h` | **All on-chip peripheral init/callbacks/instances**: TWI1 (`twi_init`/`twi_wait`/`m_twi`), SPIM0 (`spi_init`/`m_lcd_spi`, 8 MHz — nRF52832 max), SAADC+PPI+TIMER3 (`adc_init`/`adc_set_sample_us`/`saadc_callback`/`g_ecg_raw`/`g_ecg_ready`), TIMER2 (`timer2_init`/`timer2_now`), LCD backlight GPIO on/off (`pwm_init`/`lcd_set_brightness`) |
| `ecg/ecg.c` | ECG DSP pipeline + R-peak v1+v2 (calls `adc_init()` from peripheral.c); `hr_ecg_valid` gated by `g_ecg_stream_enabled` |
| `ecg/ecg.h` | `ecg_init()`, `ecg_process()`, ECG result struct |
| `cmd/cmd.c` | Gateway RX command parser — CMD_ECG_CFG / CMD_THR / CMD_PPG_CFG / CMD_VITAL_CFG / CMD_NAME_CFG |
| `cmd/cmd.h` | Command codes 0xCF/CE/CD/CC/C9, all volatile config globals |
| `ble/ble_app.c/h` | GAP/connection handling, advertising; `ble_app_get_addr()` (own BLE address) and `ble_app_get_rssi()` (latest RSSI via `BLE_GAP_EVT_RSSI_CHANGED`, RSSI reporting started on connect) |
| `ble/cus_service.c/h` | Custom GATT service — TX notify 0x1401, RX write 0x1402 |
| `app/device_mode.c/h` | Operating mode FSM (CONTINUOUS/PERIODIC/ECG), FDS persistence, `g_sensor_tick` |
| `dsp/filter.c/h` | Biquad DF2T, NLMS ALE, Savitzky-Golay, bilinear coeff calculators |
| `drivers/ppg/max.c` | MAX30102 I2C driver: init, FIFO read, streaming `max30102_process()`, register setters (uses `timer2_now()` from peripheral.c) |
| `drivers/ppg/max30102.h` | Register map, mode/SR/PW enums, public API including `max30102_set_sampling_rate/led_current` |
| `drivers/accel/mma845.c/h` | MMA8452Q I2C driver |
| `drivers/accel/pedometer.c/h` | Dynamic-threshold step counter, cadence EMA |
| `drivers/display/GC9A01.c/h` | GC9A01 LCD driver (SPI0 bus via `spi_init()` in peripheral.c; LCD GPIO + draw ops here) |
| `drivers/display/dashboard.c/h` | 4-row UI layout V4: Row1 BLE status + battery icon / Row2 Temp+SpO2 (bigger numbers) / Row3 HR sweep+bigger number (right) / Row4 ECG sweep+ON/OFF badge+bigger ECG-HR number (right). See [LCD_DASHBOARD.md](LCD_DASHBOARD.md) |
| `drivers/temp/tmp117_v2.c/h` | TMP117 I2C driver (one-shot mode, 200 ms conversion) |
| `drivers/temp/temp_filter.c/h` | median(3) + rate-limit + EMA(α=0.3) temperature smoother |
| `storage/flash_user.c/h` | FDS-based persistent config (device mode, period) |

---

## ECG Signal Chain (250 Hz)

```
TIMER3 CC0 → PPI → SAADC AIN0   (no CPU — fires every 4 ms)
  ISR: g_ecg_raw = sample; g_ecg_ready = true
  main loop: if g_ecg_raw < 1000 → electrode disconnected, skip filter chain,
             set g_sensor.hr_ecg_valid = false (no buffer/HR update this sample)
  else call ecg_process(g_ecg_raw):
    4th-order notch @ 50 Hz   (2× biquad_df2t_t, Q=10)
    4th-order bandpass 1–25 Hz (2× HP biquad + 2× LP biquad, Butterworth)
    ALE-NLMS                   (32 taps, delay=15)
    Savitzky-Golay             (window=11)
    R-peak v1: Pan-Tompkins adaptive threshold
    R-peak v2: derivative² + adaptive envelope + median(8) RR → g_sensor.hr_ecg
               (only updates hr_ecg/hr_ecg_valid when g_ecg_stream_enabled)
  → s_ecg_buf[]  (ECG_BUF_SAMPLES=500 max)
  → BLE notify when s_ecg_idx >= g_cmd_pkt_samples (default 50, 100 bytes)
```

---

## BLE

- Device name: `ECG_dev`
- Custom service UUID: `6e401400-b5a3-f393-e0a9-e50e24dcca9e`
- TX (0x1401): node → gateway — ECG batches (2N bytes, N=g_cmd_pkt_samples) + vitals (16 bytes, 4×float32 LE)
- RX (0x1402): gateway → node — config commands (see cmd.h)
- Advertising: 40 ms interval, 180 s duration, auto-restart on idle
- Connection: 8 ms interval (fixed), MTU 247 bytes negotiated
- RSSI: `sd_ble_gap_rssi_start()` is called on connect; `BLE_GAP_EVT_RSSI_CHANGED` updates the cached value read by `ble_app_get_rssi()`. Own address via `ble_app_get_addr()` (`sd_ble_gap_addr_get`). Both feed the LCD Row 1 status (signal bars + address fallback).

---

## Gateway Command Subsystem (cmd/cmd.c)

All commands arrive via BLE RX char 0x1402 → `cus_data_handler()` → `cmd_rx_handle()`.
Pending flags are checked at the start of every sensor tick and applied immediately.

| CMD | Code | Bytes | Pending flag | Applied by |
|-----|------|-------|--------------|------------|
| CMD_ECG_CFG | 0xCF | 5 | `g_cmd_cfg_pending` | `adc_set_sample_us(g_cmd_sample_us)` in main loop |
| CMD_THR | 0xCE | 31 | — | threshold globals updated immediately, no pending |
| CMD_PPG_CFG | 0xCD | 3-4 | `g_ppg_cfg_pending` | `max30102_set_sampling_rate()` in main loop (LED current is closed-loop adaptive, see max.c) |
| CMD_VITAL_CFG | 0xCC | 3 | `g_vital_cfg_pending` | vital BLE tick counter uses `g_vital_interval_ms / 10` directly |
| CMD_NAME_CFG | 0xC9 | 2-17 | — | `g_patient_name` updated immediately; shown on LCD Row 1 line 2 when connected |

Every successful command above also calls `notify_update(title, val)`, setting
`g_cmd_update_pending` + `g_cmd_update_msg` (title, e.g. "ECG Config") + `g_cmd_update_val`
(the actual new value(s), e.g. "250Hz 200ms", "PPG ECG SpO2 Temp" for CMD_THR — only the
threshold groups that actually changed, "Continuous ECG:On" for CMD_MODE_CFG, the name itself
for CMD_NAME_CFG). The main loop sensor tick checks this flag and, if an LCD is present, shows
a ~1s full-screen bold splash (title + value, `dashboard_show_update_splash()`) before
restoring the normal layout (`dashboard_init_layout()`).

**Wire flow in main.c sensor tick:**
```c
if (g_cmd_cfg_pending)      { g_cmd_cfg_pending = false;  adc_set_sample_us(g_cmd_sample_us); }
if (g_ppg_cfg_pending)      { g_ppg_cfg_pending = false;  max30102_set_sampling_rate(...); }
// LED current is adapted automatically once/sec inside max30102_process() toward PPG_TARGET_ADC
if (g_vital_cfg_pending)    { g_vital_cfg_pending = false; }  /* interval read live below */
// vital BLE send: if (++s_ble_tick >= g_vital_interval_ms / 10U) { send_vitals_packet(); }
```

---

## Main Loop Structure

```
while (1) {
  [sensor tick — every 10 ms]
    ① apply pending config (cmd pending flags)
    ② MAX30102: read 1 sample → max30102_process() → g_sensor.hr_ppg / spo2 (+ s_dash.hr_valid=true → dashboard_update_hr())
    ③ MMA8452Q: read → pedometer_update() → g_sensor.steps / cadence (not shown on LCD anymore)
    ④ LCD: s_dash.ecg_enabled = g_ecg_stream_enabled; s_dash.hr_ecg/hr_ecg_valid from g_sensor; dashboard_update_ecg()
    ⑤ TMP117: one-shot state machine (wake → 200 ms → read → filter) → g_sensor.temp
    ⑥ BLE vitals: send every g_vital_interval_ms (default 1000 ms)
    ⑦ LCD vitals refresh — every g_vital_interval_ms:
         dashboard_update_hr()/dashboard_update_temp() from g_sensor (shows "--" if not *_valid)
         Row 1: s_dash.ble_connected/rssi/mac from ble_app_*(); device_name from g_patient_name
                (CMD_NAME_CFG) if connected else "" → dashboard_update_ble_status()

  [ECG path — every 4 ms, ISR-driven]
    if g_ecg_raw < 1000 → electrode disconnected: hr_ecg_valid=false, skip filter/buffer
    else ecg_process(g_ecg_raw) → filter chain → R-peak → buffer
    → BLE notify when buffer reaches g_cmd_pkt_samples

  [idle]
    WFE when no flags set
}
```

---

## Key Constraints / Gotchas

- **TIMER3** — SAADC PPI only. Never use for PWM, app_timer, or anything else.
- **SAADC** — GAIN1_6 + 0.6 V ref → max 3.6 V. AD8232 bias must stay in [0–3.6 V].
- `g_ecg_raw`, `g_ecg_ready`, all cmd globals are `volatile` (ISR/BLE event ↔ main loop).
- TWI is IRQ-driven (`twi_handler`); use `twi_wait()` after every transfer. Never call TWI drivers concurrently.
- `adc_set_sample_us()` (peripheral.c) updates TIMER3 CC0 live via `nrf_drv_timer_extended_compare` — safe while PPI is running.
- `max30102_set_sampling_rate/led_current_1/2` write I2C registers directly — only call inside sensor tick to avoid bus conflicts.
- `wearable_ecg - Copy/` is a backup snapshot — not the active source tree.

---

## Build Targets

| Toolchain | Path |
|-----------|------|
| Keil MDK | `pca10040/s132/arm5_no_packs/` |
| armgcc | `pca10040/s132/armgcc/Makefile` |

SoftDevice required: **s132 v7.2.0**

```bash
make flash_softdevice   # first time only
make flash
```
