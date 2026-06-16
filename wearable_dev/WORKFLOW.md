# Device Workflow — nRF52832 Wearable Health Monitor

## Hardware

| Component | Interface | Role |
|-----------|-----------|------|
| nRF52832 (PCA10040) | — | MCU + BLE SoC |
| AD8232 | SAADC AIN0 (P0.02) | ECG front-end |
| MAX30102 | I2C TWI1 | PPG heart rate + SpO2 |
| MMA8452Q | I2C TWI1 | 3-axis accelerometer |
| TMP117 | I2C TWI1 | Precision temperature (addr 0x48) |
| GC9A01 | SPI0 | 240×240 round LCD (always on) |

All I2C sensors are optional — if a device does not respond at boot the firmware logs a warning and skips it in every subsequent tick without crashing.

---

## Boot Sequence

```
power on
    ├─ log_init()
    ├─ timer_init()
    ├─ twi_init()               I2C bus, IRQ-driven, 400 kHz  (TWI1)
    ├─ power_management_init()
    ├─ ble_stack_init()         SoftDevice enable
    ├─ gap_params_init()
    ├─ gatt_init()              MTU 247 bytes
    ├─ services_init()          custom GATT service (0x1401 TX / 0x1402 RX)
    ├─ advertising_init()
    ├─ conn_params_init()
    │
    ├─ timer2_init()            TIMER2 free-running 1 MHz counter (µs)
    ├─ ecg_init()               filter states + TIMER3→PPI→SAADC at 250 Hz
    │
    ├─ max30102_init()  ──► s_ppg_present
    ├─ MMA8452Q_init()  ──► s_accel_present
    ├─ tmp117_Init()    ──► s_tmp_present
    ├─ lcd_spi_init() + GC9A01_init() ──► s_lcd_present
    │       dashboard_splash() → dashboard_init_layout()
    │
    ├─ mma8452q_alert_init()    only if s_accel_present
    ├─ pedometer_reset()
    ├─ temp_filter_reset()
    ├─ device_mode_init()       read saved mode from FDS, start sensor timer
    └─ advertising_start()
```

---

## Main Loop

```
while (1)
    ├─[A] Sensor tick  — set by app_timer (10 ms CONTINUOUS / g_period_ms PERIODIC)
    ├─[B] ECG ready    — set by SAADC ISR at 250 Hz
    └─[C] Idle         — WFE when neither flag is set
```

---

### [A] Sensor Tick (10 ms)

Runs in order every tick. Each peripheral is guarded by its presence flag — absent hardware is silently skipped.

```
sensor tick
    │
    ├─[0] Apply pending gateway config commands
    │       if g_cmd_cfg_pending:   adc_set_sample_us(g_cmd_sample_us)
    │       if g_ppg_cfg_pending:   max30102_set_sampling_rate()
    │                               max30102_set_led_current_1/2()
    │       if g_vital_cfg_pending: (clear flag — interval read live below)
    │
    ├─[1] MAX30102 PPG  (if s_ppg_present)
    │       max30102_read_1_sample() — reads one FIFO entry
    │       max30102_process(ir, red, &hr, &spo2):
    │         HP 0.5 Hz + LP 5.0 Hz (IIR1 DF2T)
    │         ALE-NLMS (32 taps) + Savitzky-Golay (window=11)
    │         peak detection → HR (30–220 BPM, 3-sample history)
    │         peak-valley ratio → SpO2 (70–100 %)
    │       → g_sensor.hr_ppg, g_sensor.spo2
    │
    ├─[2] Accelerometer  (if s_accel_present)
    │       MMA8452Q_read()  →  g_accel.ax/ay/az/magnitude/ac
    │       pedometer_update(&g_pedometer, g_accel.ac, now_ms)
    │         dynamic threshold + 3-step validation + cadence EMA
    │       → g_sensor.steps, g_sensor.cadence
    │
    ├─[3] LCD dashboard (always, if s_lcd_present)
    │       dashboard_update_steps()
    │       dashboard_update_ecg()
    │
    ├─[4] TMP117 one-shot  (if s_tmp_present)
    │       tick N:   tmp117_wake_oneshot()    — 200 ms conversion starts
    │       tick N+?: (now_ms - trigger) >= 200 ms
    │                 tmp117_get_Temp()
    │                 tmp117_shutdown_mode()
    │                 temp_filter_update()     — median(3) + rate-limit + EMA(α=0.3)
    │                 → g_sensor.temp, g_sensor.temp_valid
    │
    └─[5] BLE vitals  (if connected + MTU negotiated)
            every g_vital_interval_ms (counted as g_vital_interval_ms / 10 ticks):
            send_vitals_packet()  →  4 × float32 LE = 16 bytes: [hr_ecg][hr_ppg][spo2][temp]
```

---

### [B] ECG Pipeline (250 Hz, ISR-driven)

TIMER3 CC0 → PPI → SAADC fires every 4 ms (250 Hz). ISR sets `g_ecg_ready` + `g_ecg_raw`. Main loop:

```
g_ecg_raw  (int16, raw SAADC)
    └─ ecg_process():
            4th-order notch 50 Hz   (2× biquad Q=10)
            4th-order bandpass 1–25 Hz (2× HP + 2× LP biquad, Butterworth)
            ALE-NLMS  (32 taps, delay=15)
            Savitzky-Golay (window=11)
            R-peak v1: Pan-Tompkins → g_sensor.hr_ecg
            R-peak v2: derivative² + envelope + median(8) RR → g_sensor.hr_ecg
        ▼
    g_ecg.filtered  (float)
        ├─ scale 0–999  →  s_ecg_display  →  LCD ECG sweep
        └─ cast int16   →  s_ecg_buf[]
                └─ when s_ecg_idx >= g_cmd_pkt_samples (default 50):
                       ble_app_send() — 100 bytes per notify
                       chunks > ECG_MAX_SAMPLES split into consecutive notifies
```

---

## BLE Protocol

### Advertising
- Device name: `ECG_dev`
- Interval: 40 ms, duration: 180 s (auto-restart on idle)

### Custom Service UUIDs
| Characteristic | UUID suffix | Direction | Use |
|----------------|-------------|-----------|-----|
| TX (notify) | `...1401...` | node → gateway | ECG batches + vitals |
| RX (write)  | `...1402...` | gateway → node | Config commands |

### TX Packets (node → gateway)

| Length | Content | Gateway action |
|--------|---------|----------------|
| 2N bytes (N ≤ 100) | N × int16 LE ECG samples | publish `ecg_batch` |
| 16 bytes | 4 × float32 LE vitals | publish `ecgHeartRate / ppgHeartRate / spo2 / temperature` |

### RX Commands (gateway → node)

| CMD | Bytes | Payload | Effect |
|-----|-------|---------|--------|
| `0xCF` CMD_ECG_CFG | 5 | `[freq_lo][freq_hi][interval_lo][interval_hi]` | reconfigures TIMER3 rate + packet size |
| `0xCE` CMD_THR | 31 | 18 × uint8 thresholds + 6 × uint16 temp×10 | updates alert threshold globals immediately |
| `0xCD` CMD_PPG_CFG | 3-4 | `[freqLo][freqHi][hrSrc]` | reconfigures MAX30102 sample rate + HR-source channel (LED current is adaptive) |
| `0xCC` CMD_VITAL_CFG | 3 | `[intervalLo][intervalHi]` | updates BLE vitals notify interval |

---

## Data Flow to Cloud

```
nRF52832 node
    │  BLE notify TX 0x1401
    │  ECG batch: N × int16 LE  (N = g_cmd_pkt_samples, default 50 samples)
    │  Vitals:    4 × float32 LE every g_vital_interval_ms (default 1 s)
    ▼
Gateway  (ESP32 + nRF52840 central)
    │  MQTT  →  ThingsBoard  103.116.39.179
    ▼
ThingsBoard
    │  WebSocket / REST
    ▼
VitalSync dashboard (Next.js)
    Keys: ecg_batch · ecgHeartRate · ppgHeartRate · spo2 · temperature
```

---

## LCD Dashboard Layout

```
┌────────────────────────────────┐
│  ECG_dev  -72dBm  ███░  [BLE] │  Row 1: BLE status + RSSI + signal bars
├──────────────┬─────────────────┤
│  Temp: 36.8°C│  SpO2: 98%     │  Row 2: Temperature (left) + SpO2 (right)
├──────────────┴─────────────────┤
│  ECG waveform      │  HR: 87  │  Row 3: ECG sweep (left 55%) + HR + sweep (right 45%)
├────────────────────────────────┤
│  Steps: 1234   Cadence: 92    │  Row 4: Pedometer + activity
└────────────────────────────────┘
```

- ECG waveform: updates every sensor tick from `s_ecg_display` (0–999 scaled)
- HR + SpO2: update when `max30102_process()` returns a valid result
- Temperature: updates after each TMP117 one-shot cycle (~200 ms + filter)
- Steps + cadence: update every tick from accelerometer
