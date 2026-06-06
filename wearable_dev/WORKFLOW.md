# Device Workflow — nRF52832 Wearable Health Monitor

## Hardware

| Component | Interface | Role |
|-----------|-----------|------|
| nRF52832 (PCA10040) | — | MCU + BLE SoC |
| AD8232 | SAADC (P0.04) | ECG front-end |
| MAX30102 | I2C / TWI1 | PPG heart rate + SpO2 |
| MMA8452Q | I2C / TWI1 | 3-axis accelerometer |
| TMP117 | I2C / TWI1 | Precision temperature |
| GC9A01 | SPI0 | 240×240 round LCD (always on) |

All I2C sensors are optional — if a device does not respond at boot the firmware logs a warning and skips that sensor in every subsequent tick without crashing.

---

## Boot Sequence

```
power on
    │
    ├─ log_init()
    ├─ timer_init()
    ├─ power_management_init()
    ├─ twi_init()               I2C bus — blocking/polling mode, 400 kHz
    │                           (NULL event handler: NACK returns error, never hangs)
    │
    ├─ m_record_init()          FDS flash — load saved config
    ├─ ble_app_init()           SoftDevice + GATT + custom service
    │
    ├─ max30102_init()  ──► s_ppg_present   (false if NACK)
    ├─ MMA8452Q_init()  ──► s_accel_present (false if NACK)
    ├─ tmp117_Init()    ──► s_tmp_present   (false if NACK)
    │
    ├─ mma8452q_alert_init()    only if s_accel_present
    │
    ├─ timer2_init()            TIMER2 free-running 1 MHz counter
    ├─ ecg_init()               TIMER3 → PPI → SAADC at 250 Hz
    │
    ├─ pedometer_reset()
    ├─ temp_filter_reset()
    ├─ dashboard_splash / layout init (GC9A01 always on)
    │
    ├─ device_mode_init()       read saved mode from FDS, start sensor timer
    └─ ble_app_advertising_start()
```

---

## Main Loop

```
while(1)
    │
    ├─[A] ECG ready flag    — set by SAADC ISR at 250 Hz
    ├─[B] Sensor tick       — set by app_timer (10 ms CONTINUOUS / g_period_ms PERIODIC)
    └─[C] Idle              — WFE when neither flag is set
```

---

### [A] ECG Pipeline (250 Hz, ISR-driven)

TIMER3 → PPI → SAADC fires every 4 ms. ISR sets `g_ecg_ready` + `g_ecg_raw`. Main loop processes each sample:

```
g_ecg_raw  (int16, raw SAADC)
    │
    └─ ecg_process()
            │  DC removal  (Savitzky-Golay)
            │  Notch 50 Hz (Q = 30)
            │  Bandpass 1–25 Hz (2× biquad HP + 2× biquad LP)
            │  ALE-NLMS adaptive cancellation (32 taps, delay = 15)
            │  Savitzky-Golay smoothing (window = 11)
            ▼
        g_ecg.filtered  (float)
            │
            ├─ scale to 0–999  →  s_ecg_display  →  LCD waveform (dashboard_update_ecg)
            │
            └─ cast int16  →  s_ecg_buf[]
                    │
                    └─ when s_ecg_idx >= g_cmd_pkt_samples (default 50 samples):
                           ble_app_send() — 100 bytes per notify
                           chunks of ≤ ECG_MAX_SAMPLES split into consecutive notifies
```

---

### [B] Sensor Tick (10 ms / g_period_ms)

Runs in order every tick. Each peripheral is guarded by its presence flag — absent hardware is silently skipped.

```
sensor tick
    │
    ├─[1] MAX30102 PPG  (if s_ppg_present)
    │       max30102_read_samples() — reads FIFO, returns 0 on I2C error
    │       append to s_ir_buf[] / s_red_buf[]
    │       when s_ppg_count >= 100 samples (~1 s at 100 Hz):
    │           max30102_compute()
    │               peak detection on IR channel
    │               HR from peak intervals  (30–220 bpm valid range)
    │               SpO2 from peak-valley R-ratio  (70–100% valid range)
    │               3-sample HR history smoothing
    │           → g_sensor.hr_ppg, g_sensor.spo2
    │           → dashboard_update_hr()
    │           reset accumulator
    │
    ├─[2] Accelerometer  (if s_accel_present)
    │       MMA8452Q_read()  →  g_accel.ax/ay/az/magnitude/ac
    │       pedometer_update(&g_pedometer, g_accel.ac, now_ms)
    │           dynamic threshold + 3-step validation + cadence EMA
    │       → g_sensor.steps, g_sensor.cadence
    │       → s_dash.steps, s_dash.cadence
    │
    ├─[3] LCD dashboard (always, LCD always on)
    │       dashboard_update_steps(&s_dash, g_accel.ac)
    │       dashboard_update_ecg(&s_dash, s_ecg_display)
    │
    ├─[4] TMP117 one-shot  (if s_tmp_present)
    │       tick N:   tmp117_wake_oneshot()   — sensor starts 200 ms conversion
    │       tick N+?: (now_ms - trigger) >= 200 ms
    │                 tmp117_get_Temp()       — returns TMP117_TEMP_INVALID on error
    │                 tmp117_shutdown_mode()
    │                 temp_filter_update()    — median(3) + rate-limit + EMA(α=0.3)
    │                 → g_sensor.temp, g_sensor.temp_valid
    │                 → dashboard_update_temp()
    │
    └─[5] BLE vitals  (if connected)
            send_vitals_packet()
            4 × float32 LE = 16 bytes: [hr_ecg][hr_ppg][spo2][temp]
```

---

## BLE Protocol

### Advertising
- Device name: `ECG_dev`
- Interval: 40 ms, duration: 180 s

### Custom Service UUIDs
| Characteristic | UUID suffix | Direction | Use |
|----------------|-------------|-----------|-----|
| TX (notify) | `...1401...` | node → gateway | ECG batches + vitals |
| RX (write)  | `...1402...` | gateway → node | Config commands |

### TX Packets (node → gateway)

| Length | Content | Gateway action |
|--------|---------|----------------|
| 2N bytes, N ≤ 100 | N × int16 LE ECG samples | publish `ecg_batch` |
| 16 bytes | 4 × float32 LE vitals | publish `ecgHeartRate / ppgHeartRate / spo2 / temperature` |

### RX Commands (gateway → node)

| CMD | Bytes | Payload |
|-----|-------|---------|
| `0xCF` CMD_ECG_CFG | 5 | `[freq_lo][freq_hi][interval_lo][interval_hi]` |
| `0xCE` CMD_THR | 31 | 18 × uint8 thresholds + 6 × uint16 temp×10 |
| `0xCD` CMD_PPG_CFG | 5 | `[freqLo][freqHi][redMa][irMa]` |
| `0xCC` CMD_VITAL_CFG | 3 | `[intervalLo][intervalHi]` |

---

## Data Flow to Cloud

```
nRF52832
    │  BLE notify: ECG (100 B) / vitals (16 B)
    ▼
Gateway  (gateway.py  OR  ESP32 + nRF52840 central)
    │  MQTT  v1/gateway/telemetry
    ▼
ThingsBoard  (103.116.39.179)
    │  WebSocket / REST
    ▼
VitalSync dashboard  (Next.js)
    Keys: ecg_batch · ecgHeartRate · ppgHeartRate · spo2 · temperature
```

---

## LCD Dashboard Layout (always on)

```
┌────────────────────────────┐
│  BLE: ECG_dev  -72dBm ███░ │  Row 1: BLE status + RSSI
├──────────────┬─────────────┤
│  Temp: 36.8°C│  SpO2: 98%  │  Row 2: Temperature + SpO2
├──────────────┴─────────────┤
│  ECG waveform  │  HR: 87   │  Row 3: ECG sweep + HR number
├────────────────────────────┤
│  Steps: 1234   Cadence: 92 │  Row 4: Pedometer
└────────────────────────────┘
```

- ECG waveform updates every sensor tick from `s_ecg_display` (0–999 scaled)
- HR + SpO2 update when 100-sample PPG buffer is computed (~every 1 s)
- Temperature updates after each TMP117 one-shot cycle
- Steps + cadence update every tick from accelerometer
