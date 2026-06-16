# Wearable Health Monitor — nRF52832 Node Firmware

Embedded firmware for the wearable health-monitor node running on nRF52832 (PCA10040, s132).
Acquires ECG via SAADC at 250 Hz (PPI+TIMER3, zero CPU polling), reads PPG/SpO2 (MAX30102),
temperature (TMP117), and acceleration (MMA8452Q), then streams data over BLE to an ESP32 gateway.

---

## Hardware

| Component | Interface | Pin(s) |
|-----------|-----------|--------|
| nRF52832 (PCA10040) | — | MCU + BLE SoC |
| AD8232 ECG front-end | SAADC AIN0 | P0.02 |
| MAX30102 PPG/SpO2 | I2C TWI1 | SCL P0.29 / SDA P0.28 |
| MMA8452Q accelerometer | I2C TWI1 | shared bus |
| TMP117 temperature | I2C TWI1 | shared bus (addr 0x48) |
| GC9A01 round LCD 240×240 | SPI0 | CLK P0.27 / MOSI P0.26 / CS P0.25 / DC P0.24 / RST P0.23 |
| MAX30102 interrupt | GPIO | P0.11 |

---

## Directory Structure

```
wearable_dev/
├── main.c / main.h          — top-level init, main loop; main.h holds the board pin map
├── peripheral/
│   └── peripheral.c/h       — all on-chip peripherals: TWI1, SPI0, SAADC+PPI+TIMER3, TIMER2, PWM
├── ecg/
│   ├── ecg.c                — DSP pipeline + R-peak detection (calls adc_init() in peripheral.c)
│   └── ecg.h                — ecg_init(), ecg_process()
├── cmd/
│   ├── cmd.c                — gateway command parser (CMD_ECG_CFG / THR / PPG_CFG / VITAL_CFG)
│   └── cmd.h                — command codes, config globals (g_cmd_*, g_ppg_*, g_vital_*)
├── ble/
│   └── cus_service.c/h      — custom GATT service (TX notify 0x1401 / RX write 0x1402)
├── app/
│   └── device_mode.c/h      — operating mode FSM (CONTINUOUS / PERIODIC / ECG), FDS persistence
├── dsp/
│   └── filter.c/h           — biquad DF2T, NLMS, ALE, Savitzky-Golay, runtime coeff calculators
├── drivers/
│   ├── ppg/
│   │   ├── max.c/h          — MAX30102 driver (I2C init, FIFO read, streaming HR/SpO2)
│   │   ├── max30102.h       — register map, enums, public API
│   │   ├── spo2_algorithm.c — SpO2 peak-valley ratio computation
│   │   └── algorithm_by_RF.c — HR peak detection (ring-buffer variant)
│   ├── accel/
│   │   ├── mma845.c/h       — MMA8452Q I2C driver
│   │   ├── pedometer.c/h    — dynamic-threshold step counter + cadence EMA
│   │   └── accel_filter.c/h — accelerometer signal filtering
│   ├── display/
│   │   ├── GC9A01.c/h       — GC9A01 LCD driver (SPI0 bus via spi_init(); LCD GPIO + draw ops)
│   │   ├── dashboard.c/h    — 4-row UI layout, update functions
│   │   └── lcd_power.c/h    — LCD backlight control
│   └── temp/
│       ├── tmp117_v2.c/h    — TMP117 I2C driver (one-shot mode)
│       └── temp_filter.c/h  — median(3) + rate-limit + EMA(α=0.3) temperature filter
└── storage/
    └── flash_user.c/h       — FDS-based persistent config storage
```

---

## Timer Allocation

| Timer | Owner | Notes |
|-------|-------|-------|
| TIMER0 | SoftDevice | reserved — do not touch |
| TIMER1 | PWM | reserved; `pwm_init()` stub in peripheral.c |
| TIMER2 | µs counter | `timer2_init()` / `timer2_now()` in peripheral.c |
| TIMER3 | SAADC PPI | 250 Hz ECG (peripheral.c) — **do not reassign** |

---

## ECG Signal Chain (250 Hz)

```
SAADC AIN0 (12-bit, GAIN1_6, 0.6 V ref)
  ← TIMER3 CC0 → PPI → SAADC sample task (4 ms period = 250 Hz, no CPU)
  → saadc_callback() sets g_ecg_raw + g_ecg_ready
  → ecg_process() in main loop:
      4th-order notch   50 Hz  (2× biquad Q=10)
      4th-order bandpass 1–25 Hz (2× HP + 2× LP biquad, Butterworth)
      ALE-NLMS          32 taps, delay=15  (adaptive narrowband canceller)
      Savitzky-Golay    window=11           (QRS-preserving smoother)
      R-peak v1         Pan-Tompkins adaptive threshold
      R-peak v2         derivative² + envelope + median(8) RR
  → g_sensor.hr_ecg (BPM)
  → s_ecg_buf[]  → BLE notify 2×int16 per packet (ECG_PACKET_SAMPLES, max ECG_MAX_SAMPLES)
```

---

## PPG / SpO2 Signal Chain (100 Hz default)

```
MAX30102 FIFO (IR + RED, 18-bit)
  → max30102_read_1_sample() per sensor tick
  → max30102_process():
      HP 0.5 Hz (IIR1 DF2T) + LP 5.0 Hz (IIR1 DF2T)
      ALE-NLMS 32 taps + Savitzky-Golay window=11
      peak detection → HR (30–220 BPM, 3-sample history EMA)
      peak-valley SpO2 ratio (70–100 % valid)
  → g_sensor.hr_ppg, g_sensor.spo2
```

---

## BLE Protocol

| Parameter | Value |
|-----------|-------|
| Device name | `ECG_dev` |
| Service UUID | `6e401400-b5a3-f393-e0a9-e50e24dcca9e` |
| TX Char (notify) | `0x1401` — node → gateway |
| RX Char (write) | `0x1402` — gateway → node |
| Advertising interval | 40 ms |
| Advertising duration | 180 s (auto-restart on idle) |
| Connection interval | 8 ms (fixed) |
| MTU | 247 bytes (negotiated via `NRF_SDH_BLE_GATT_MAX_MTU_SIZE`) |

### Uplink packets (node → gateway)

| Length | Content |
|--------|---------|
| 2N bytes | N × int16 LE ECG samples (N = g_cmd_pkt_samples) |
| 16 bytes | 4 × float32 LE: `[hr_ecg][hr_ppg][spo2][temp_C]` |

### Downlink commands (gateway → node via RX Char 0x1402)

| CMD | Bytes | Payload | Effect |
|-----|-------|---------|--------|
| `0xCF` CMD_ECG_CFG | 5 | `[freq_lo][freq_hi][int_lo][int_hi]` | reconfigures TIMER3 rate + packet size |
| `0xCE` CMD_THR | 31 | PPG/ECG/SpO2 thresholds (uint8) + temp thresholds (uint16 ×10 °C) | updates alert globals immediately |
| `0xCD` CMD_PPG_CFG | 3-4 | `[freqLo][freqHi][hrSrc]` | reconfigures MAX30102 sample rate + HR-source channel (LED current is adaptive) |
| `0xCC` CMD_VITAL_CFG | 3 | `[intervalLo][intervalHi]` | sets BLE vitals notify interval (ms) |

---

## Key Constraints

- **TIMER3** is exclusively owned by `peripheral.c` for SAADC PPI — never reassign it.
- **SAADC gain**: GAIN1_6 + 0.6 V internal ref → full scale ≈ 3.6 V. AD8232 output must stay in [0–3.6 V].
- `g_ecg_raw`, `g_ecg_ready` are `volatile` (ISR ↔ main loop).
- All cmd config globals (`g_cmd_*`, `g_ppg_*`, `g_vital_*`) are `volatile` (BLE RX event ↔ main loop).
- Sensor presence flags (`s_ppg_present`, `s_accel_present`, `s_tmp_present`) are set at boot; absent hardware is silently skipped every tick.
- `wearable_ecg - Copy/` is a backup snapshot — not the active source tree.

---

## Build

Requires **nRF5 SDK 17.1.0** + **GNU Arm Embedded Toolchain** + SoftDevice **s132 v7.2.0**.

```bash
# Keil MDK
pca10040/s132/arm5_no_packs/

# armgcc
cd pca10040/s132/armgcc && make
```

Flash:
```bash
make flash_softdevice   # first time only
make flash
```

---

## RTT Debugging

With J-Link RTT Viewer open on TCP 127.0.0.1:19021:

```bash
python rtt_stream.py
```

Streams raw + filtered ECG and saves `ecg_log.csv`.
