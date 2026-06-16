# Node Device — Setup & Reference Guide

Firmware for the wearable health monitor node (nRF52832 + AD8232 + MAX30102 + MMA8452Q + TMP117 + GC9A01).

---

## Hardware Wiring

| Signal | nRF52832 Pin | Connected To |
|--------|-------------|--------------|
| SAADC (ECG) | AIN0 (P0.02) | AD8232 OUTPUT |
| TWI SCL | P0.29 | MAX30102, MMA8452Q, TMP117 |
| TWI SDA | P0.28 | MAX30102, MMA8452Q, TMP117 |
| SPI CLK | P0.27 | GC9A01 CLK |
| SPI MOSI | P0.26 | GC9A01 MOSI |
| SPI CS (LCD) | P0.25 | GC9A01 CS |
| LCD DC | P0.24 | GC9A01 DC |
| LCD RST | P0.23 | GC9A01 RST |
| MAX30102 INT | P0.11 | MAX30102 INT |

AD8232 SDN pin held LOW (amp always active). OUTPUT must stay within 0–3.6 V.

---

## Build

Requires **nRF5 SDK 17.1.0** and **GNU Arm Embedded Toolchain**.

```bash
cd pca10040/s132/armgcc
make
```

Output: `_build/nrf52832_xxaa.hex`

Softdevice required: **s132 v7.2.0** (`s132_nrf52_7.2.0_softdevice.hex`)

---

## Flash

```bash
# 1. Flash softdevice (first time only)
make flash_softdevice

# 2. Flash application
make flash
```

Or manually:
```bash
nrfjprog -f nrf52 --program _build/nrf52832_xxaa.hex --sectorerase
nrfjprog -f nrf52 --reset
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
| Advertising duration | 180 s (auto-restart on timeout) |
| Connection interval | 8 ms (fixed) |
| MTU | 247 bytes (negotiated) |

### BLE Uplink (node → gateway)

| Packet | Length | Format |
|--------|--------|--------|
| ECG batch | 2N bytes | N × int16 LE (N = g_cmd_pkt_samples, default 50 = 100 bytes) |
| Vitals | 16 bytes | 4 × float32 LE: `[ecgHr, ppgHr, spo2, temp_C]` |

Gateway dispatches by packet length: ≥2 bytes even → ECG batch, 16 bytes → vitals.

---

## Gateway Command Reference (RX Char 0x1402)

All commands are sent by the ESP32 gateway (via nRF52840 central → BLE write) to this node.
The central's `conn_handle` identifies the target — **no node_id byte** is included in any command.

### CMD_ECG_CFG `0xCF` — ECG Sample Rate & Packet Interval (5 bytes)

```
[0xCF][freq_lo][freq_hi][int_lo][int_hi]
```

- `freq`: uint16 LE, ECG sample rate in Hz (default 250)
- `int`: uint16 LE, packet interval in ms (default 200 → 50 samples/packet)
- Applied on next sensor tick: `adc_set_sample_us(1000000 / freq)` reconfigures TIMER3 CC0 live

### CMD_THR `0xCE` — Vital Alert Thresholds (31 bytes)

```
[0xCE]
  [ppgHr norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 1-6  (uint8, bpm)
  [ecgHr norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 7-12 (uint8, bpm)
  [spo2  norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 13-18 (uint8, %)
  [temp  norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 19-30 (uint16 LE, ×10 °C)
```

Temperature encoded ×10 (e.g., 36.1 °C = 361). Thresholds take effect immediately (globals updated inline).

Tier convention matching dashboard: `norm` = green, `warn` = orange, `dang` = red.

### CMD_PPG_CFG `0xCD` — MAX30102 Sensor Config (3-4 bytes)

```
[0xCD][freq_lo][freq_hi][hr_src]
```

- `freq`: uint16 LE, MAX30102 sample rate in Hz (valid: 50 / 100 / 200 / 400 / 800 / 1000 / 1600 / 3200)
- `hr_src`: optional uint8 — 0=IR, 1=RED, selects the LED channel used for HR peak detection and as the
  control input for the adaptive LED current loop (default unchanged if omitted)
- Applied on next sensor tick: `max30102_set_sampling_rate()` via I2C
- LED current is no longer configurable — it's adapted automatically once per second toward
  `PPG_TARGET_ADC` (150000 raw counts) with a ±5000 deadband, in 0.2 mA steps shared by both
  IR and RED LEDs (see `drivers/ppg/max.c`)

### CMD_VITAL_CFG `0xCC` — Vitals BLE Notify Interval (3 bytes)

```
[0xCC][interval_lo][interval_hi]
```

- `interval`: uint16 LE, vitals reporting period in ms (default 1000)
- Takes effect immediately: `g_vital_interval_ms` is read live in the BLE tick counter (`/ 10` to convert to 10 ms ticks)

---

## ECG Signal Chain

```
SAADC 12-bit, AIN0, GAIN1_6, 0.6V ref
  → 250 Hz PPI trigger (TIMER3 → SAADC task, no CPU polling)
  → 4th-order notch @ 50 Hz (2× biquad, Q=10)
  → 4th-order bandpass 1–25 Hz (2× HP + 2× LP biquad, Butterworth)
  → ALE-NLMS (32 taps, delay=15) — adaptive narrowband canceller
  → Savitzky-Golay (window=11) — QRS-preserving smoother
  → R-peak v1: Pan-Tompkins adaptive threshold → BPM
  → R-peak v2: derivative² + envelope + median(8) RR → BPM
  → buffer → BLE notify every g_cmd_pkt_samples (default 50, 100 bytes)
```

---

## Timer Allocation

| Timer | Owner |
|-------|-------|
| TIMER0 | SoftDevice (reserved) |
| TIMER1 | PWM (reserved; `pwm_init()` stub in peripheral.c) |
| TIMER2 | `timer2_now()` µs counter (peripheral.c) |
| TIMER3 | SAADC PPI (peripheral.c) — **do not reassign** |

---

## RTT Debugging

With J-Link RTT Viewer open (TCP 127.0.0.1:19021):

```bash
python rtt_stream.py
```

Streams raw + filtered ECG to terminal, saves `ecg_log.csv`.

---

## Sensor Defaults After Power-On

| Sensor / Parameter | Default |
|--------------------|---------|
| ECG rate | 250 Hz |
| ECG packet | 50 samples / 200 ms |
| PPG rate | 100 Hz |
| PPG LED current | 6 mA red + 6 mA IR |
| Vital interval | 1000 ms |
| Device mode | CONTINUOUS (or last FDS-saved mode) |
