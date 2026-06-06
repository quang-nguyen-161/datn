# Node Device — Setup & Reference Guide

Firmware for the wearable ECG node (nRF52832 + AD8232 + MAX30102 + MMA8452Q + TMP117 + GC9A01).

---

## Hardware Wiring

| Signal | nRF52832 Pin | Connected To |
|--------|-------------|--------------|
| SAADC (ECG) | AIN0 (P0.02) | AD8232 OUTPUT |
| TWI SCL | P0.29 | MMA8452Q, TMP117 |
| TWI SDA | P0.28 | MMA8452Q, TMP117 |
| SPI CLK | P0.27 | GC9A01 CLK |
| SPI MOSI | P0.26 | GC9A01 MOSI |
| SPI CS (LCD) | P0.25 | GC9A01 CS |
| LCD DC | P0.24 | GC9A01 DC |
| LCD RST | P0.23 | GC9A01 RST |
| MAX30102 INT | P0.11 | MAX30102 INT |
| I2C (MAX30102) | shared TWI1 | MAX30102 SDA/SCL |

AD8232 bias: SDN (shutdown) held LOW to keep amp active. Ensure OUTPUT stays within 0–3.6 V (AD8232 at 3.3 V supply with Vref = 1.5 V is safe).

---

## Build

Requires **nRF5 SDK 17.1.0** and **GNU Arm Embedded Toolchain**.

```bash
cd pca10040/s132/armgcc
make
```

Output: `_build/nrf52832_xxaa.hex`

To clean: `make clean`

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
| Connection interval | 20–50 ms (ECG mode), 100–200 ms (vitals) |
| MTU | 247 bytes (negotiated) |

### BLE Uplink (node → nRF52 central)

| Packet | Length | Format |
|--------|--------|--------|
| ECG batch | 100 bytes | 50 × int16 LE (raw filtered samples at 250 Hz) |
| Vitals | 16 bytes | 4 × float32 LE: `[ecgHr, ppgHr, spo2, temp_C]` |

The nRF52832 central dispatches by packet length: 100 B → UART TYPE 0x01, 16 B → UART TYPE 0x02.

---

## Gateway Command Reference (RX Char 0x1402)

All commands are sent by the nRF52832 central (which receives them from the ESP32 over UART).

### CMD_ACK `0xA0` — Node Identity Assignment (6 bytes)

```
[0xA0][addr_b2][addr_b3][addr_b4][addr_b5][node_id]
```

- `addr_b2..b5`: own MAC bytes `addr[3..0]` (little-endian, bytes 2–5 of the 6-byte MAC)
- `node_id`: assigned index (0–N), stored in `g_node_id`
- Node validates the MAC before accepting; ignores if mismatch

### CMD_ECG_CFG `0xCF` — ECG Sample Rate & Packet Interval (6 bytes)

```
[0xCF][node_id][freq_lo][freq_hi][int_lo][int_hi]
```

- `freq`: uint16 LE, ECG sample rate in Hz (default 250)
- `int`: uint16 LE, packet interval in ms (default 200 → 50 samples/pkt)
- Sets `g_cmd_cfg_pending = true`; main loop calls `ecg_config_apply()` on next iteration

### CMD_THR `0xCE` — Vital Thresholds (32 bytes)

```
[0xCE][node_id]
  [ppgHr norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 2-7  (uint8, bpm)
  [ecgHr norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 8-13 (uint8, bpm)
  [spo2  norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 14-19 (uint8, %)
  [temp  norm_min][norm_max][warn_min][warn_max][dang_min][dang_max]   bytes 20-31 (uint16 LE ×10 °C)
```

Temperature is encoded ×10 (e.g., 36.1 °C = 361).

### CMD_PPG_CFG `0xCD` — PPG Sensor Config (6 bytes)

```
[0xCD][node_id][freq_lo][freq_hi][red_ma][ir_ma]
```

- `freq`: uint16 LE, MAX30102 sample rate in Hz
- `red_ma`, `ir_ma`: LED current in mA (uint8)
- Sets `g_ppg_cfg_pending = true`; main loop reconfigures MAX30102

### CMD_VITAL_CFG `0xCC` — Vital Reporting Interval (4 bytes)

```
[0xCC][node_id][interval_lo][interval_hi]
```

- `interval`: uint16 LE, vitals reporting period in ms (default 1000)

---

## ECG Signal Chain

```
SAADC 12-bit, AIN0, GAIN1_6, 0.6V ref, 4× oversample
  → 250 Hz PPI trigger (TIMER3 → SAADC task, no CPU involvement)
  → DC removal: sliding-mean window 125 samples (~0.5 s)
  → 50 Hz notch: biquad DF2T (Q≈30)
  → 25 Hz LPF: 2× cascaded biquad (Butterworth 2nd order)
  → Pan-Tompkins R-peak → ECG heart rate (g_sensor.hr_ecg)
  → buffer 50 samples → BLE notify (100 bytes, every 200 ms)
```

**Filter parameters:**
- NLMS adaptive filter: 32 taps, ALE_DELAY = 15 samples
- Notch Q = 30, centre 50 Hz, Fs = 250 Hz

---

## Timer Allocation

| Timer | Owner |
|-------|-------|
| TIMER0 | SoftDevice (reserved) |
| TIMER1 | PWM |
| TIMER2 | `timer2_now()` µs counter (dashboard) |
| TIMER3 | SAADC PPI — **do not reassign** |

---

## RTT Debugging

With J-Link RTT Viewer open (TCP 127.0.0.1:19021):

```bash
python rtt_stream.py
```

Streams raw + filtered ECG to terminal and saves `ecg_log.csv`.

---

## Sensor Defaults After Power-On

| Sensor | Default |
|--------|---------|
| ECG rate | 250 Hz |
| ECG packet | 50 samples / 200 ms |
| PPG rate | 100 Hz |
| PPG LED current | 6 mA red + 6 mA IR |
| Vital interval | 1000 ms |
| Node ID | 0xFF (unassigned — waits for CMD_ACK) |
| Device mode | CONTINUOUS (or last FDS-saved mode) |
