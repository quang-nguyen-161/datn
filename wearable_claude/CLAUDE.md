# Wearable ECG / BLE Project — nRF5 SDK 17.1.0

## Project overview
Embedded firmware for a wearable ECG device on nRF52832 (pca10040) and nRF52840 (pca10056).
Acquires ECG via SAADC at 250 Hz (PPI+TIMER3 driven, no CPU polling), runs a DSP filter chain, and streams results over BLE using a custom GATT service (NUS-style).

## Hardware
- **MCU**: nRF52832 (pca10040, s112/s132 softdevice) or nRF52840 (pca10056, s140)
- **ECG front-end**: AD8232, sampled on SAADC via PPI
- **Display**: GC9A01 round LCD (SPI)
- **IMU/accelerometer**: MMA845x (I2C, TWI instance 1, SCL=P0.29, SDA=P0.28)
- **Heart-rate / SpO2**: MAX sensor (max.c / max.h)
- **Temperature**: TMP117 (I2C)
- **Flash storage**: flash_user.c (FDS-based)

## Timer allocation (do not change without updating all conflicts)
| Timer | Owner |
|-------|-------|
| TIMER0 | SoftDevice (reserved) |
| TIMER1 | PWM (APP_PWM_INSTANCE) |
| TIMER2 | dashboard / timer2_now() |
| TIMER3 | SAADC PPI sampling (250 Hz ECG) |

## Signal chain (250 Hz ECG)
```
raw ADC → ×scale → dc_remove (SG) → notch50 (50 Hz, Q=30) → lpf2_ecg (2× biquad 25 Hz LP) → BLE notify
```
- Filter coefficients: `butter(4, [0.5, 25], 'bandpass', fs=250)` split into HPF + LPF biquad pairs
- BLE payload: 50 samples × int16 = 100 bytes, one notify every 200 ms
- ISR sets `g_ecg_ready`; heavy DSP runs in main loop

## BLE
- Device name: `ECG_dev`
- Custom service UUID: `CUS_SERVICE_UUID` (cus_service.h)
- Advertising interval: 40 ms, duration 180 s
- Connection interval: 20–75 ms
- MTU: default (ATT_MTU_DEFAULT − 3 = 20 bytes min; negotiated via `m_ble_max_data_len`)

## Source files
| File | Purpose |
|------|---------|
| main.c | Top-level init, SAADC PPI, BLE stack, ECG main loop |
| filter.c / filter.h | Biquad DF2T, SG (Savitzky-Golay) DC removal, notch, LP/HP |
| max.c / max.h | MAX heart-rate / SpO2 driver |
| mma845.c / mma845.h | MMA845x accelerometer driver (I2C) |
| GC9A01.c / GC9A01.h | Round LCD driver |
| dashboard.c / dashboard.h | UI rendering, timer2_init/timer2_now |
| cus_service.c / cus_service.h | Custom BLE GATT service |
| flash_user.c / flash_user.h | Persistent storage via FDS |
| tmp117.c / tmp117.h | TMP117 temperature sensor |
| spo2_algorithm.c | SpO2 calculation |
| algorithm_by_RF.c | ECG R-wave / HR algorithm |
| accel_filter.c | Accelerometer signal filtering |
| rtt_stream.py | Host-side tool: reads J-Link RTT via TCP (port 19021), plots raw+filtered ECG in real time, saves ecg_log.csv |

## Build targets
- **Keil MDK (arm5_no_packs)**: `pca10040/s112/arm5_no_packs/` or `pca10040/s132/arm5_no_packs/`
- **GCC (armgcc)**: `pca10040/s112/armgcc/Makefile` — run `make` in that directory
- **IAR / SES**: `.ewp` / `.emProject` files present for both targets
- Pre-built hex files are in `hex/`

## RTT debugging
Run `rtt_stream.py` with J-Link RTT Viewer connected (TCP telnet on 127.0.0.1:19021).
Log format expected: `raw:<int> filt:<int>` per line.

## Key constraints / gotchas
- TIMER3 is used for SAADC PPI — do not reuse it for PWM or other peripherals.
- SAADC gain: GAIN1_6 + 0.6 V internal ref; AD8232 bias must stay in [0–3.6 V].
- `g_ecg_raw`, `g_ecg_filtered`, `g_ecg_ready` are `volatile` (shared ISR ↔ main).
- `wearable_ecg - Copy/` is a sibling backup directory — not the active source tree.
