# Sensor Algorithms Reference

This document describes the signal-processing and algorithm pipelines used by
each sensor driver in `wearable_dev`: PPG (HR/SpO2), ECG, temperature (TMP117),
and accelerometer (motion/pedometer).

---

## 1. PPG — Heart Rate & SpO2 (`drivers/ppg/max.c`)

### 1.1 Sensor configuration (`max30102_init`)

The MAX30102 is configured via the register-setter helpers to match a known-good
reference configuration:

| Setting | Value |
|---|---|
| FIFO config | `SMP_AVE = 1` (no averaging), rollover off, `A_FULL = 17` |
| Mode | SpO2 mode (`0x03`) — both RED and IR LEDs active |
| ADC range | 2048 nA full-scale (`max30102_adc_2048`) |
| Sample rate | 100 Hz (`max30102_sr_100`) |
| LED pulse width | 18-bit (`max30102_pw_18_bit`) |
| LED current 1 (IR) | 4 mA |
| LED current 2 (RED) | 6 mA |

`max30102_set_sampling_rate()` writes the `SR` bits (4:2) of `SPO2_CONFIG`
without disturbing `ADC_RGE`/`LED_PW`:
```c
config = (config & ~(0x07 << SPO2_SR_SHIFT)) | (sr << SPO2_SR_SHIFT);
```

After any LED-current change, `max30102_reset_filters()` should be called so the
DC/AC estimators re-converge instead of producing a transient SpO2 artifact.

### 1.2 Streaming pipeline (`max30102_process`)

Runs once per sample at 100 Hz:

1. **Finger detection** — raw IR is compared against `PPG_FINGER_THRESHOLD = 30000`.
   A `PPG_FINGER_DEBOUNCE = 5`-sample debounce avoids flicker when the finger is
   placed/removed. While "finger off", filters/peak state are held/reset.
2. **DC removal** — `dc_remove()` subtracts a sliding-window mean
   (`dc_filter_t`, window = `DC_WINDOW`) from each raw sample, separately for IR
   and RED.
3. **Bandpass filtering for HR** — the DC-removed signal passes through two
   cascaded biquads per channel:
   - High-pass at 1 Hz (`butter2_hp_coeffs(100, 1.0, ...)`) → `s_ir_hp` / `s_red_hp`
   - Low-pass at 3 Hz (`butter2_lp_coeffs(100, 3.0, ...)`) → `s_ir_lp` / `s_red_lp`

   This isolates the cardiac (~0.5–3 Hz) component for peak detection.
4. **Peak detection** — `find_peak_thresh()` does 4-neighbor local-maximum
   detection on the filtered IR signal, gated by an adaptive envelope:
   - `PPG_PEAK_ENV_DECAY = 0.992` — envelope decays slowly between beats.
   - `PPG_PEAK_THRESH_RATIO = 0.4` — a sample must exceed 40% of the running
     envelope peak to be accepted.
   - 350 ms refractory period (35 samples @ 100 Hz) prevents double-counting.
5. **Raw ring buffers** — `ir_raw_rb` / `red_raw_rb` (size `RB_SIZE = 100`,
   i.e. 1 second of raw, un-filtered samples) are continuously updated for the
   SpO2 calculation (see §1.4).
6. **Output** — every `PPG_UPDATE_PERIOD = 100` samples (~once/sec),
   `hr_count()` and `spo2_count()` are evaluated, smoothed (§1.5), and emitted as
   `PPG: hr=%d spo2=%d peaks=%d dc_ir=%d`.

### 1.3 Heart-rate computation (`hr_count`)

Maintains a ring buffer of inter-beat intervals (`dt_rb`, µs) and matching peak
weights (`w_rb`, from the envelope at the time of detection). HR is the
**envelope-weighted average RR interval**, converted to BPM:

- Valid interval range: `MIN_DT = 300000 µs` (200 BPM) .. `MAX_DT = 2000000 µs` (30 BPM).
- Intervals outside this range, or weights below `MIN_WEIGHT = 1.0`, are excluded
  from the average.
- `bpm = 60 * 1e6 / weighted_mean(dt)`.

### 1.4 SpO2 computation (`spo2_count`) — Robert Fraczkiewicz (RF) algorithm

Operates **only on the raw (un-filtered) ring buffers** `ir_raw_rb` /
`red_raw_rb` (100 samples = 1 s window), accessed in chronological order via:
```c
#define RB_CHRONO(rb, i) ((rb)->buff[((rb)->rb_head + (i)) % RB_SIZE])
```

Steps:

1. **DC** — arithmetic mean of each channel's raw samples over the window:
   `ir_dc = mean(ir_raw)`, `red_dc = mean(red_raw)`.
2. **Detrend** — subtract the DC, then remove a **linear trend** (regression of
   the de-meaned signal against sample index `i`, centered at
   `mean_x = (RB_SIZE-1)/2`):
   ```
   beta = Σ(x_i * d_i) / Σ(x_i²)         where x_i = i - mean_x
   d_i  -= beta * x_i
   ```
   This removes baseline drift/wander so the AC estimate isn't biased by slow
   trends within the 1 s window.
3. **AC** — RMS of the detrended signal:
   `ac = sqrt( Σ(d_i²) / RB_SIZE )` for each channel.
4. **Ratio-of-ratios**:
   ```
   R = (red_ac * ir_dc) / (ir_ac * red_dc)
   ```
5. **R → slope mapping** (piecewise, ported from the legacy `spo2_count1`):

   | R range | slope formula | slope range |
   |---|---|---|
   | `0.8 ≤ R ≤ 2.5` | `(R-0.8)/(2.5-0.8) * (0.294118-0.241176) + 0.241176` | 0.241–0.294 |
   | `0 ≤ R < 0.8` | `R/0.8 * (0.235294-0.182353) + 0.182353` | 0.182–0.235 |
   | `2.5 < R ≤ 10` | `(R-2.5)/(10-2.5) * (0.470588-0.3) + 0.3` | 0.300–0.471 |
   | `R > 10` | `0.529` (fixed) | 0.529 |
   | else (R < 0) | `0.24` (fallback) | 0.24 |

6. **Slope → SpO2** (empirically calibrated, see §1.6):
   ```
   spo2 = 114.6 - 55.85 * slope
   ```
   clamped to `[85, 100]`.

If any of `ir_ac`, `red_ac`, `ir_dc`, `red_dc` is `≤ 0`, the function returns 0
(invalid reading — typically no finger or sensor saturation).

### 1.5 Output smoothing

`hr_count()` and `spo2_count()` outputs each pass through an **independent
3-sample median filter** (`s_hr_hist` / `s_spo2_hist`). If a filter's window
isn't yet full/valid, the last good output (`s_last_hr_out` / `s_last_spo2_out`)
is held — so a transient dropout in one metric doesn't blank the other.

### 1.6 Calibration history (R → SpO2 mapping)

The slope→SpO2 linear formula was empirically anchored against a CMS50D
clinical pulse oximeter (data in `r_log.csv`, captured via `ppg_r_log.py`):

- Mean observed `R ≈ 2.405` (slope ≈ 0.2778) corresponded to clinical SpO2
  ≈ 98.3%.
- The high end of the slope range (`slope_max = 0.529412`, R ≥ 10 / severe
  desaturation) is anchored to SpO2 = 85% (lower clamp).
- Solving the two anchor points for `spo2 = A - B*slope` gives
  `A = 114.6`, `B = 55.85`.
- Output is clamped to `[85, 100]` — values below 85 are not considered
  reliable from this sensor/placement, and the formula is not validated below
  that range.

`RCAL: R=%d.%03d` is logged once every 5 s (throttled inside `spo2_count()`,
which itself runs ~once/sec) for ongoing calibration data collection via
`ppg_r_log.py` → `r_log.csv` (manual `spo2_clinical` column).

---

## 2. ECG (`ecg/ecg.c`, `dsp/filter.c`)

### 2.1 Acquisition

- `saadc_init()` — 12-bit SAADC on `AIN0`, gain `1/6`, internal 0.6 V reference
  (full-scale ≈ 3.6 V).
- `ppi_init()` — `TIMER3` generates a 250 Hz trigger, routed via PPI directly to
  the SAADC (hardware-timed sampling, no CPU jitter).

### 2.2 Filter chain (`ecg_process`, run per sample @ 250 Hz)

1. **Notch filter @ 50 Hz** (mains hum rejection) — 4th-order, implemented as
   **two cascaded 2nd-order biquads**, `Q = 10` each (`notch2_coeffs`).
2. **Bandpass 1–25 Hz** — 4th-order Butterworth, implemented as 2× high-pass
   biquads @ 1 Hz (`butter2_hp_coeffs`) cascaded with 2× low-pass biquads @ 25 Hz
   (`butter2_lp_coeffs`). Removes baseline wander and high-frequency noise while
   preserving the QRS complex.
3. **ALE (Adaptive Line Enhancer) via NLMS** — `nlms_init`/`nlms_step`,
   16 or 32-tap adaptive filter (`mu = 0.01`, `eps = 1e-6`), used via
   `delay_init`/`delay_process`/`ale_process` to further suppress noise that
   isn't well-modeled by the fixed notch/bandpass (adapts online).
4. **Savitzky-Golay smoothing** — `sg_init_n`/`sg_step`, window size = 11,
   closed-form coefficients; smooths the signal for peak detection without
   significantly distorting the QRS shape.

### 2.3 R-peak detection — two implementations

**v1 — `rpeak_detect()`** (simplified Pan-Tompkins adaptive threshold):
- Maintains an adaptive threshold that decays by a factor of `0.9999` per
  sample toward a floor of `50.0`.
- A sample exceeding the threshold (and past a refractory window) is accepted
  as an R-peak; the threshold is then raised toward the peak amplitude.

**v2 — `rpeak_detect_v2()`** (derivative² + adaptive envelope):
- Computes the squared derivative of the filtered signal.
- Maintains an adaptive envelope with decay `0.992`.
- Fires when the squared derivative exceeds `0.4 × envelope` (same
  envelope/threshold style as the PPG `adenv` detector, §1.2).
- `RPEAK2_REFRACT = 62` samples (~250 ms @ 250 Hz) refractory period.
- RR interval is the **median of the last 8** intervals (`RR2_BUF_SIZE = 8`),
  bounded to `RR2_MIN_SAMPLES = 75` .. `RR2_MAX_SAMPLES = 375`
  (i.e. 0.3–1.5 s, 40–200 BPM @ 250 Hz).

### 2.4 Generic filter primitives (`dsp/filter.c`)

- `iir1_filter` / `iir2_filter` (+ ring-buffer variants) — direct-form 1st/2nd
  order IIR.
- `biquad_init`/`biquad_step` — Direct-Form-II-Transposed biquad (used by all
  notch/Butterworth stages).
- `iir1_init`/`iir1_step` — single-pole IIR (general-purpose).
- Runtime coefficient calculators: `butter1_lp/hp_coeffs`, `butter2_lp/hp_coeffs`,
  `notch2_coeffs`, `ecg_bandpass_init` — all derived via the bilinear transform
  for arbitrary sample rate / cutoff, so filters can be re-tuned without
  hand-computing coefficients.
- Legacy hardcoded Butterworth coefficients (5 Hz / 10 Hz @ 100 Hz) remain in
  the file for reference/fallback but are not part of the active ECG chain.

---

## 3. Temperature — TMP117 (`drivers/temp/tmp117_v2.c`, `drivers/temp/temp_filter.c`)

### 3.1 Sensor driver (`tmp117_v2.c`)

- `tmp117_Init()` auto-scans the four possible I2C addresses (`GND/VCC/SDA/SCL`
  ADDR-pin straps), verifies the device ID, then configures averaging/conversion
  mode and high/low alert limits.
- `tmp117_get_Temp()` reads the raw 16-bit temperature register and converts:
  `temp_C = TMP117_RESOLUTION * (int16_t)raw` (resolution = 0.0078125 °C/LSB,
  per TMP117 datasheet). A `TMP117_TEMP_MIN`/`MAX` sanity range is checked and a
  warning logged if exceeded.
- `tmp117_wake_oneshot()` sets the `MOD` bits (15:14 → bits 9:8 of the config
  word here) to one-shot conversion mode; per the datasheet a one-shot
  conversion takes ~15.5 ms at default averaging (the project's CLAUDE.md notes
  a conservative 200 ms wait is used by the caller before reading).
- `tmp117_read_temperature()` wraps `tmp117_get_Temp()` with an
  `NRF_LOG_INFO` of the result.

### 3.2 Software filter (`temp_filter.c`)

`temp_filter_update()` applies, in order:

1. **Median-of-3** (`TEMP_FILTER_MEDIAN_SIZE = 3`) — rejects single-sample
   spikes. A `TEMP_FILTER_SETTLE_COUNT = 3` warm-up is required before the
   median window is considered valid.
2. **Rate limiter** (`TEMP_FILTER_RATE_LIMIT = 0.5 °C` per update) — clamps how
   fast the filtered output can change, smoothing step artifacts.
3. **EMA** (`TEMP_FILTER_EMA_ALPHA = 0.3`) — final exponential smoothing for a
   stable display value.

---

## 4. Accelerometer & Pedometer (`drivers/accel/mma845.c`, `accel_filter.c`, `pedometer.c`)

### 4.1 Raw read (`mma845.c` — `MMA8452Q_read`)

- Reads 6 bytes (X/Y/Z, 12-bit each, `raw >> 4`).
- Converts to physical units: `c{x,y,z} = raw / (1<<11) * m_scale` (g), then
  `g_accel.magnitude = sqrt(cx² + cy² + cz²) * 9.80665` (m/s²).
- Fall detection: `g_accel.fall_detected` is set from the `FF_MT_SRC` register
  (hardware free-fall/motion interrupt).

### 4.2 Software filter (`accel_filter.c`)

`accel_filter_update()` applies, in order:

1. **Median-of-3** — spike rejection.
2. **Rate limiter** (`ACCEL_FILTER_RATE_LIMIT = 2.0 m/s²` per update).
3. **EMA** (`ACCEL_FILTER_EMA_ALPHA = 0.5`) — fast-tracking smoothed magnitude.
4. **Gravity estimate** — a much slower EMA (`ACCEL_GRAVITY_ALPHA = 0.02`,
   ~50 samples / settling time) tracks the DC (gravity) component.
5. **AC component** — `ac = |ema_value - gravity_estimate|`, i.e. the
   motion-only component with gravity removed.

> **Note (dead code):** `accel_filter_update()` / `accel_filter_get_ac()` are
> not currently called from `main.c` or `mma845.c`. `g_accel.ac` — which
> `pedometer_update()` and `dashboard_update_steps()` read — is therefore never
> assigned and stays at its zero-initialized value. To make the AC-based
> pedometer path functional, `accel_filter_update()` must be called on each new
> accelerometer sample and its result stored into `g_accel.ac`.

### 4.3 Pedometer (`pedometer.c`)

`pedometer_update()`:

1. **Dynamic threshold** (`update_dynamic_threshold()`) — tracks
   `recent_peak_avg` and `recent_valley_avg` via EMA (`α = 0.2`); the step
   threshold is the midpoint of these two, scaled by `0.6`, clamped to
   `[0.15, 3.0]` m/s². Falls back to `PEDOMETER_INITIAL_THRESHOLD = 0.4` before
   enough peaks/valleys have been observed.
2. **Peak detection** — a step candidate is a rising→falling transition of the
   AC signal that crosses above the dynamic threshold.
3. **Validation** — candidates must satisfy:
   - Interval between `PEDOMETER_MIN_STEP_INTERVAL_MS = 250` and
     `PEDOMETER_MAX_STEP_INTERVAL_MS = 2000` (i.e. 30–240 steps/min).
   - `intervals_are_regular()` — the last `PEDOMETER_VALIDATION_COUNT = 3`
     intervals must be within `±50%` of each other
     (`PEDOMETER_INTERVAL_TOLERANCE = 0.5`), to reject isolated jolts.
4. **Cadence** — `add_step_to_cadence()` maintains an 8-sample sliding window
   of step intervals and an EMA (`α = 0.3`) for a smoothed steps-per-minute
   value.

---

## 5. Cross-cutting notes

- All HR/SpO2/ECG filter coefficients are computed at `init` time via the
  bilinear-transform helpers in `dsp/filter.c`, so sample-rate or cutoff
  changes only require updating the `*_coeffs(...)` call arguments — no
  hand-derived coefficient tables.
- `NRF_LOG_INFO` supports at most **7 format arguments**
  (`LOG_INTERNAL_0`..`LOG_INTERNAL_7`); an 8th argument causes a link error
  (`Undefined symbol LOG_INTERNAL_8`). Split wide debug logs into multiple
  `NRF_LOG_INFO` calls.
- PPG and ECG both use an "adaptive envelope + ratio threshold" peak-detection
  pattern (`PPG_PEAK_ENV_DECAY`/`PPG_PEAK_THRESH_RATIO` and
  `rpeak_detect_v2`'s envelope/`0.4×` threshold respectively) — conceptually
  the same self-calibrating approach applied to two different signals.
