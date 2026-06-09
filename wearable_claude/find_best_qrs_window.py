"""
find_best_qrs_window.py
=======================
Slides a 10s window across the firmware-logged "filtered" signal in
ecg_log1.csv, scores each window on QRS-shape quality, and plots the
best-scoring window (zoomed time-domain view with detected R-peaks marked).

Score components (all favour clean, well-formed, regular QRS complexes):
    - peak sharpness: median QRS amplitude (99th percentile |signal|)
    - regularity: low RR-interval std (consistent rhythm = clean detection)
    - plausibility: HR within a sane resting range (50-110 bpm)
    - detectability: a full beat count for the window length (no missed beats)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

CSV_FILE = "ecg_log1.csv"
Fs = 250.0
WIN_S = 10.0
STEP_S = 2.0

df = pd.read_csv(CSV_FILE)
flt = df["filtered"].values.astype(float)
n = len(flt)
t = np.arange(n) / Fs
total_s = n / Fs
print(f"Loaded {n} samples ({total_s:.1f} s)")

win_n  = int(WIN_S * Fs)
step_n = int(STEP_S * Fs)

def find_peaks_simple(sig):
    thr = np.std(sig) * 1.5
    min_dist = int(0.3 * Fs)
    pk = []
    last = -min_dist
    for i in range(1, len(sig)-1):
        if sig[i] > thr and sig[i] >= sig[i-1] and sig[i] >= sig[i+1] and (i-last) >= min_dist:
            pk.append(i)
            last = i
    return np.array(pk)

candidates = []
for start in range(0, n - win_n, step_n):
    seg = flt[start:start+win_n]
    seg = seg - np.mean(seg)
    if np.all(seg == seg[0]):
        continue
    peaks = find_peaks_simple(seg)
    if len(peaks) < 6:
        continue
    rr_ms = np.diff(peaks) / Fs * 1000.0
    hr = 60000.0 / rr_ms
    hr_mean = hr.mean()
    rr_std = rr_ms.std()

    # hard gates: a clean resting-ECG segment has both a steady beat-to-beat
    # interval AND a physiologically plausible rate -- artifact/ringing bursts
    # can look "regular" while being far too fast/tall to be real heartbeats
    if rr_std > 150.0:
        continue
    if not (50.0 <= hr_mean <= 110.0):
        continue

    peak_amps = seg[peaks]
    qrs_amp = np.median(peak_amps)             # robust to single-sample artifacts
    qrs_consistency = np.std(peak_amps) / (qrs_amp + 1e-9)   # lower = more uniform beat shape

    # outlier penalty: any sample far beyond the typical QRS height = motion artifact
    outlier_ratio = np.max(np.abs(seg)) / (qrs_amp + 1e-9)
    outlier_penalty = max(0.0, outlier_ratio - 2.5) * 50

    plausible = 1.0 if (50 <= hr_mean <= 110) else 0.0
    # reward: tall+uniform QRS, regular rhythm, plausible HR, full beat coverage
    # penalise: RR jitter, shape inconsistency, artifact outliers
    score = (qrs_amp
             - 0.5 * rr_std
             - 100 * qrs_consistency
             - outlier_penalty
             + 40 * plausible
             + 3 * len(peaks))

    candidates.append((start, score, len(peaks), hr_mean, rr_std, qrs_amp))

candidates.sort(key=lambda c: -c[1])
best = candidates[0]
start, score, n_pk, hr_mean, rr_std, qrs_amp = best
t0 = start / Fs
t1 = (start + win_n) / Fs

print(f"\nBest window: [{t0:.1f}, {t1:.1f}] s")
print(f"  beats={n_pk}  HR={hr_mean:.1f} bpm  RRstd={rr_std:.1f} ms  QRS-amp(99pct)={qrs_amp:.1f}  score={score:.1f}")

print("\nTop 5 candidate windows:")
for c in candidates[:5]:
    s, sc, npk, hrm, rrs, qa = c
    print(f"  [{s/Fs:6.1f}, {(s+win_n)/Fs:6.1f}] s | beats={npk:2d}  HR={hrm:6.1f}  RRstd={rrs:6.1f}  QRSamp={qa:6.1f}  score={sc:6.1f}")

# ---- plot the winning window ----
seg   = flt[start:start+win_n]
seg_dc = seg - np.mean(seg)
t_seg = t[start:start+win_n]
peaks = find_peaks_simple(seg_dc)

fig, ax = plt.subplots(figsize=(14, 5))
ax.plot(t_seg, seg_dc, color="tab:orange", linewidth=1.0, label="Firmware filtered (x7)")
ax.plot(t_seg[peaks], seg_dc[peaks], "rx", markersize=9, markeredgewidth=2, label=f"Detected R-peaks (n={len(peaks)})")
ax.set_title(f"Best QRS-shape window: [{t0:.1f}-{t1:.1f}]s  —  HR={hr_mean:.1f} bpm, "
             f"RR jitter={rr_std:.1f} ms, QRS amp={qrs_amp:.1f}")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Amplitude (DC-removed)")
ax.grid(True, alpha=0.3)
ax.legend(fontsize=10, loc="upper right")
plt.tight_layout()
plt.savefig("best_qrs_window.png", dpi=150)
plt.close(fig)
print("\nSaved: best_qrs_window.png")
print(f"\n--> Set T_START = {t0:.1f}, T_END = {t1:.1f} in pipeline_replay.py / plot_ecg.py to inspect this window further.")
