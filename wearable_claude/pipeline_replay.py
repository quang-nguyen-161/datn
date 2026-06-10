"""
pipeline_replay.py
==================
Replays the exact on-device ECG signal chain (main.c / filter.c) in Python,
sample-by-sample, on the raw ADC column of ecg_log1.csv:

    raw -> BPF 1-25 Hz (4th-order Butterworth: 2x HPF biquad + 2x LPF biquad)
        -> notch50 (applied twice, Q=30) -> ALE (delay + NLMS, 16 taps)
        -> Savitzky-Golay (window=11, hardcoded coeffs) -> *ECG_TX_SCALE (10)

Each stage is plotted and saved as its own figure, and the final Python
output is compared against the "filtered" column already logged by the
firmware in ecg_log1.csv (its on-device final stage).

To analyse a different slice of the log, edit T_START / T_END below.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal as sp

# ============================================================
#  CONFIG — edit these to pick the CSV file and time range
# ============================================================
CSV_FILE = "ecg_log3.csv"
T_START  = 18.0     # seconds
T_END    = 28.0    # seconds  (set to None for full recording)

# Overlay PSDs of selected stages on one combined figure (1-based stage indices,
# matching the "Stage N" numbering used for the per-stage files below):
#   1 Raw ADC | 2 BPF 1-25Hz | 3 Notch 50Hz | 4 ALE | 5 Savitzky-Golay
#   6 Firmware logged (filtered) -- the on-device "filtered" column from the CSV
# Example: [5, 6] overlays the Python replay's final stage with the firmware's
# own logged output, to see how closely the replay matches the real device.
# Set to None or [] to skip the combined overlay plot.
PSD_COMPARE_STAGES = [1,5,6]

Fs            = 250.0
HP_HZ         = 1.0
LP_HZ         = 25.0
NOTCH_HZ      = 50.0
NOTCH_Q       = 30.0
NLMS_TAPS     = 16
NLMS_MU       = 0.005
NLMS_EPS      = 1e-6
ALE_DELAY     = 5
SG_WINDOW     = 11
ECG_TX_SCALE  = 10.0

SG_COEFFS = np.array([-36.0, 9.0, 44.0, 69.0, 84.0,
                       89.0, 84.0, 69.0, 44.0,  9.0, -36.0])
SG_NORM   = 429.0

# ============================================================
#  Biquad (Direct Form II Transposed) — matches biquad_step()
# ============================================================
class Biquad:
    def __init__(self, b0, b1, b2, a1, a2):
        self.b0, self.b1, self.b2 = b0, b1, b2
        self.a1, self.a2 = a1, a2
        self.s1 = 0.0
        self.s2 = 0.0

    def step(self, x):
        y = self.b0 * x + self.s1
        self.s1 = self.b1 * x - self.a1 * y + self.s2
        self.s2 = self.b2 * x - self.a2 * y
        return y

    def run(self, x_arr):
        return np.array([self.step(x) for x in x_arr])


def butter2_lp_coeffs(fs, fc):
    K    = np.tan(np.pi * fc / fs)
    K2   = K * K
    norm = 1.0 + np.sqrt(2.0) * K + K2
    return Biquad(K2 / norm, 2.0 * K2 / norm, K2 / norm,
                  2.0 * (K2 - 1.0) / norm,
                  (1.0 - np.sqrt(2.0) * K + K2) / norm)


def butter2_hp_coeffs(fs, fc):
    K    = np.tan(np.pi * fc / fs)
    K2   = K * K
    norm = 1.0 + np.sqrt(2.0) * K + K2
    return Biquad(1.0 / norm, -2.0 / norm, 1.0 / norm,
                  2.0 * (K2 - 1.0) / norm,
                  (1.0 - np.sqrt(2.0) * K + K2) / norm)


def notch2_coeffs(fs, fn, Q):
    w0     = 2.0 * np.pi * fn / fs
    alpha  = np.sin(w0) / (2.0 * Q)
    cos_w0 = np.cos(w0)
    inv_a0 = 1.0 / (1.0 + alpha)
    b1_val = -2.0 * cos_w0 * inv_a0
    return Biquad(inv_a0, b1_val, inv_a0,
                  b1_val, (1.0 - alpha) * inv_a0)


# ============================================================
#  Delay line — matches delay_process()
# ============================================================
class Delay:
    def __init__(self, n):
        self.buf = np.zeros(n)
        self.idx = 0
        self.n   = n

    def step(self, x):
        y = self.buf[self.idx]
        self.buf[self.idx] = x
        self.idx = (self.idx + 1) % self.n
        return y


# ============================================================
#  NLMS adaptive filter — matches nlms_step()
# ============================================================
class NLMS:
    def __init__(self, taps, mu, eps):
        self.w   = np.zeros(taps)
        self.x   = np.zeros(taps)
        self.idx = 0
        self.mu  = mu
        self.eps = eps
        self.n   = taps

    def step(self, x_new, d):
        self.x[self.idx] = x_new

        idxs = [(self.idx + self.n - i) % self.n for i in range(self.n)]
        xi   = self.x[idxs]

        y    = np.dot(self.w, xi)
        norm = np.dot(xi, xi)

        e = d - y
        g = self.mu / (norm + self.eps)

        self.w += g * e * xi

        self.idx = (self.idx + 1) % self.n
        return y


def ale_process(nlms, delay, x_arr):
    out = np.empty(len(x_arr))
    for n, x in enumerate(x_arr):
        x_delayed = delay.step(x)
        out[n] = nlms.step(x_delayed, x)
    return out


# ============================================================
#  Savitzky-Golay ring-buffer smoother — matches sg_step()
# ============================================================
class SGFilter:
    def __init__(self):
        self.buf  = np.zeros(SG_WINDOW)
        self.head = 0

    def step(self, x):
        self.buf[self.head] = x
        self.head = (self.head + 1) % SG_WINDOW
        acc = 0.0
        for i in range(SG_WINDOW):
            acc += SG_COEFFS[i] * self.buf[(self.head + i) % SG_WINDOW]
        return acc / SG_NORM

    def run(self, x_arr):
        return np.array([self.step(x) for x in x_arr])


# ============================================================
#  LOAD DATA
# ============================================================
df       = pd.read_csv(CSV_FILE)
ecg_raw  = df["raw"].values.astype(float)
fw_filt  = df["filtered"].values.astype(float)
n        = len(ecg_raw)
t        = np.arange(n) / Fs

print(f"Loaded {n} samples from {CSV_FILE}  ({n / Fs:.1f} s)")

# ============================================================
#  STAGE-BY-STAGE PIPELINE  (mirrors main.c main loop exactly)
# ============================================================
hp1 = butter2_hp_coeffs(Fs, HP_HZ)
hp2 = butter2_hp_coeffs(Fs, HP_HZ)
lp1 = butter2_lp_coeffs(Fs, LP_HZ)
lp2 = butter2_lp_coeffs(Fs, LP_HZ)
notch = notch2_coeffs(Fs, NOTCH_HZ, NOTCH_Q)

# 4th-order Butterworth bandpass (1-25 Hz) = 2x HPF biquad + 2x LPF biquad cascade
stage_bpf = hp1.run(ecg_raw)
stage_bpf = hp2.run(stage_bpf)
stage_bpf = lp1.run(stage_bpf)
stage_bpf = lp2.run(stage_bpf)

stage_x5 = notch.run(stage_bpf)           # 50 Hz notch, pass 1
stage_x5 = notch.run(stage_x5)            # 50 Hz notch, pass 2 (matches main.c — applied twice)

nlms  = NLMS(NLMS_TAPS, NLMS_MU, NLMS_EPS)
delay = Delay(ALE_DELAY)
stage_x6 = ale_process(nlms, delay, stage_x5)   # NLMS adaptive line enhancer

sg = SGFilter()
stage_x7 = sg.run(stage_x6)               # Savitzky-Golay smoothing

# Firmware logs (int16_t)x7 directly (see main.c: NRF_LOG_INFO("raw:%d filt:%d", raw, (int16_t)x7))
# — NOT the BLE TX-scaled value (x7 * ECG_TX_SCALE is just a linear rescale for BLE
# transport and looks identical to x7, so it isn't shown as its own stage).
py_final = np.clip(stage_x7, -32768, 32767).astype(np.int16).astype(float)

stages = [
    ("Raw ADC",                         ecg_raw,   "tab:blue"),
    ("BPF 1-25 Hz (Butterworth, order 4)", stage_bpf, "tab:orange"),
    ("Notch 50 Hz (Q=30)",              stage_x5,  "tab:brown"),
    ("ALE (NLMS, 16 taps)",             stage_x6,  "tab:pink"),
    ("Savitzky-Golay (w=11)",           stage_x7,  "tab:gray"),
    ("Firmware logged (filtered)",      fw_filt,   "tab:red"),
]

# Selectable time window (set via T_START / T_END above) — drives all plots
# and the PSD/SNR analysis below
x_min = max(0.0, T_START)
x_max = t[-1] if T_END is None else min(T_END, t[-1])
if x_max <= x_min:
    raise SystemExit(f"T_END ({T_END}) must be greater than T_START ({T_START}); "
                     f"log spans 0-{t[-1]:.1f} s")
mask = (t >= x_min) & (t <= x_max)
print(f"Analysing range [{x_min:.1f}, {x_max:.1f}] s  ({mask.sum()} samples)")

for i, (name, sig, color) in enumerate(stages, start=1):
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t[mask], sig[mask], color=color, linewidth=0.8)
    ax.set_title(f"Stage {i}: {name}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Amplitude")
    ax.set_xlim([x_min, x_max])
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fname = f"pipeline_stage{i}_{name.split(' ')[0].lower().replace('(', '').replace(')', '')}.png"
    plt.savefig(fname, dpi=150)
    plt.close(fig)
    print(f"Saved: {fname}")

# ============================================================
#  COMPARE: Python pipeline final stage vs firmware-logged "filtered"
# ============================================================
fig, ax = plt.subplots(figsize=(13, 5))
ax.plot(t[mask], fw_filt[mask],  color="tab:orange", linewidth=1.0,
        label="Firmware final (logged in ecg_log1.csv)")
ax.plot(t[mask], py_final[mask], color="tab:blue",   linewidth=1.0,
        linestyle="--", label="Python pipeline replay (final stage)")
ax.set_title("Final stage comparison — Firmware output vs Python pipeline replay")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Amplitude")
ax.set_xlim([x_min, x_max])
ax.grid(True, alpha=0.3)
ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig("pipeline_compare_final.png", dpi=150)
plt.close(fig)
print("Saved: pipeline_compare_final.png")

# Quantitative agreement (over the selected range only)
fw_sel    = fw_filt[mask]
py_sel    = py_final[mask]
valid     = fw_sel != 0.0
corr = np.corrcoef(fw_sel[valid], py_sel[valid])[0, 1]
rmse = np.sqrt(np.mean((fw_sel[valid] - py_sel[valid]) ** 2))
print(f"Correlation (firmware vs python final), [{x_min:.1f}, {x_max:.1f}] s: {corr:.4f}")
print(f"RMSE (firmware vs python final),        [{x_min:.1f}, {x_max:.1f}] s: {rmse:.2f}")

# ============================================================
#  PSD + SNR PER STAGE  (mirrors psd_ecg.py: Welch PSD, in/out-of-band SNR)
# ============================================================
SNR_LO  = 0.5
SNR_HI  = 25.0
NPERSEG = 512


def calc_snr(freqs, psd, lo, hi):
    df_bin    = freqs[1] - freqs[0]
    sig_mask  = (freqs >= lo) & (freqs <= hi)
    sig_pow   = np.sum(psd[sig_mask])  * df_bin
    noise_pow = np.sum(psd[~sig_mask]) * df_bin
    return 10.0 * np.log10(sig_pow / (noise_pow + 1e-12))


n_sel   = int(mask.sum())
nperseg = min(NPERSEG, n_sel // 4)

print(f"\nPer-stage PSD / SNR over selected range [{x_min:.1f}, {x_max:.1f}] s "
      f"(in-band {SNR_LO}-{SNR_HI} Hz vs out-of-band):")
for i, (name, sig, color) in enumerate(stages, start=1):
    sig_sel      = sig[mask]
    sig_dc       = sig_sel - np.mean(sig_sel)
    freqs, psd   = sp.welch(sig_dc, fs=Fs, nperseg=nperseg)
    snr_db       = calc_snr(freqs, psd, SNR_LO, SNR_HI)
    psd_db       = 10 * np.log10(psd + 1e-12)

    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(freqs, psd_db, color=color, linewidth=1.1, label=name)
    ax.axvspan(SNR_LO, SNR_HI, alpha=0.07, color="green", label=f"ECG band ({SNR_LO}-{SNR_HI} Hz)")
    ax.axvline(50.0, color="red",    ls=":",  lw=1.0, alpha=0.8, label="50 Hz notch")
    ax.axvline(SNR_HI, color="purple", ls="--", lw=1.0, alpha=0.7, label=f"{SNR_HI} Hz LP")
    ax.axvline(SNR_LO, color="gray",   ls="--", lw=1.0, alpha=0.7, label=f"{SNR_LO} Hz HP")
    ax.set_title(f"Stage {i} PSD: {name}  —  SNR (in/out-of-band) = {snr_db:.1f} dB")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("PSD (dB/Hz)")
    ax.set_xlim([0, Fs / 2])
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    plt.tight_layout()
    fname = f"pipeline_stage{i}_psd_{name.split(' ')[0].lower().replace('(', '').replace(')', '')}.png"
    plt.savefig(fname, dpi=150)
    plt.close(fig)
    print(f"  Stage {i:2d} [{name:32s}] SNR = {snr_db:6.1f} dB   -> {fname}")

# ------------------------------------------------------------
#  COMBINED OVERLAY — selected stages' PSDs on one figure
#  (set PSD_COMPARE_STAGES above to choose which stages, e.g. [1, 3])
# ------------------------------------------------------------
if PSD_COMPARE_STAGES:
    fig, ax = plt.subplots(figsize=(13, 6))
    title_parts = []
    for idx in PSD_COMPARE_STAGES:
        if not (1 <= idx <= len(stages)):
            print(f"  (skipping PSD_COMPARE_STAGES entry {idx}: no such stage)")
            continue
        name, sig, color = stages[idx - 1]
        sig_sel    = sig[mask]
        sig_dc     = sig_sel - np.mean(sig_sel)
        freqs, psd = sp.welch(sig_dc, fs=Fs, nperseg=nperseg)
        snr_db     = calc_snr(freqs, psd, SNR_LO, SNR_HI)
        psd_db     = 10 * np.log10(psd + 1e-12)

        ax.plot(freqs, psd_db, color=color, linewidth=1.2,
                label=f"Stage {idx}: {name}  (SNR {snr_db:.1f} dB)")
        title_parts.append(f"Stage {idx} ({snr_db:.1f} dB)")

    ax.axvspan(SNR_LO, SNR_HI, alpha=0.07, color="green", label=f"ECG band ({SNR_LO}-{SNR_HI} Hz)")
    ax.axvline(50.0, color="red",    ls=":",  lw=1.0, alpha=0.8, label="50 Hz notch")
    ax.axvline(SNR_HI, color="purple", ls="--", lw=1.0, alpha=0.7, label=f"{SNR_HI} Hz LP")
    ax.axvline(SNR_LO, color="gray",   ls="--", lw=1.0, alpha=0.7, label=f"{SNR_LO} Hz HP")
    ax.set_title("PSD comparison — " + " vs ".join(title_parts))
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("PSD (dB/Hz)")
    ax.set_xlim([0, Fs / 2])
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=9, loc="upper right")
    plt.tight_layout()
    stage_tag = "_".join(str(i) for i in PSD_COMPARE_STAGES)
    fname = f"pipeline_psd_compare_stages_{stage_tag}.png"
    plt.savefig(fname, dpi=150)
    plt.close(fig)
    print(f"\nSaved combined PSD overlay (stages {PSD_COMPARE_STAGES}): {fname}")

# ------------------------------------------------------------
#  OVERALL PSD/SNR COMPARISON — Raw vs Firmware-logged vs Python final
#  (mirrors psd_ecg.py final summary plot)
# ------------------------------------------------------------
raw_dc      = ecg_raw[mask]  - np.mean(ecg_raw[mask])
fw_filt_dc  = fw_filt[mask]  - np.mean(fw_filt[mask])
py_final_dc = py_final[mask] - np.mean(py_final[mask])

f_raw, p_raw = sp.welch(raw_dc,      fs=Fs, nperseg=nperseg)
f_fw,  p_fw  = sp.welch(fw_filt_dc,  fs=Fs, nperseg=nperseg)
f_py,  p_py  = sp.welch(py_final_dc, fs=Fs, nperseg=nperseg)

snr_raw = calc_snr(f_raw, p_raw, SNR_LO, SNR_HI)
snr_fw  = calc_snr(f_fw,  p_fw,  SNR_LO, SNR_HI)
snr_py  = calc_snr(f_py,  p_py,  SNR_LO, SNR_HI)

print(f"\nOverall SNR  Raw:               {snr_raw:6.1f} dB")
print(f"Overall SNR  Firmware (logged): {snr_fw:6.1f} dB  ({snr_fw - snr_raw:+.1f} dB)")
print(f"Overall SNR  Python replay:     {snr_py:6.1f} dB  ({snr_py - snr_raw:+.1f} dB)")

fig, ax = plt.subplots(figsize=(13, 6))
fig.suptitle(
    f"PSD — Raw vs Firmware-logged filtered vs Python pipeline replay\n"
    f"SNR  Raw: {snr_raw:.1f} dB  |  "
    f"Firmware: {snr_fw:.1f} dB ({snr_fw - snr_raw:+.1f})  |  "
    f"Python: {snr_py:.1f} dB ({snr_py - snr_raw:+.1f})",
    fontsize=11, fontweight="bold")

ax.plot(f_raw, 10*np.log10(p_raw + 1e-12), color="tab:blue",   linewidth=1.0, label="Raw ECG")
ax.plot(f_fw,  10*np.log10(p_fw  + 1e-12), color="tab:orange", linewidth=1.2, label="Firmware-logged filtered (x7)")
ax.plot(f_py,  10*np.log10(p_py  + 1e-12), color="tab:red",    linewidth=1.2, linestyle="--",
        label="Python pipeline replay (final stage)")

ax.axvspan(SNR_LO, SNR_HI, alpha=0.07, color="green", label=f"ECG band ({SNR_LO}-{SNR_HI} Hz)")
ax.axvline(50.0, color="red",    ls=":",  lw=1.0, alpha=0.8, label="50 Hz notch")
ax.axvline(SNR_HI, color="purple", ls="--", lw=1.0, alpha=0.7, label=f"{SNR_HI} Hz LP")
ax.axvline(SNR_LO, color="gray",   ls="--", lw=1.0, alpha=0.7, label=f"{SNR_LO} Hz HP")

ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("PSD (dB/Hz)")
ax.set_xlim([0, Fs / 2])
ax.grid(True, which="both", alpha=0.3)
ax.legend(fontsize=8, loc="upper right")

plt.tight_layout()
plt.savefig("pipeline_compare_psd.png", dpi=150)
plt.close(fig)
print("Saved: pipeline_compare_psd.png")
