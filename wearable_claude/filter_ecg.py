"""
filter_ecg.py
=============
Figure 1: 3 subplots — Raw ECG | nRF52 HW filtered | Savitzky-Golay
Figure 2: PSD of Raw vs HW filtered vs SavGol, M2 band-power SNR in title
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal as sp
from scipy.signal import savgol_filter

sys.stdout.reconfigure(encoding="utf-8")

# ============================================================
#  LOAD
# ============================================================
df           = pd.read_csv("ecg_log.csv")
ecg_raw_full = df["raw"].values.astype(float)
filt_full    = df["filtered"].values.astype(float)

Fs         = 250.0
total_time = np.arange(len(ecg_raw_full)) / Fs

# ============================================================
#  TIME WINDOW
# ============================================================
x_min = 0.0
x_max = min(80.0, total_time[-1])
mask  = (total_time >= x_min) & (total_time <= x_max)

sample = total_time[mask]
ecg    = ecg_raw_full[mask]
filt   = filt_full[mask]

print(f"Window: {x_min}s - {x_max}s  ({mask.sum()} samples)")

# ============================================================
#  DC REMOVAL
# ============================================================
ecg_dc  = ecg  - np.mean(ecg)
filt_dc = filt - np.mean(filt)

# Amplitude match if firmware gain differs
gain_ratio = np.std(filt_dc) / (np.std(ecg_dc) + 1e-12)
if not (0.5 < gain_ratio < 2.0):
    k          = np.dot(ecg_dc, filt_dc) / (np.dot(filt_dc, filt_dc) + 1e-12)
    filt_dc_sc = filt_dc * k
    print(f"Amplitude match applied: k={k:.4f}")
else:
    filt_dc_sc = filt_dc

# ============================================================
#  PYTHON BANDPASS FILTER — Butterworth 0.5–25 Hz (mirrors nRF52 spec)
# ============================================================
BP_LO    = 0.5
BP_HI    = 25.0
BP_ORDER = 4

sos_bp    = sp.butter(BP_ORDER, [BP_LO, BP_HI], btype="bandpass", fs=Fs, output="sos")
py_filt   = sp.sosfiltfilt(sos_bp, ecg_dc)

# Amplitude-match Python filter to HW filtered for fair visual comparison
k_py     = np.dot(filt_dc_sc, py_filt) / (np.dot(py_filt, py_filt) + 1e-12)
py_filt_sc = py_filt * k_py
print(f"Python BPF — order: {BP_ORDER}, [{BP_LO}–{BP_HI} Hz], amplitude scale k={k_py:.4f}")

# ============================================================
#  SAVITZKY-GOLAY — applied on top of HW filtered signal
#
#  window_length : ~40 ms at 250 Hz → 11 samples (must be odd)
#  polyorder     : 4  (preserves QRS peak shape well)
# ============================================================
SG_WINDOW = 13   # samples  (~44 ms at 250 Hz)
SG_ORDER  = 3    # polynomial order

savgol_sig = savgol_filter(filt_dc_sc, window_length=SG_WINDOW, polyorder=SG_ORDER)
print(f"SavGol params — window: {SG_WINDOW} samples ({SG_WINDOW/Fs*1000:.0f} ms), order: {SG_ORDER}")

# ============================================================
#  SNR — in-band signal power vs full out-of-band noise power
#
#  Signal power = PSD integral over [lo, hi]  (0.5–25 Hz ECG band)
#  Noise  power = PSD integral over everything outside [lo, hi]
#                 (baseline wander below 0.5 Hz + all leakage above 25 Hz)
#  Any energy a filter leaks above 25 Hz is correctly penalised as noise.
# ============================================================
def snr_band(sig, fs, lo=0.5, hi=25.0, nperseg=512):
    freqs, psd = sp.welch(sig, fs=fs, nperseg=min(nperseg, len(sig)//4))
    df_f       = freqs[1] - freqs[0]
    sig_mask   = (freqs >= lo) & (freqs <= hi)
    noise_mask = ~sig_mask
    sig_pow    = np.sum(psd[sig_mask])   * df_f
    noise_pow  = np.sum(psd[noise_mask]) * df_f
    return 10.0 * np.log10(sig_pow / (noise_pow + 1e-12))

snr_raw_db    = snr_band(ecg_dc,     Fs)
snr_hw_db     = snr_band(filt_dc_sc, Fs)
snr_py_db     = snr_band(py_filt_sc, Fs)
snr_savgol_db = snr_band(savgol_sig, Fs)

print(f"SNR (in-band/out-of-band) — Raw: {snr_raw_db:.1f} dB  |  HW filtered: {snr_hw_db:.1f} dB  |  Python BPF: {snr_py_db:.1f} dB  |  SavGol: {snr_savgol_db:.1f} dB")

# ============================================================
#  FIGURE 1 — Raw | HW filtered | SavGol  (3 subplots)
# ============================================================
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig1.suptitle("ECG Signal — Raw vs nRF52 HW Filtered vs Python BPF vs Savitzky-Golay",
              fontsize=12, fontweight="bold")

# Subplot 1: Raw
ax1.plot(sample, ecg, color="tab:blue", linewidth=0.7, label="Raw ECG")
ax1.set_title("Raw ECG (ADC counts)")
ax1.set_ylabel("ADC Counts")
ax1.set_xlim([x_min, x_max])
ax1.grid(True, alpha=0.3)
ax1.legend(fontsize=9)

# Subplot 2: nRF52 HW filtered vs Python Butterworth BPF
ax2.plot(sample, filt_dc_sc, color="tab:orange", linewidth=0.9, label="nRF52 HW Filtered")
ax2.plot(sample, py_filt_sc, color="tab:red",    linewidth=0.9, linestyle="--",
         label=f"Python BPF Butterworth (order={BP_ORDER}, {BP_LO}–{BP_HI} Hz)")
ax2.set_title(f"nRF52 HW Filtered vs Python BPF (Butterworth order={BP_ORDER}, {BP_LO}–{BP_HI} Hz)")
ax2.set_ylabel("Amplitude")
ax2.set_xlim([x_min, x_max])
ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=9)

# Subplot 3: Savitzky-Golay
ax3.plot(sample, filt_dc_sc, color="tab:orange", linewidth=0.5,
         alpha=0.4, label="HW Filtered (ref)")
ax3.plot(sample, savgol_sig, color="tab:green",  linewidth=1.0,
         label=f"Savitzky-Golay (w={SG_WINDOW}, order={SG_ORDER})")
ax3.set_title(f"Savitzky-Golay on HW Filtered  [window={SG_WINDOW} smp / {SG_WINDOW/Fs*1000:.0f} ms, order={SG_ORDER}]")
ax3.set_ylabel("Amplitude")
ax3.set_xlabel("Time (s)")
ax3.set_xlim([x_min, x_max])
ax3.grid(True, alpha=0.3)
ax3.legend(fontsize=9)

plt.tight_layout()
plt.savefig("ecg_fig1_signals.png", dpi=150)
plt.show()
print("Saved: ecg_fig1_signals.png")

# ============================================================
#  FIGURE 2 — PSD: Raw vs HW filtered vs SavGol  (SNR in title)
# ============================================================
def rms_norm(x):
    rms = np.sqrt(np.mean(x**2))
    return x / rms if rms > 0 else x

nperseg = min(512, len(ecg_dc) // 4)
f_raw,    p_raw    = sp.welch(rms_norm(ecg_dc),     fs=Fs, nperseg=nperseg)
f_hw,     p_hw     = sp.welch(rms_norm(filt_dc_sc), fs=Fs, nperseg=nperseg)
f_py,     p_py     = sp.welch(rms_norm(py_filt_sc), fs=Fs, nperseg=nperseg)
f_savgol, p_savgol = sp.welch(rms_norm(savgol_sig), fs=Fs, nperseg=nperseg)

fig2, ax = plt.subplots(figsize=(13, 5))

ax.plot(f_raw,    10*np.log10(p_raw    + 1e-12),
        color="tab:blue",   linewidth=1.0, label="Raw ECG")
ax.plot(f_hw,     10*np.log10(p_hw     + 1e-12),
        color="tab:orange", linewidth=1.2, label="nRF52 HW Filtered")
ax.plot(f_py,     10*np.log10(p_py     + 1e-12),
        color="tab:red",    linewidth=1.2, linestyle="-.",
        label=f"Python BPF (Butterworth order={BP_ORDER}, {BP_LO}–{BP_HI} Hz)")
ax.plot(f_savgol, 10*np.log10(p_savgol + 1e-12),
        color="tab:green",  linewidth=1.2, linestyle="--",
        label=f"Savitzky-Golay (w={SG_WINDOW}, order={SG_ORDER})")

# Band shading
ax.axvspan(0.5, 25.0, alpha=0.07, color="green",  label="ECG signal band (0.5-25 Hz)")
ax.axvline(50.0, color="red",    ls=":",  lw=1.0, alpha=0.8, label="50 Hz notch")
ax.axvline(25.0, color="purple", ls="--", lw=1.0, alpha=0.8, label="25 Hz BPF cutoff")
ax.axvline(0.5,  color="gray",   ls="--", lw=1.0, alpha=0.8, label="0.5 Hz HP cutoff")

ax.set_title(
    f"Power Spectral Density — Raw vs nRF52 HW Filtered vs Python BPF vs Savitzky-Golay\n"
    f"SNR (in-band / out-of-band)  |  Raw: {snr_raw_db:.1f} dB  "
    f"|  HW: {snr_hw_db:.1f} dB  "
    f"|  Python BPF: {snr_py_db:.1f} dB  "
    f"|  SavGol: {snr_savgol_db:.1f} dB",
    fontsize=10, fontweight="bold"
)
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("PSD (dB/Hz)")
ax.set_xlim([0, 100])
ax.set_ylim([-60, 10])
ax.grid(True, which="both", alpha=0.3)
ax.legend(fontsize=8, loc="upper right")

plt.tight_layout()
plt.savefig("ecg_fig2_psd.png", dpi=150)
plt.show()
print("Saved: ecg_fig2_psd.png")