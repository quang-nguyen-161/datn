import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal as sp

sys.stdout.reconfigure(encoding="utf-8")

# =========================
# CONFIG
# =========================
CSV_FILE = "ecg_log.csv"
FS       = 250.0
NPERSEG  = 512
SNR_LO   = 1.0    # Hz  — ECG signal band
SNR_HI   = 25.0   # Hz

# Butterworth bandpass
BP_LO    = 1.0
BP_HI    = 25.0
BP_ORDER = 4

# Savitzky-Golay causal FIR — must match filter.c coefficients exactly
SG_B = np.array([-36, 9, 44, 69, 84, 89, 84, 69, 44, 9, -36]) / 429.0

# ALE NLMS  (must match filter.h and main.c)
NLMS_TAPS = 16
ALE_DELAY = 5
NLMS_MU   = 0.01
NLMS_EPS  = 1e-6

# =========================
# ALE — mirrors filter.c
# =========================
def ale_nlms(signal, n_taps=NLMS_TAPS, delay=ALE_DELAY, mu=NLMS_MU, eps=NLMS_EPS):
    n      = len(signal)
    output = np.zeros(n)
    w      = np.zeros(n_taps)
    x_buf  = np.zeros(n_taps)
    d_buf  = np.zeros(delay)
    x_idx  = 0
    d_idx  = 0

    tap_idx = np.arange(n_taps)

    for i in range(n):
        x_delayed    = d_buf[d_idx]
        d_buf[d_idx] = signal[i]
        d_idx        = (d_idx + 1) % delay

        x_buf[x_idx] = x_delayed

        idx_arr = (x_idx + n_taps - tap_idx) % n_taps
        x_vec   = x_buf[idx_arr]

        y    = np.dot(w, x_vec)
        norm = np.dot(x_vec, x_vec)

        e  = signal[i] - y
        w += (mu / (norm + eps)) * e * x_vec

        output[i] = y
        x_idx = (x_idx + 1) % n_taps

    return output

# =========================
# LOAD CSV
# =========================
df  = pd.read_csv(CSV_FILE)
raw = df["raw"].values.astype(float)
flt = df["filtered"].values.astype(float)

print(f"Loaded {len(raw)} samples  ({len(raw)/FS:.1f} s)")

# =========================
# DC REMOVAL
# =========================
raw = raw - np.mean(raw)
flt = flt - np.mean(flt)

# =========================
# PYTHON FULL CHAIN
# sosfilt → ALE → SG  (mirrors hardware x7)
# =========================
sos    = sp.butter(BP_ORDER, [BP_LO, BP_HI], btype="bandpass", fs=FS, output="sos")
py_bp  = sp.sosfilt(sos, raw)

print("Running ALE NLMS...")
py_ale  = ale_nlms(py_bp)

py_full = sp.lfilter(SG_B, [1.0], py_ale)   # causal FIR, matches hardware

print(f"Python chain: sosfilt → ALE(taps={NLMS_TAPS}, D={ALE_DELAY}) → SG(causal)")

# =========================
# WELCH PSD
# =========================
nperseg = min(NPERSEG, len(raw) // 4)

f_raw,  p_raw  = sp.welch(raw,     fs=FS, nperseg=nperseg)
f_flt,  p_flt  = sp.welch(flt,     fs=FS, nperseg=nperseg)
f_py,   p_py   = sp.welch(py_full, fs=FS, nperseg=nperseg)

p_raw_db = 10 * np.log10(p_raw  + 1e-12)
p_flt_db = 10 * np.log10(p_flt  + 1e-12)
p_py_db  = 10 * np.log10(p_py   + 1e-12)

# =========================
# SNR  (in-band / out-of-band)
# =========================
def calc_snr(freqs, psd, lo, hi):
    df_bin    = freqs[1] - freqs[0]
    sig_mask  = (freqs >= lo) & (freqs <= hi)
    sig_pow   = np.sum(psd[sig_mask])  * df_bin
    noise_pow = np.sum(psd[~sig_mask]) * df_bin
    return 10.0 * np.log10(sig_pow / (noise_pow + 1e-12))

snr_raw = calc_snr(f_raw, p_raw, SNR_LO, SNR_HI)
snr_flt = calc_snr(f_flt, p_flt, SNR_LO, SNR_HI)
snr_py  = calc_snr(f_py,  p_py,  SNR_LO, SNR_HI)

print(f"SNR  Raw:            {snr_raw:.1f} dB")
print(f"SNR  nRF52 HW:       {snr_flt:.1f} dB  ({snr_flt - snr_raw:+.1f} dB)")
print(f"SNR  Python chain:   {snr_py:.1f} dB  ({snr_py  - snr_raw:+.1f} dB)")

# =========================
# PLOT
# =========================
fig, ax = plt.subplots(figsize=(13, 6))

fig.suptitle(
    f"Power Spectral Density — Raw vs nRF52 HW vs Python chain\n"
    f"SNR  Raw: {snr_raw:.1f} dB  |  "
    f"nRF52 HW: {snr_flt:.1f} dB ({snr_flt - snr_raw:+.1f})  |  "
    f"Python: {snr_py:.1f} dB ({snr_py - snr_raw:+.1f})",
    fontsize=11,
    fontweight="bold"
)

ax.plot(f_raw, p_raw_db,
        color="tab:blue",   linewidth=1.0, label="Raw ECG")

ax.plot(f_flt, p_flt_db,
        color="tab:orange", linewidth=1.2, label="nRF52 HW (x7)")

ax.plot(f_py, p_py_db,
        color="tab:red",    linewidth=1.2, linestyle="--",
        label=f"Python: sosfilt → ALE(D={ALE_DELAY}) → SG(causal)")

ax.axvspan(0.5, 25.0, alpha=0.07, color="green",  label="ECG band (0.5–25 Hz)")
ax.axvline(50.0, color="red",    ls=":",  lw=1.0, alpha=0.8, label="50 Hz notch")
ax.axvline(25.0, color="purple", ls="--", lw=1.0, alpha=0.7, label="25 Hz LP")
ax.axvline(0.5,  color="gray",   ls="--", lw=1.0, alpha=0.7, label="0.5 Hz HP")

ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("PSD (dB/Hz)")
ax.set_xlim([0, FS / 2])
ax.grid(True, which="both", alpha=0.3)
ax.legend(fontsize=9, loc="upper right")

plt.tight_layout()
plt.savefig("psd_ecg.png", dpi=150)
plt.show()

print("Saved: psd_ecg.png")
