import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal as sp

sys.stdout.reconfigure(encoding="utf-8")

# =========================
# CONFIG
# =========================
CSV_FILE  = "ecg_log1.csv"
FS        = 250.0     # Hz
T_START   = 0.0       # seconds
T_END     = None      # seconds  (set to None for full recording)

# Python Butterworth bandpass
BP_LO    = 1.0
BP_HI    = 25.0
BP_ORDER = 4

# Savitzky-Golay causal FIR — coefficients must match filter.c exactly
# savgol_filter() is zero-phase (forward+backward) → squared response → wrong
# sp.lfilter() is single causal pass → matches hardware
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
        # delay line: output x[n-delay], store x[n]
        x_delayed   = d_buf[d_idx]
        d_buf[d_idx] = signal[i]
        d_idx        = (d_idx + 1) % delay

        # write into NLMS ring buffer
        x_buf[x_idx] = x_delayed

        # tap vector: w[0]*x[n], w[1]*x[n-1], ...  (mirrors C indexing)
        idx_arr = (x_idx + n_taps - tap_idx) % n_taps
        x_vec   = x_buf[idx_arr]

        # output y = w^T * x
        y    = np.dot(w, x_vec)
        norm = np.dot(x_vec, x_vec)

        # NLMS weight update
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

total_samples = len(raw)
total_time    = total_samples / FS

print(f"Loaded {total_samples} samples  ({total_time:.1f} s)")

# =========================
# TIME AXIS
# =========================
t = np.arange(total_samples) / FS

# =========================
# TIME WINDOW
# =========================
t_end = min(T_END, t[-1]) if T_END is not None else t[-1]

mask = (t >= T_START) & (t <= t_end)

t_win   = t[mask]
raw_win = raw[mask]
flt_win = flt[mask]

print(f"Window: {T_START:.1f}s — {t_end:.1f}s  ({mask.sum()} samples)")

# =========================
# DC REMOVAL
# =========================
raw_win = raw_win - np.mean(raw_win)
flt_win = flt_win - np.mean(flt_win)

# =========================
# PYTHON FILTER CHAIN
# Mirrors full hardware chain: sosfilt → ALE → SG  (= x7)
# =========================
sos      = sp.butter(BP_ORDER, [BP_LO, BP_HI], btype="bandpass", fs=FS, output="sos")
py_bp    = sp.sosfilt(sos, raw_win)

print("Running ALE NLMS...")
py_ale   = ale_nlms(py_bp)

py_full  = sp.lfilter(SG_B, [1.0], py_ale)   # causal FIR, matches hardware

print(f"Python chain: sosfilt({BP_LO}–{BP_HI} Hz) → ALE(taps={NLMS_TAPS}, D={ALE_DELAY}) → SG(causal)")

# =========================
# PLOT
# =========================
fig, (ax_raw, ax_flt) = plt.subplots(
    2, 1,
    figsize=(14, 7),
    sharex=True
)

fig.suptitle(
    f"ECG Signal — Time Domain  [{T_START:.1f}s – {t_end:.1f}s]",
    fontsize=13,
    fontweight="bold"
)

# -------------------------
# RAW ECG
# -------------------------
ax_raw.plot(
    t_win,
    raw_win,
    color="tab:blue",
    linewidth=0.7,
    label="Raw ECG"
)

ax_raw.set_title("Raw ECG (ADC counts)")
ax_raw.set_ylabel("ADC Counts")
ax_raw.grid(True, alpha=0.3)
ax_raw.legend(loc="upper right", fontsize=9)

# -------------------------
# HW vs PYTHON FULL CHAIN
# -------------------------
ax_flt.plot(
    t_win,
    flt_win,
    color="tab:orange",
    linewidth=0.9,
    label="nRF52 HW (BP + ALE + SG = x7)"
)

ax_flt.plot(
    t_win,
    py_full,
    color="tab:red",
    linewidth=0.9,
    linestyle="--",
    label=f"Python: sosfilt → ALE(D={ALE_DELAY}) → SG(causal)"
)

ax_flt.set_title(
    f"nRF52 HW (x7) vs Python full chain  [{BP_LO}–{BP_HI} Hz]"
)
ax_flt.set_ylabel("Amplitude")
ax_flt.set_xlabel("Time (s)")
ax_flt.grid(True, alpha=0.3)
ax_flt.legend(loc="upper right", fontsize=9)

plt.tight_layout()
plt.savefig("plot_ecg.png", dpi=150)
plt.show()

print("Saved: plot_ecg.png")
