"""
ale_sweep.py
============
Standalone experiment (does not modify pipeline_replay.py / filter.h).

Sweeps ALE_DELAY x NLMS_TAPS x NLMS_MU through the exact on-device chain
(BPF -> notch x2 -> ALE -> SG) and scores each combination against the
firmware-logged "filtered" reference using:
    - correlation
    - RMSE
    - std (vs firmware reference std)
    - R-peak count / mean HR plausibility (sanity check on morphology)

Edit CSV_FILE / T_START / T_END to pick the analysis window.
"""

import numpy as np
import pandas as pd
from scipy import signal as sp

CSV_FILE = "ecg_log1.csv"
T_START  = 50.0
T_END    = 60.0

Fs       = 250.0
HP_HZ    = 1.0
LP_HZ    = 25.0
NOTCH_HZ = 50.0
NOTCH_Q  = 30.0
NLMS_EPS = 1e-6
SG_WINDOW = 11

SG_COEFFS = np.array([-36.0, 9.0, 44.0, 69.0, 84.0,
                       89.0, 84.0, 69.0, 44.0,  9.0, -36.0])
SG_NORM   = 429.0

# ---------------- filters (mirrors pipeline_replay.py) ----------------
class Biquad:
    def __init__(self, b0, b1, b2, a1, a2):
        self.b0, self.b1, self.b2, self.a1, self.a2 = b0, b1, b2, a1, a2
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
    K = np.tan(np.pi * fc / fs); K2 = K*K
    norm = 1.0 + np.sqrt(2.0)*K + K2
    return Biquad(K2/norm, 2.0*K2/norm, K2/norm,
                  2.0*(K2-1.0)/norm, (1.0 - np.sqrt(2.0)*K + K2)/norm)

def butter2_hp_coeffs(fs, fc):
    K = np.tan(np.pi * fc / fs); K2 = K*K
    norm = 1.0 + np.sqrt(2.0)*K + K2
    return Biquad(1.0/norm, -2.0/norm, 1.0/norm,
                  2.0*(K2-1.0)/norm, (1.0 - np.sqrt(2.0)*K + K2)/norm)

def notch2_coeffs(fs, fn, Q):
    w0 = 2.0*np.pi*fn/fs
    alpha = np.sin(w0)/(2.0*Q)
    cos_w0 = np.cos(w0)
    inv_a0 = 1.0/(1.0+alpha)
    b1_val = -2.0*cos_w0*inv_a0
    return Biquad(inv_a0, b1_val, inv_a0, b1_val, (1.0-alpha)*inv_a0)

class Delay:
    def __init__(self, n):
        self.buf = np.zeros(n); self.idx = 0; self.n = n
    def step(self, x):
        y = self.buf[self.idx]
        self.buf[self.idx] = x
        self.idx = (self.idx+1) % self.n
        return y

class NLMS:
    def __init__(self, taps, mu, eps):
        self.w = np.zeros(taps); self.x = np.zeros(taps)
        self.idx = 0; self.mu = mu; self.eps = eps; self.n = taps
    def step(self, x_new, d):
        self.x[self.idx] = x_new
        idxs = [(self.idx + self.n - i) % self.n for i in range(self.n)]
        xi = self.x[idxs]
        y = np.dot(self.w, xi)
        norm = np.dot(xi, xi)
        e = d - y
        g = self.mu / (norm + self.eps)
        self.w += g * e * xi
        self.idx = (self.idx+1) % self.n
        return y

def ale_process(nlms, delay, x_arr):
    out = np.empty(len(x_arr))
    for n, x in enumerate(x_arr):
        xd = delay.step(x)
        out[n] = nlms.step(xd, x)
    return out

class SGFilter:
    def __init__(self):
        self.buf = np.zeros(SG_WINDOW); self.head = 0
    def step(self, x):
        self.buf[self.head] = x
        self.head = (self.head+1) % SG_WINDOW
        acc = 0.0
        for i in range(SG_WINDOW):
            acc += SG_COEFFS[i] * self.buf[(self.head+i) % SG_WINDOW]
        return acc / SG_NORM
    def run(self, x_arr):
        return np.array([self.step(x) for x in x_arr])

# ---------------- load + fixed pre-ALE chain (computed once) ----------------
df = pd.read_csv(CSV_FILE)
ecg_raw = df["raw"].values.astype(float)
fw_filt = df["filtered"].values.astype(float)
n = len(ecg_raw)
t = np.arange(n) / Fs

x_min = max(0.0, T_START)
x_max = t[-1] if T_END is None else min(T_END, t[-1])
mask = (t >= x_min) & (t <= x_max)
print(f"Loaded {n} samples, analysing [{x_min:.1f}, {x_max:.1f}] s ({mask.sum()} samples)\n")

hp1 = butter2_hp_coeffs(Fs, HP_HZ); hp2 = butter2_hp_coeffs(Fs, HP_HZ)
lp1 = butter2_lp_coeffs(Fs, LP_HZ); lp2 = butter2_lp_coeffs(Fs, LP_HZ)
notch = notch2_coeffs(Fs, NOTCH_HZ, NOTCH_Q)

stage_bpf = lp2.run(lp1.run(hp2.run(hp1.run(ecg_raw))))
stage_x5 = notch.run(notch.run(stage_bpf))   # notch x2, matches firmware

fw_sel = fw_filt[mask]
valid = fw_sel != 0.0
fw_std = np.std(fw_sel[valid])
print(f"Firmware reference std over window: {fw_std:.1f}\n")

# ---------------- R-peak / HR sanity check ----------------
def hr_stats(sig):
    s = sig - np.mean(sig)
    peaks, _ = find_peaks_simple(s)
    if len(peaks) < 2:
        return 0, np.nan, np.nan
    rr_ms = np.diff(peaks) / Fs * 1000.0
    hr = 60000.0 / rr_ms
    return len(peaks), hr.mean(), rr_ms.std()

def find_peaks_simple(sig):
    thr = np.std(sig) * 1.5
    min_dist = int(0.3 * Fs)
    pk = []
    last = -min_dist
    for i in range(1, len(sig)-1):
        if sig[i] > thr and sig[i] >= sig[i-1] and sig[i] >= sig[i+1] and (i-last) >= min_dist:
            pk.append(i)
            last = i
    return np.array(pk), None

# ---------------- sweep ----------------
DELAYS = [5, 8, 10, 15]
TAPS   = [16, 24, 32]
MUS    = [0.01, 0.005]

results = []
print(f"{'delay':>6} {'taps':>5} {'mu':>6} | {'corr':>6} {'rmse':>7} {'std':>7} {'std_err%':>9} | {'beats':>6} {'HR':>6} {'RRstd':>7}")
print("-"*90)
for delay_n in DELAYS:
    for taps in TAPS:
        for mu in MUS:
            nlms = NLMS(taps, mu, NLMS_EPS)
            delay = Delay(delay_n)
            stage_x6 = ale_process(nlms, delay, stage_x5)
            sg = SGFilter()
            stage_x7 = sg.run(stage_x6)
            py_final = np.clip(stage_x7, -32768, 32767).astype(np.int16).astype(float)

            py_sel = py_final[mask]
            corr = np.corrcoef(fw_sel[valid], py_sel[valid])[0, 1]
            rmse = np.sqrt(np.mean((fw_sel[valid] - py_sel[valid])**2))
            py_std = np.std(py_sel[valid])
            std_err_pct = 100.0 * abs(py_std - fw_std) / fw_std

            n_peaks, hr, rr_std = hr_stats(py_sel[valid])

            results.append((delay_n, taps, mu, corr, rmse, py_std, std_err_pct, n_peaks, hr, rr_std))
            print(f"{delay_n:>6} {taps:>5} {mu:>6.3f} | {corr:>6.3f} {rmse:>7.1f} {py_std:>7.1f} "
                  f"{std_err_pct:>8.1f}% | {n_peaks:>6} {hr:>6.1f} {rr_std:>7.1f}")

print("\n--- current firmware setting (delay=5, taps=16, mu=0.01) shown above for reference ---")

# rank by combined score: prioritise correlation, then std error, then RMSE
best = sorted(results, key=lambda r: (-r[3], r[6], r[4]))[:5]
print("\nTop 5 candidates (ranked by correlation, then std-match, then RMSE):")
print(f"{'delay':>6} {'taps':>5} {'mu':>6} | {'corr':>6} {'rmse':>7} {'std_err%':>9} | {'beats':>6} {'HR':>6}")
for r in best:
    delay_n, taps, mu, corr, rmse, py_std, std_err_pct, n_peaks, hr, rr_std = r
    print(f"{delay_n:>6} {taps:>5} {mu:>6.3f} | {corr:>6.3f} {rmse:>7.1f} {std_err_pct:>8.1f}% | {n_peaks:>6} {hr:>6.1f}")
