"""
sg_sweep.py
===========
Tests whether widening the Savitzky-Golay window (currently 11, poly order 3,
hardcoded in filter.c) would give more smoothing without hurting the signal.

For each candidate window, generates SG coefficients (poly order 3, matching
filter.c's design) via scipy.signal.savgol_coeffs, applies them causally
(mirrors sg_step's ring-buffer FIR -> equivalent to lfilter), and reports:
    - high-frequency power reduction (how much "smoother")
    - QRS peak amplitude retention vs the pre-SG (post-ALE) signal
    - added group delay (window-1)/2 samples -> ms

Does not modify pipeline_replay.py / filter.c.
"""

import numpy as np
import pandas as pd
from scipy import signal as sp

CSV_FILE = "ecg_log1.csv"
T_START  = 50.0
T_END    = 60.0

Fs       = 250.0
HP_HZ, LP_HZ = 1.0, 25.0
NOTCH_HZ, NOTCH_Q = 50.0, 30.0
NLMS_TAPS, NLMS_MU, NLMS_EPS, ALE_DELAY = 16, 0.005, 1e-6, 5

# ---- reuse minimal filter chain (mirrors pipeline_replay.py) ----
class Biquad:
    def __init__(self, b0, b1, b2, a1, a2):
        self.b0,self.b1,self.b2,self.a1,self.a2 = b0,b1,b2,a1,a2
        self.s1=self.s2=0.0
    def step(self,x):
        y=self.b0*x+self.s1
        self.s1=self.b1*x-self.a1*y+self.s2
        self.s2=self.b2*x-self.a2*y
        return y
    def run(self,xs): return np.array([self.step(x) for x in xs])

def hp(fs,fc):
    K=np.tan(np.pi*fc/fs); K2=K*K; n=1+np.sqrt(2)*K+K2
    return Biquad(1/n,-2/n,1/n,2*(K2-1)/n,(1-np.sqrt(2)*K+K2)/n)
def lp(fs,fc):
    K=np.tan(np.pi*fc/fs); K2=K*K; n=1+np.sqrt(2)*K+K2
    return Biquad(K2/n,2*K2/n,K2/n,2*(K2-1)/n,(1-np.sqrt(2)*K+K2)/n)
def notch(fs,fn,Q):
    w0=2*np.pi*fn/fs; a=np.sin(w0)/(2*Q); c=np.cos(w0); ia=1/(1+a); b1=-2*c*ia
    return Biquad(ia,b1,ia,b1,(1-a)*ia)

class Delay:
    def __init__(s,n): s.buf=np.zeros(n); s.idx=0; s.n=n
    def step(s,x):
        y=s.buf[s.idx]; s.buf[s.idx]=x; s.idx=(s.idx+1)%s.n; return y
class NLMS:
    def __init__(s,t,mu,eps): s.w=np.zeros(t); s.x=np.zeros(t); s.idx=0; s.mu=mu; s.eps=eps; s.n=t
    def step(s,xn,d):
        s.x[s.idx]=xn
        idxs=[(s.idx+s.n-i)%s.n for i in range(s.n)]
        xi=s.x[idxs]
        y=np.dot(s.w,xi); norm=np.dot(xi,xi)
        e=d-y; g=s.mu/(norm+s.eps)
        s.w += g*e*xi
        s.idx=(s.idx+1)%s.n
        return y
def ale(nlms,delay,xs):
    out=np.empty(len(xs))
    for n,x in enumerate(xs):
        xd=delay.step(x); out[n]=nlms.step(xd,x)
    return out

df=pd.read_csv(CSV_FILE)
raw=df["raw"].values.astype(float)
n=len(raw); t=np.arange(n)/Fs
x_min=max(0.0,T_START); x_max=t[-1] if T_END is None else min(T_END,t[-1])
mask=(t>=x_min)&(t<=x_max)
print(f"Analysing [{x_min:.1f},{x_max:.1f}]s ({mask.sum()} samples)\n")

bpf = lp(Fs,LP_HZ).run(lp(Fs,LP_HZ).run(hp(Fs,HP_HZ).run(hp(Fs,HP_HZ).run(raw))))
x5  = notch(Fs,NOTCH_HZ,NOTCH_Q).run(notch(Fs,NOTCH_HZ,NOTCH_Q).run(bpf))
x6  = ale(NLMS(NLMS_TAPS,NLMS_MU,NLMS_EPS), Delay(ALE_DELAY), x5)   # post-ALE, pre-SG

x6_sel = x6[mask]
nperseg = min(512, mask.sum()//4)

def hf_power(sig):
    # power above the ECG band (>25 Hz) -- what SG is supposed to suppress
    f,p = sp.welch(sig - np.mean(sig), fs=Fs, nperseg=nperseg)
    return np.sum(p[f > LP_HZ])

def qrs_amplitude(sig):
    # crude proxy: 95th percentile peak-to-peak swing -> tracks QRS height
    s = sig - np.mean(sig)
    return np.percentile(np.abs(s), 99)

hf_pre  = hf_power(x6_sel)
qrs_pre = qrs_amplitude(x6_sel)
print(f"Pre-SG (post-ALE):  HF power(>25Hz)={hf_pre:.3g}   QRS-amp(99pct)={qrs_pre:.1f}\n")

print(f"{'window':>7} {'delay_ms':>9} | {'HF power':>10} {'HF reduct%':>11} | {'QRS amp':>8} {'QRS keep%':>10}")
print("-"*65)
for win in [11, 15, 19, 23, 27]:
    coeffs = sp.savgol_coeffs(win, polyorder=3)   # centered FIR, applied causally via lfilter
    sg_out = sp.lfilter(coeffs, [1.0], x6)
    sel = sg_out[mask]
    hf  = hf_power(sel)
    qrs = qrs_amplitude(sel)
    delay_ms = (win - 1) / 2 / Fs * 1000.0
    print(f"{win:>7} {delay_ms:>9.1f} | {hf:>10.3g} {100*(1-hf/hf_pre):>10.1f}% | "
          f"{qrs:>8.1f} {100*qrs/qrs_pre:>9.1f}%")
