#!/usr/bin/env python3
"""
ppg_rt_stream.py  —  Real-time PPG waveform viewer via J-Link RTT TCP

Setup:
  1. Flash firmware and connect J-Link.
  2. Open J-Link RTT Viewer → RTT Control Block found → leaves TCP server on 127.0.0.1:19021.
  3. python ppg_rt_stream.py [--host 127.0.0.1] [--port 19021] [--save ppg_log.csv]

Firmware logs one line per sample (100 Hz):
  NRF_LOG_INFO("ppg:%d %d %d", (int)ir_raw, (int)ir_filtered, peak_accepted);

RTT Viewer wraps it as:  <info> app: ppg:54321 1234 0
The parser searches for "ppg:" anywhere in each line so the prefix is ignored.
"""

import socket
import threading
import csv
import argparse
import time
from collections import deque

import numpy as np
import matplotlib
matplotlib.use("TkAgg")           # swap to "Qt5Agg" if Tk is not installed
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── constants ────────────────────────────────────────────────────────────────
FS         = 100           # firmware sample rate (Hz)
WINDOW_SEC = 8             # rolling display window length
WINDOW_N   = FS * WINDOW_SEC
PSD_FMAX   = 8.0           # show PSD up to this frequency (Hz)


# ── reader thread ─────────────────────────────────────────────────────────────
class PPGReader:
    def __init__(self, host: str, port: int, csv_path: str):
        self.raw   = deque([0.0]   * WINDOW_N, maxlen=WINDOW_N)
        self.sig   = deque([0.0]   * WINDOW_N, maxlen=WINDOW_N)
        self.peaks = deque([False] * WINDOW_N, maxlen=WINDOW_N)
        self.ts    = deque([i / FS for i in range(WINDOW_N)], maxlen=WINDOW_N)
        self.n     = 0
        self.lock  = threading.Lock()
        self._run  = threading.Event()
        self._run.set()
        self._csv  = None

        if csv_path:
            self._csv = open(csv_path, "w", newline="", buffering=1)
            self._cw  = csv.writer(self._csv)
            self._cw.writerow(["n", "t_s", "ir_raw", "ir_filt", "peak"])

        threading.Thread(target=self._loop, args=(host, port), daemon=True).start()

    def _loop(self, host: str, port: int):
        while self._run.is_set():
            try:
                sock = socket.create_connection((host, port), timeout=5)
                print(f"[ppg_rt] connected  {host}:{port}")
                sock.settimeout(0.5)
                buf = ""
                while self._run.is_set():
                    try:
                        buf += sock.recv(4096).decode("ascii", errors="ignore")
                    except socket.timeout:
                        continue
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self._parse(line)
            except Exception as exc:
                print(f"[ppg_rt] {exc}  —  retrying in 2 s")
                time.sleep(2)

    def _parse(self, line: str):
        idx = line.find("ppg:")
        if idx < 0:
            return
        parts = line[idx + 4:].split()
        if len(parts) < 2:
            return
        try:
            raw_val = float(parts[0])
            val     = float(parts[1])
            peak    = len(parts) > 2 and parts[2].strip() == "1"
        except ValueError:
            return
        with self.lock:
            self.n += 1
            t = self.n / FS
            self.raw.append(raw_val)
            self.sig.append(val)
            self.peaks.append(peak)
            self.ts.append(t)
        if self._csv:
            self._cw.writerow([self.n, f"{t:.3f}", f"{raw_val:.1f}", f"{val:.1f}", int(peak)])

    def snapshot(self):
        with self.lock:
            return list(self.ts), list(self.raw), list(self.sig), list(self.peaks)

    def close(self):
        self._run.clear()
        if self._csv:
            self._csv.close()


# ── plot ──────────────────────────────────────────────────────────────────────
BG   = "#111111"
GRID = "#2a2a2a"
RAW  = "#888888"
SIG  = "#00ccff"
PKC  = "#ff3333"
PSD  = "#66ff99"
TXT  = "#dddddd"


def main():
    ap = argparse.ArgumentParser(description="Real-time PPG viewer (J-Link RTT)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=19021)
    ap.add_argument("--save", default="ppg_log.csv",
                    help="CSV output path  (pass empty string to disable)")
    args   = ap.parse_args()
    reader = PPGReader(args.host, args.port, args.save or None)

    fig, (ax, ax3) = plt.subplots(1, 2, figsize=(16, 5),
                                   gridspec_kw={"width_ratios": [2, 1]})
    fig.patch.set_facecolor(BG)

    # ── left: raw + filtered IR ─────────────────────────────────────────────
    ax.set_facecolor(BG)
    ax.grid(color=GRID, lw=0.5)
    for sp in ax.spines.values():
        sp.set_color("#333")
    ax.tick_params(colors=TXT, labelsize=9)
    ax.set_xlabel("Time (s)", color=TXT, fontsize=9)
    ax.set_ylabel("Raw IR (counts)", color=RAW, fontsize=9)
    ax.set_title("PPG  —  raw IR (left) vs HP 0.5 Hz → LP 5 Hz → SG filtered IR (right)  (RTT live)",
                 color="white", fontsize=10, pad=6)

    ax2 = ax.twinx()
    ax2.set_facecolor(BG)
    for sp in ax2.spines.values():
        sp.set_color("#333")
    ax2.tick_params(colors=TXT, labelsize=9)
    ax2.set_ylabel("Filtered IR (counts)", color=SIG, fontsize=9)

    raw_line, = ax.plot([], [], color=RAW, lw=0.8, label="IR raw")
    line,     = ax2.plot([], [], color=SIG, lw=0.9, label="IR filtered")
    scatter   = ax2.scatter([], [], color=PKC, s=40, zorder=5, label="Accepted peak")
    info      = ax.text(0.01, 0.96, "waiting for data…",
                      transform=ax.transAxes, color="white",
                      fontsize=10, va="top", family="monospace")
    lines = [raw_line, line, scatter]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc="upper right", facecolor="#222", labelcolor=TXT,
              fontsize=8, framealpha=0.7)

    # ── right: PSD of the filtered IR signal ────────────────────────────────
    ax3.set_facecolor(BG)
    ax3.grid(color=GRID, lw=0.5)
    for sp in ax3.spines.values():
        sp.set_color("#333")
    ax3.tick_params(colors=TXT, labelsize=9)
    ax3.set_xlabel("Frequency (Hz)", color=TXT, fontsize=9)
    ax3.set_ylabel("PSD (dB)", color=PSD, fontsize=9)
    ax3.set_title("PSD of filtered IR (Welch, Hann window)",
                  color="white", fontsize=10, pad=6)
    ax3.set_xlim(0, PSD_FMAX)

    psd_line, = ax3.plot([], [], color=PSD, lw=1.2)
    hr_marker = ax3.axvline(0, color=PKC, lw=1.0, ls="--", label="HR freq")
    ax3.legend(loc="upper right", facecolor="#222", labelcolor=TXT,
               fontsize=8, framealpha=0.7)

    def update(_):
        ts, raw, sig, pks = reader.snapshot()

        if max(abs(v) for v in sig) < 0.5:
            return raw_line, line, scatter, info, psd_line, hr_marker

        raw_line.set_data(ts, raw)
        line.set_data(ts, sig)
        ax.set_xlim(ts[0], ts[-1] + 0.01)

        rlo, rhi = min(raw), max(raw)
        rmg = max((rhi - rlo) * 0.12, 1.0)
        ax.set_ylim(rlo - rmg, rhi + rmg)

        lo, hi = min(sig), max(sig)
        mg = max((hi - lo) * 0.12, 1.0)
        ax2.set_ylim(lo - mg, hi + mg)

        # peak markers
        px = [t for t, p in zip(ts, pks) if p]
        py = [v for v, p in zip(sig, pks) if p]
        scatter.set_offsets(list(zip(px, py)) if px else [(0, lo - mg * 2)])

        # ── PSD via Welch's method (50%-overlap Hann segments) ──────────────
        x = np.asarray(sig, dtype=float)
        x = x - np.mean(x)
        seg_len = min(len(x), 256)
        step    = seg_len // 2
        win     = np.hanning(seg_len)
        win_pow = np.sum(win ** 2)

        psds = []
        for start in range(0, len(x) - seg_len + 1, step):
            seg = x[start:start + seg_len] * win
            spec = np.fft.rfft(seg)
            psds.append((np.abs(spec) ** 2) / (FS * win_pow))
        if not psds:
            return raw_line, line, scatter, info, psd_line, hr_marker

        psd  = np.mean(psds, axis=0)
        freqs = np.fft.rfftfreq(seg_len, d=1.0 / FS)
        psd_db = 10.0 * np.log10(psd + 1e-12)

        mask = freqs <= PSD_FMAX
        psd_line.set_data(freqs[mask], psd_db[mask])
        ax3.set_ylim(np.min(psd_db[mask]) - 3, np.max(psd_db[mask]) + 3)

        # dominant frequency in the plausible HR band (0.5-3.5 Hz = 30-210 bpm)
        hr_psd = 0.0
        band = (freqs >= 0.5) & (freqs <= 3.5)
        if np.any(band):
            f_peak = freqs[band][np.argmax(psd_db[band])]
            hr_marker.set_xdata([f_peak, f_peak])
            hr_psd = f_peak * 60.0

        # time-domain HR from accepted-peak spacing (firmware adenv detector)
        hr_peaks = 0.0
        if len(px) >= 2:
            gaps = [px[i+1] - px[i] for i in range(len(px) - 1)
                    if 0.30 < px[i+1] - px[i] < 2.0]
            if gaps:
                hr_peaks = 60.0 / (sum(gaps) / len(gaps))

        hr_str = ""
        if hr_psd > 0:
            hr_str += f"   HR(PSD) ≈ {hr_psd:.0f} bpm"
        if hr_peaks > 0:
            hr_str += f"   HR(peaks) ≈ {hr_peaks:.0f} bpm"

        # flag harmonic mismatch: PSD peak landing near 2x or 0.5x the
        # peak-detector rate usually means the fundamental/harmonic got picked
        if hr_psd > 0 and hr_peaks > 0:
            ratio = hr_psd / hr_peaks
            if abs(ratio - 2.0) < 0.2:
                hr_str += "  [PSD≈2x peaks]"
            elif abs(ratio - 0.5) < 0.1:
                hr_str += "  [PSD≈0.5x peaks]"

        info.set_text(f"t = {ts[-1]:.1f} s{hr_str}")
        return raw_line, line, scatter, info, psd_line, hr_marker

    ani = animation.FuncAnimation(fig, update, interval=200, blit=False)
    plt.tight_layout()
    print("[ppg_rt] plot open — close window to exit")
    try:
        plt.show()
    finally:
        reader.close()
        print("[ppg_rt] done")


if __name__ == "__main__":
    main()
