#!/usr/bin/env python3
"""
ppg_rt_stream.py  —  Real-time PPG waveform viewer via J-Link RTT TCP

Setup:
  1. Flash firmware and connect J-Link.
  2. Open J-Link RTT Viewer → RTT Control Block found → leaves TCP server on 127.0.0.1:19021.
  3. python ppg_rt_stream.py [--host 127.0.0.1] [--port 19021] [--save ppg_log.csv]

Firmware logs one line per sample (100 Hz):
  NRF_LOG_INFO("ppg:%d %d", (int)ir_filtered, peak_accepted);

RTT Viewer wraps it as:  <info> app: ppg:1234 0
The parser searches for "ppg:" anywhere in each line so the prefix is ignored.
"""

import socket
import threading
import csv
import argparse
import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")           # swap to "Qt5Agg" if Tk is not installed
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── constants ────────────────────────────────────────────────────────────────
FS         = 100           # firmware sample rate (Hz)
WINDOW_SEC = 8             # rolling display window length
WINDOW_N   = FS * WINDOW_SEC


# ── reader thread ─────────────────────────────────────────────────────────────
class PPGReader:
    def __init__(self, host: str, port: int, csv_path: str):
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
            self._cw.writerow(["n", "t_s", "ir_filt", "peak"])

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
        if not parts:
            return
        try:
            val  = float(parts[0])
            peak = len(parts) > 1 and parts[1].strip() == "1"
        except ValueError:
            return
        with self.lock:
            self.n += 1
            t = self.n / FS
            self.sig.append(val)
            self.peaks.append(peak)
            self.ts.append(t)
        if self._csv:
            self._cw.writerow([self.n, f"{t:.3f}", f"{val:.1f}", int(peak)])

    def snapshot(self):
        with self.lock:
            return list(self.ts), list(self.sig), list(self.peaks)

    def close(self):
        self._run.clear()
        if self._csv:
            self._csv.close()


# ── plot ──────────────────────────────────────────────────────────────────────
BG   = "#111111"
GRID = "#2a2a2a"
SIG  = "#00ccff"
PKC  = "#ff3333"
TXT  = "#dddddd"


def main():
    ap = argparse.ArgumentParser(description="Real-time PPG viewer (J-Link RTT)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=19021)
    ap.add_argument("--save", default="ppg_log.csv",
                    help="CSV output path  (pass empty string to disable)")
    args   = ap.parse_args()
    reader = PPGReader(args.host, args.port, args.save or None)

    fig, ax = plt.subplots(figsize=(14, 4))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.grid(color=GRID, lw=0.5)
    for sp in ax.spines.values():
        sp.set_color("#333")
    ax.tick_params(colors=TXT, labelsize=9)
    ax.set_xlabel("Time (s)", color=TXT, fontsize=9)
    ax.set_ylabel("Filtered IR (counts)", color=TXT, fontsize=9)
    ax.set_title("PPG  —  HP 0.5 Hz → LP 5 Hz → ALE-NLMS → SG  (RTT live)",
                 color="white", fontsize=10, pad=6)

    line,   = ax.plot([], [], color=SIG, lw=0.9, label="IR filtered")
    scatter = ax.scatter([], [], color=PKC, s=40, zorder=5, label="Accepted peak")
    info    = ax.text(0.01, 0.96, "waiting for data…",
                      transform=ax.transAxes, color="white",
                      fontsize=10, va="top", family="monospace")
    ax.legend(loc="upper right", facecolor="#222", labelcolor=TXT,
              fontsize=8, framealpha=0.7)

    def update(_):
        ts, sig, pks = reader.snapshot()

        if max(abs(v) for v in sig) < 0.5:
            return line, scatter, info

        line.set_data(ts, sig)
        ax.set_xlim(ts[0], ts[-1] + 0.01)
        lo, hi = min(sig), max(sig)
        mg = max((hi - lo) * 0.12, 1.0)
        ax.set_ylim(lo - mg, hi + mg)

        # peak markers
        px = [t for t, p in zip(ts, pks) if p]
        py = [v for v, p in zip(sig, pks) if p]
        scatter.set_offsets(list(zip(px, py)) if px else [(0, lo - mg * 2)])

        # live HR from peak spacing
        hr_str = ""
        if len(px) >= 2:
            gaps = [px[i+1] - px[i] for i in range(len(px) - 1)
                    if 0.30 < px[i+1] - px[i] < 2.0]
            if gaps:
                hr = 60.0 / (sum(gaps) / len(gaps))
                hr_str = f"   HR ≈ {hr:.0f} bpm"

        info.set_text(f"t = {ts[-1]:.1f} s{hr_str}")
        return line, scatter, info

    ani = animation.FuncAnimation(fig, update, interval=100, blit=True)
    plt.tight_layout()
    print("[ppg_rt] plot open — close window to exit")
    try:
        plt.show()
    finally:
        reader.close()
        print("[ppg_rt] done")


if __name__ == "__main__":
    main()
