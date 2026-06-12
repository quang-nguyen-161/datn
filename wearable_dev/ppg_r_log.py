#!/usr/bin/env python3
"""
ppg_r_log.py — Capture R-ratio calibration samples via J-Link RTT TCP

Setup:
  1. Flash firmware and connect J-Link.
  2. Open J-Link RTT Viewer -> RTT Control Block found -> leaves TCP server on 127.0.0.1:19021.
  3. python ppg_r_log.py [--host 127.0.0.1] [--port 19021] [--save r_log.csv]

Firmware logs once every ~5 s:
  NRF_LOG_INFO("RCAL: R=%d.%03d", ...);

RTT Viewer wraps it as:  <info> app: RCAL: R=2.437
This script appends one row per sample to the CSV:
  timestamp, R, spo2_clinical
The spo2_clinical column is left empty for you to fill in later from the
CMS50D reading taken at that time.
"""

import socket
import csv
import re
import argparse
import time
from datetime import datetime

RCAL_RE = re.compile(r"RCAL:\s*R=(-?\d+\.\d+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=19021)
    ap.add_argument("--save", default="r_log.csv")
    args = ap.parse_args()

    f = open(args.save, "w", newline="", buffering=1)
    writer = csv.writer(f)
    writer.writerow(["timestamp", "R", "spo2_clinical"])
    print(f"[ppg_r_log] writing to {args.save}")

    while True:
        try:
            sock = socket.create_connection((args.host, args.port), timeout=5)
            print(f"[ppg_r_log] connected {args.host}:{args.port}")
            sock.settimeout(0.5)
            buf = ""
            while True:
                try:
                    chunk = sock.recv(4096).decode("ascii", errors="ignore")
                except socket.timeout:
                    continue
                if not chunk:
                    raise ConnectionError("socket closed")
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    m = RCAL_RE.search(line)
                    if not m:
                        continue
                    r_val = float(m.group(1))
                    ts = datetime.now().isoformat(timespec="seconds")
                    writer.writerow([ts, r_val, ""])
                    print(f"{ts}  R={r_val:.3f}")
        except Exception as exc:
            print(f"[ppg_r_log] {exc} -- retrying in 2 s")
            time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
