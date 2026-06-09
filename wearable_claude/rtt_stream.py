import socket
import re
import csv
import matplotlib.pyplot as plt
from collections import deque

# =========================
# CONFIG
# =========================
HOST = "127.0.0.1"
PORT = 19021

MAX_SAMPLES = 1000
CSV_FILE = "ecg_log4.csv"

# =========================
# REGEX
# Matches:
# 00> <info> app: raw:1906 filt:20
# =========================
pattern = re.compile(
    r"raw:\s*(-?\d+)\s+filt:\s*(-?\d+)",
    re.IGNORECASE
)

# =========================
# BUFFERS
# =========================
raw_buf  = deque(maxlen=MAX_SAMPLES)
filt_buf = deque(maxlen=MAX_SAMPLES)

# =========================
# CSV SETUP
# =========================
csv_file = open(CSV_FILE, mode='w', newline='')

csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    "sample",
    "raw",
    "filtered"
])

sample_idx = 0

# =========================
# SOCKET
# =========================
sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

sock.connect((HOST, PORT))

sock.setblocking(False)

buffer = ""

# =========================
# PLOT SETUP
# =========================
plt.ion()

fig, axes = plt.subplots(
    2,
    1,
    figsize=(12, 7),
    sharex=True
)

fig.suptitle("ECG Real-time — J-Link RTT", fontsize=13, fontweight="bold")

# -------------------------
# RAW ECG
# -------------------------
line_raw, = axes[0].plot(
    [],
    [],
    color="tab:blue",
    linewidth=0.8,
    label="Raw ECG"
)

axes[0].set_title("Raw ECG (ADC counts)")

axes[0].set_ylabel("ADC Counts")

axes[0].grid(True, alpha=0.3)

axes[0].legend(loc="upper left")

# -------------------------
# FILTERED ECG
# -------------------------
line_filt, = axes[1].plot(
    [],
    [],
    color="tab:orange",
    linewidth=0.8,
    label="Filtered ECG"
)

axes[1].set_title("Filtered ECG  [0.5–25 Hz BP · 50 Hz notch]")

axes[1].set_ylabel("Amplitude")

axes[1].set_xlabel("Samples")

axes[1].grid(True, alpha=0.3)

axes[1].legend(loc="upper left")

plt.tight_layout()

# =========================
# REAL-TIME LOOP
# =========================
try:

    while True:

        # -------------------------
        # RECEIVE SOCKET DATA
        # -------------------------
        try:

            chunk = sock.recv(4096).decode(errors="ignore")

            if chunk:
                buffer += chunk

        except BlockingIOError:
            pass

        except Exception:
            pass

        # -------------------------
        # SPLIT LINES
        # -------------------------
        lines = buffer.split("\n")

        buffer = lines[-1]

        # -------------------------
        # PROCESS COMPLETE LINES
        # -------------------------
        for line in lines[:-1]:

            m = pattern.search(line)

            if not m:
                continue

            # -------------------------
            # PARSE VALUES
            # -------------------------
            raw_val  = float(m.group(1))

            filt_val = float(m.group(2))

            # -------------------------
            # STORE BUFFERS
            # -------------------------
            raw_buf.append(raw_val)

            filt_buf.append(filt_val)

            # -------------------------
            # SAVE CSV
            # -------------------------
            csv_writer.writerow([
                sample_idx,
                raw_val,
                filt_val
            ])

            csv_file.flush()

            sample_idx += 1

        # -------------------------
        # UPDATE PLOTS
        # -------------------------
        n = len(raw_buf)

        if n > 0:

            x_axis = range(n)

            line_raw.set_data(x_axis, list(raw_buf))

            line_filt.set_data(x_axis, list(filt_buf))

            for ax in axes:

                ax.relim()

                ax.autoscale_view()

        plt.pause(0.01)

# =========================
# EXIT
# =========================
except KeyboardInterrupt:

    print("Stopping ECG monitor...")

finally:

    csv_file.close()

    sock.close()

    print(f"CSV saved: {CSV_FILE}")
