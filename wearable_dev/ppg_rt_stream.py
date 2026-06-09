import socket
import re
import csv
import matplotlib.pyplot as plt
from collections import deque

# ============================================================
# CONFIG
# ============================================================
HOST = "127.0.0.1"
PORT = 19021

MAX_SAMPLES = 1000
CSV_FILE = "ppg_log.csv"

# ============================================================
# REGEX
#
# Matches:
# 00> <info> app: ir: 112442, fil: 631
# ============================================================
pattern = re.compile(
    r"ir:\s*(\d+)\s*,\s*fil:\s*(-?\d+)",
    re.IGNORECASE
)

# ============================================================
# BUFFERS
# ============================================================
ir_buf = deque(maxlen=MAX_SAMPLES)
fil_buf = deque(maxlen=MAX_SAMPLES)

# ============================================================
# CSV SETUP
# ============================================================
csv_file = open(CSV_FILE, mode="w", newline="")

csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    "sample",
    "ir_raw",
    "filtered"
])

sample_idx = 0

# ============================================================
# RTT SOCKET
#
# Start:
# JLinkRTTLogger.exe
# or
# JLinkRTTClient.exe
#
# Default RTT TCP port = 19021
# ============================================================
sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

print(f"Connecting to RTT @ {HOST}:{PORT} ...")

sock.connect((HOST, PORT))

sock.setblocking(False)

print("Connected.")

buffer = ""

# ============================================================
# PLOT SETUP
# ============================================================
plt.ion()

fig, axes = plt.subplots(
    3,
    1,
    figsize=(12, 8),
    sharex=True
)

fig.suptitle(
    "MAX30102 Real-Time Monitor",
    fontsize=14,
    fontweight="bold"
)

# ------------------------------------------------------------
# RAW IR
# ------------------------------------------------------------
line_ir, = axes[0].plot(
    [],
    [],
    linewidth=0.8,
    label="Raw IR"
)

axes[0].set_title("Raw IR Signal")

axes[0].set_ylabel("Counts")

axes[0].grid(True, alpha=0.3)

axes[0].legend(loc="upper left")

# ------------------------------------------------------------
# FILTERED
# ------------------------------------------------------------
line_fil, = axes[1].plot(
    [],
    [],
    linewidth=0.8,
    label="Filtered"
)

axes[1].set_title("Filtered Signal")

axes[1].set_ylabel("Amplitude")

axes[1].grid(True, alpha=0.3)

axes[1].legend(loc="upper left")

# ------------------------------------------------------------
# ZOOMED FILTERED
# ------------------------------------------------------------
line_zoom, = axes[2].plot(
    [],
    [],
    linewidth=1.2,
    label="PPG Waveform"
)

axes[2].set_title("Zoomed Pulse Waveform")

axes[2].set_ylabel("Amplitude")

axes[2].set_xlabel("Samples")

axes[2].grid(True, alpha=0.3)

axes[2].legend(loc="upper left")

plt.tight_layout()

# ============================================================
# MAIN LOOP
# ============================================================
try:

    while True:

        # ----------------------------------------------------
        # RECEIVE RTT DATA
        # ----------------------------------------------------
        try:

            chunk = sock.recv(4096).decode(
                errors="ignore"
            )

            if chunk:
                buffer += chunk

        except BlockingIOError:
            pass

        except Exception:
            pass

        # ----------------------------------------------------
        # SPLIT COMPLETE LINES
        # ----------------------------------------------------
        lines = buffer.split("\n")

        buffer = lines[-1]

        # ----------------------------------------------------
        # PARSE LINES
        # ----------------------------------------------------
        for line in lines[:-1]:

            m = pattern.search(line)

            if not m:
                continue

            ir_val = int(m.group(1))
            fil_val = int(m.group(2))

            ir_buf.append(ir_val)
            fil_buf.append(fil_val)

            csv_writer.writerow([
                sample_idx,
                ir_val,
                fil_val
            ])

            sample_idx += 1

        csv_file.flush()

        # ----------------------------------------------------
        # UPDATE PLOTS
        # ----------------------------------------------------
        n = len(ir_buf)

        if n > 0:

            x_axis = range(n)

            ir_list = list(ir_buf)
            fil_list = list(fil_buf)

            line_ir.set_data(
                x_axis,
                ir_list
            )

            line_fil.set_data(
                x_axis,
                fil_list
            )

            line_zoom.set_data(
                x_axis,
                fil_list
            )

            # Auto scale raw + filtered
            axes[0].relim()
            axes[0].autoscale_view()

            axes[1].relim()
            axes[1].autoscale_view()

            # Zoomed pulse waveform
            if len(fil_list) > 10:

                ymin = min(fil_list) - 50
                ymax = max(fil_list) + 50

                if ymin == ymax:
                    ymin -= 1
                    ymax += 1

                axes[2].set_xlim(
                    0,
                    len(fil_list)
                )

                axes[2].set_ylim(
                    ymin,
                    ymax
                )

        plt.pause(0.01)

# ============================================================
# EXIT
# ============================================================
except KeyboardInterrupt:

    print("\nStopping monitor...")

finally:

    csv_file.close()

    sock.close()

    print(f"CSV saved to: {CSV_FILE}")