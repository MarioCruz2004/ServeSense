import serial
import json
import time
from collections import deque

import matplotlib.pyplot as plt

# -----------------------------
# Configuration
# -----------------------------
PORT = '/dev/cu.usbmodem1101'
BAUD = 115200

GYRO_BIAS_X = -2
GYRO_BIAS_Y = -4
GYRO_BIAS_Z = 2

MAX_POINTS_ON_SCREEN = 200
OUTPUT_JSON = 'data.json'
PLOT_EVERY_N_SAMPLES = 10   # only redraw every 10 samples

# -----------------------------
# Serial setup
# -----------------------------
ser = serial.Serial(PORT, BAUD, timeout=1)
print("3 seconds until start")
time.sleep(3)

# Clear any old buffered data
ser.reset_input_buffer()

print("Sending 'g' trigger...")
ser.write(b'g')

# -----------------------------
# Data storage
# -----------------------------
data_points = []
recording = False

x_vals = deque(maxlen=MAX_POINTS_ON_SCREEN)
gx_vals = deque(maxlen=MAX_POINTS_ON_SCREEN)
gy_vals = deque(maxlen=MAX_POINTS_ON_SCREEN)
gz_vals = deque(maxlen=MAX_POINTS_ON_SCREEN)

sample_index = 0

# -----------------------------
# Plot setup
# -----------------------------
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))

line_gx, = ax.plot([], [], label='gx corrected')
line_gy, = ax.plot([], [], label='gy corrected')
line_gz, = ax.plot([], [], label='gz corrected')

ax.set_title("Live Gyroscope Data (Bias Corrected)")
ax.set_xlabel("Sample")
ax.set_ylabel("Gyro Value")
ax.legend()
ax.grid(True)

def update_plot():
    line_gx.set_data(x_vals, gx_vals)
    line_gy.set_data(x_vals, gy_vals)
    line_gz.set_data(x_vals, gz_vals)

    if len(x_vals) > 1:
        ax.set_xlim(x_vals[0], x_vals[-1])
    elif len(x_vals) == 1:
        ax.set_xlim(x_vals[0] - 1, x_vals[0] + 1)

    all_y = list(gx_vals) + list(gy_vals) + list(gz_vals)
    if all_y:
        y_min = min(all_y)
        y_max = max(all_y)

        if y_min == y_max:
            pad = 1
        else:
            pad = 0.1 * (y_max - y_min)

        ax.set_ylim(y_min - pad, y_max + pad)

    fig.canvas.draw_idle()
    plt.pause(0.001)

# -----------------------------
# Main loop
# -----------------------------
while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip().replace('\r', '')

    if not line:
        continue

    if line == "START":
        recording = True
        data_points = []
        x_vals.clear()
        gx_vals.clear()
        gy_vals.clear()
        gz_vals.clear()
        sample_index = 0
        print("Recording data...")
        continue

    elif line == "END":
        recording = False
        with open(OUTPUT_JSON, 'w') as f:
            json.dump({"sensor_readings": data_points}, f)
        print(f"Done! Saved {len(data_points)} points to {OUTPUT_JSON}")
        break

    elif recording:
        try:
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 4:
                continue

            timestamp = int(parts[0])
            gx_raw = float(parts[1])
            gy_raw = float(parts[2])
            gz_raw = float(parts[3])

            gx = gx_raw - GYRO_BIAS_X
            gy = gy_raw - GYRO_BIAS_Y
            gz = gz_raw - GYRO_BIAS_Z

            data_points.append([timestamp, gx, gy, gz])

            x_vals.append(sample_index)
            gx_vals.append(gx)
            gy_vals.append(gy)
            gz_vals.append(gz)
            sample_index += 1

            # Optional debug print
            print(f"raw: {[timestamp, gx_raw, gy_raw, gz_raw]} -> corrected: {[timestamp, gx, gy, gz]}")

            # Only redraw occasionally
            if sample_index % PLOT_EVERY_N_SAMPLES == 0:
                update_plot()

        except ValueError:
            pass

# Final plot refresh
update_plot()
plt.ioff()
plt.show()

ser.close()