import serial
import json
import time
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# -----------------------------
# Configuration
# -----------------------------
PORT = '/dev/cu.usbmodem1101'
BAUD = 115200

GYRO_BIAS_X = -2
GYRO_BIAS_Y = -4
GYRO_BIAS_Z = 2

OUTPUT_JSON = 'data.json'
PLOT_EVERY_N_SAMPLES = 5

# Set this depending on your gyro units from the Feather.
# If your gyro data is already in deg/s, leave this True.
# If it is in rad/s, set to False.
GYRO_UNITS_ARE_DEG_PER_SEC = True

# -----------------------------
# Helpers
# -----------------------------
def rot_x(a):
    ca, sa = math.cos(a), math.sin(a)
    return [
        [1, 0, 0],
        [0, ca, -sa],
        [0, sa, ca]
    ]

def rot_y(a):
    ca, sa = math.cos(a), math.sin(a)
    return [
        [ca, 0, sa],
        [0, 1, 0],
        [-sa, 0, ca]
    ]

def rot_z(a):
    ca, sa = math.cos(a), math.sin(a)
    return [
        [ca, -sa, 0],
        [sa, ca, 0],
        [0, 0, 1]
    ]

def matmul(A, B):
    out = [[0.0] * len(B[0]) for _ in range(len(A))]
    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(B)):
                out[i][j] += A[i][k] * B[k][j]
    return out

def apply_rot(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]

def euler_to_matrix(roll, pitch, yaw):
    # Z-Y-X convention: yaw, then pitch, then roll
    Rz = rot_z(yaw)
    Ry = rot_y(pitch)
    Rx = rot_x(roll)
    return matmul(matmul(Rz, Ry), Rx)

# -----------------------------
# Serial setup
# -----------------------------
ser = serial.Serial(PORT, BAUD, timeout=1)
print("3 seconds until start")
time.sleep(3)
ser.reset_input_buffer()

print("Sending 'g' trigger...")
ser.write(b'g')

# -----------------------------
# Data storage
# -----------------------------
data_points = []
recording = False
sample_index = 0

# Orientation state
roll = 0.0
pitch = 0.0
yaw = 0.0
last_timestamp = None

# -----------------------------
# Plot setup
# -----------------------------
plt.ion()
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

def update_orientation_plot(roll, pitch, yaw):
    ax.cla()

    R = euler_to_matrix(roll, pitch, yaw)

    origin = [0, 0, 0]
    x_axis = apply_rot(R, [1, 0, 0])
    y_axis = apply_rot(R, [0, 1, 0])
    z_axis = apply_rot(R, [0, 0, 1])

    ax.quiver(*origin, *x_axis, length=1.0, normalize=True)
    ax.quiver(*origin, *y_axis, length=1.0, normalize=True)
    ax.quiver(*origin, *z_axis, length=1.0, normalize=True)

    ax.set_xlim([-1.2, 1.2])
    ax.set_ylim([-1.2, 1.2])
    ax.set_zlim([-1.2, 1.2])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(
        f"IMU Orientation\n"
        f"roll={math.degrees(roll):.1f}°, "
        f"pitch={math.degrees(pitch):.1f}°, "
        f"yaw={math.degrees(yaw):.1f}°"
    )

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
        sample_index = 0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        last_timestamp = None
        print("Recording data...")
        update_orientation_plot(roll, pitch, yaw)
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

            if last_timestamp is not None:
                dt = (timestamp - last_timestamp) / 1_000_000.0  # assumes microseconds

                if dt > 0:
                    if GYRO_UNITS_ARE_DEG_PER_SEC:
                        gx_rad = math.radians(gx)
                        gy_rad = math.radians(gy)
                        gz_rad = math.radians(gz)
                    else:
                        gx_rad = gx
                        gy_rad = gy
                        gz_rad = gz

                    # Simple integration
                    roll += gx_rad * dt
                    pitch += gy_rad * dt
                    yaw += gz_rad * dt

                    if sample_index % PLOT_EVERY_N_SAMPLES == 0:
                        update_orientation_plot(roll, pitch, yaw)

            last_timestamp = timestamp
            sample_index += 1

        except ValueError:
            pass

update_orientation_plot(roll, pitch, yaw)
plt.ioff()
plt.show()
ser.close()