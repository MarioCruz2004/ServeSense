# Live IMU Orientation Visualizer
#
# Reads a continuous stream of gyroscope data from a Feather microcontroller
# over USB serial (COM5, 115200 baud) and displays a live 3D orientation of
# the IMU in a pyqtgraph OpenGL window, updating at 60fps.
#
# Architecture:
#   - A background thread (threading) handles all serial I/O via pyserial,
#     parsing CSV lines (timestamp, gx, gy, gz) and integrating gyro rates
#     over time (dt from microsecond timestamps) to track roll, pitch, and yaw.
#   - The main thread runs a PyQt5 GUI with a pyqtgraph GLViewWidget, refreshing
#     a set of 3D axis arrows at 60fps via a QTimer. A threading.Lock protects
#     the shared orientation state between the two threads.
#
# Controls (visualization window must be focused):
#   Q / Escape  — quit
#   R           — reset orientation to zero
#
# Dependencies: pyserial, pyqtgraph, PyQt5, PyOpenGL, numpy


import sys
from PyQt5 import QtWidgets
app = QtWidgets.QApplication(sys.argv)  # must be before any other pyqtgraph imports

import serial
import math
import time
import threading

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtCore

# -----------------------------
# Configuration
# -----------------------------
PORT = 'COM5'
BAUD = 115200

GYRO_BIAS_X = -2
GYRO_BIAS_Y = -4
GYRO_BIAS_Z = 2

GYRO_UNITS_ARE_DEG_PER_SEC = True

DISPLAY_FPS = 60

# -----------------------------
# Shared orientation state
# -----------------------------
lock = threading.Lock()
roll  = 0.0
pitch = 0.0
yaw   = 0.0
last_timestamp = None

# -----------------------------
# Rotation helpers
# -----------------------------
def euler_to_matrix(r, p, y_):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y_), math.sin(y_)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ]
    ], dtype=float)

# -----------------------------
# Serial reader thread
# -----------------------------
def serial_reader():
    global roll, pitch, yaw, last_timestamp

    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    print("Connected. Streaming...")

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 4:
                continue

            timestamp = int(parts[0])
            gx = float(parts[1]) # - GYRO_BIAS_X
            gy = float(parts[2]) # - GYRO_BIAS_Y
            gz = float(parts[3]) # - GYRO_BIAS_Z

            # print every 1000th sample (adjust as needed)
            # printing every single sample was too m,uch to read, 
            if timestamp%1000 == 0:  
                print(f"Data: ts={timestamp} gx={gx:.2f} gy={gy:.2f} gz={gz:.2f}")

            with lock:
                if last_timestamp is not None:
                    dt = (timestamp - last_timestamp) / 1_000_000.0
                    if 0 < dt < 0.1:  # sanity check: ignore huge gaps on reconnect
                        if GYRO_UNITS_ARE_DEG_PER_SEC:
                            gx_rad = math.radians(gx)
                            gy_rad = math.radians(gy)
                            gz_rad = math.radians(gz)
                        else:
                            gx_rad, gy_rad, gz_rad = gx, gy, gz

                        roll  += gx_rad * dt
                        pitch += gy_rad * dt
                        yaw   += gz_rad * dt

                last_timestamp = timestamp

        except ValueError:
            continue
        except Exception as e:
            print(f"Serial thread error: {e}")
            break

    ser.close()
    print("Serial port closed.")

# -----------------------------
# PyQtGraph 3D visualizer
# -----------------------------
class OrientationWidget(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('IMU Live Orientation  |  Q to quit  |  R to reset orientation')
        self.resize(800, 800)
        self.setCameraPosition(distance=4, elevation=25, azimuth=45)

        # Grid for reference
        grid = gl.GLGridItem()
        grid.setSize(4, 4)
        grid.setSpacing(0.5, 0.5)
        self.addItem(grid)

        # Three axis arrows: X=red, Y=green, Z=blue
        self.x_arrow = self._make_arrow([1, 0, 0], (1.0, 0.2, 0.2, 1.0))
        self.y_arrow = self._make_arrow([0, 1, 0], (0.2, 1.0, 0.2, 1.0))
        self.z_arrow = self._make_arrow([0, 0, 1], (0.2, 0.5, 1.0, 1.0))

        # Static faint reference axes
        self._add_reference_axes()

        # Timer drives GUI updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(int(1000 / DISPLAY_FPS))

    def _make_arrow(self, direction, color):
        pts = np.array([[0, 0, 0], direction], dtype=float)
        item = gl.GLLinePlotItem(pos=pts, color=color, width=4, antialias=True)
        self.addItem(item)
        return item

    def _add_reference_axes(self):
        for vec, col in [
            ([1, 0, 0], (1.0, 0.2, 0.2, 0.2)),
            ([0, 1, 0], (0.2, 1.0, 0.2, 0.2)),
            ([0, 0, 1], (0.2, 0.5, 1.0, 0.2)),
        ]:
            pts = np.array([[0, 0, 0], vec], dtype=float)
            item = gl.GLLinePlotItem(pos=pts, color=col, width=1, antialias=True)
            self.addItem(item)

    def update_frame(self):
        with lock:
            r, p, y_ = roll, pitch, yaw

        R = euler_to_matrix(r, p, y_)

        x_end = R @ np.array([1.0, 0.0, 0.0])
        y_end = R @ np.array([0.0, 1.0, 0.0])
        z_end = R @ np.array([0.0, 0.0, 1.0])

        origin = np.array([0.0, 0.0, 0.0])

        self.x_arrow.setData(pos=np.array([origin, x_end]))
        self.y_arrow.setData(pos=np.array([origin, y_end]))
        self.z_arrow.setData(pos=np.array([origin, z_end]))

        self.setWindowTitle(
            f"IMU Live Orientation  |  Q to quit  |  R to reset  |  "
            f"roll={math.degrees(r):.1f}°  "
            f"pitch={math.degrees(p):.1f}°  "
            f"yaw={math.degrees(y_):.1f}°"
        )

    def keyPressEvent(self, event):
        if event.key() in (QtCore.Qt.Key_Q, QtCore.Qt.Key_Escape):
            QtWidgets.QApplication.quit()
        elif event.key() == QtCore.Qt.Key_R:
            global roll, pitch, yaw, last_timestamp
            with lock:
                roll, pitch, yaw = 0.0, 0.0, 0.0
                last_timestamp = None

# -----------------------------
# Entry point
# -----------------------------
if __name__ == '__main__':
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    widget = OrientationWidget()
    widget.show()
    sys.exit(app.exec_())
