import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import pandas as pd

def process_swing(json_file):
    # 1. Load Data
    with open(json_file, 'r') as f:
        raw_data = json.load(f)
    
    data = np.array(raw_data["sensor_readings"])
    timestamps = data[:, 0]
    gyro = data[:, 1:4]  # Expected in degrees/s
    accel = data[:, 4:7] # Expected in G-units
    
    # 2. Time Conversion (Microseconds to Seconds)
    t_sec = (timestamps - timestamps[0]) / 1_000_000.0
    dt = np.diff(t_sec, prepend=0)
    
    # # 3. Calibration
    # # We use the first 50 samples to find the "zero" of the gyroscope
    # # and the direction of gravity.
    N_cal = min(5, len(data))
    # gyro_bias = np.mean(gyro[:N_cal], axis=0)
    # gyro_calibrated = gyro - gyro_bias
    gyro_rad = np.radians(gyro) # Convert to Rad/s for math
    
    # 4. Integration Loop
    current_q = R.identity()
    positions = []
    velocity = np.array([0.0, 0.0, 0.0])
    position = np.array([0.0, 0.0, 0.0])
    
    # Identify initial gravity vector
    # If magnitude is > 0.5G, we assume gravity is present and subtract it
    avg_accel = np.mean(accel[:N_cal], axis=0)
    has_gravity = np.linalg.norm(avg_accel) > 0.5
    gravity_vec = np.array([0, 0, 1.0]) if has_gravity else np.array([0, 0, 0])

    for i in range(len(data)):
        # Update orientation quaternion
        if i > 0:
            # Calculate how much we rotated since the last frame
            step_rotation = R.from_rotvec(gyro_rad[i] * dt[i])
            current_q = current_q * step_rotation
        
        # Transform accelerometer reading from local racket frame to world frame
        accel_world = current_q.apply(accel[i])
        
        # Remove gravity and convert G to m/s^2
        linear_accel = (accel_world - gravity_vec) * 9.81
        
        # Double integrate (Acceleration -> Velocity -> Position)
        if i > 0:
            velocity += linear_accel * dt[i]
            position += velocity * dt[i]
        
        positions.append(position.copy())
    
    return np.array(positions)

# --- Execution ---
path_coords = process_swing('data5.json')

# 5. Plotting
print("IMU initial graphing attempt...")
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(path_coords[:, 0], path_coords[:, 1], path_coords[:, 2], lw=2, label='Swing Path')
ax.set_title("3D Swing Path Reconstruction")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.set_zlabel("Z (meters)")
plt.legend()
plt.show()