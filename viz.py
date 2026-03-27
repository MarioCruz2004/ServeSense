import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# --- Tuning Parameters ---
ACCEL_THRESHOLD = 0.15     # m/s^2: threshold for "true" motion
STILLNESS_WINDOW = 5       # Number of samples to check for variance
VELOCITY_DAMPING = 0.98    # Global "friction" to stop infinite gliding

def process_imu_json(file_path):
    # 1. Load Data
    with open(file_path, 'r') as f:
        raw_data = json.load(f)
    
    readings = np.array(raw_data['sensor_readings'])
    
    # Extract & Convert Units (Note: we keep the 9.81 scaling for now)
    accel_raw = readings[:, 0:3] * 9.81   
    gyro_raw = np.radians(readings[:, 3:6])
    time = readings[:, 6] / 1e6           
    
    num_samples = len(time)
    num_cal = 50

    # --- DYNAMIC INITIAL ALIGNMENT ---
    if num_samples > num_cal:
        avg_accel = np.mean(accel_raw[:num_cal], axis=0)
        avg_gyro = np.mean(gyro_raw[:num_cal], axis=0)
        
        # The sensor's 'Down' vector during calibration
        measured_gravity_vec = avg_accel
        measured_gravity_mag = np.linalg.norm(measured_gravity_vec)
        
        # WE DEFINE THE WORLD: 
        # Create a rotation that maps the measured gravity vector to the global [0, 0, 1]
        # This handles your "sensor pointed down" or "tilted" start automatically.
        initial_gravity_dir = measured_gravity_vec / measured_gravity_mag
        # align_vectors returns the rotation and the residual error
        current_rot, _ = R.align_vectors([[0, 0, 1]], [initial_gravity_dir])
        
        gyro_bias = avg_gyro
        # Use the actual magnitude measured by the sensor as our global constant
        gravity_global = np.array([0, 0, measured_gravity_mag])
        
        print(f"Initial Gravity Magnitude: {measured_gravity_mag:.4f} m/s^2")
        print(f"Initial Orientation Aligned to Gravity Vector: {avg_accel}")
    else:
        current_rot = R.identity()
        gyro_bias = np.zeros(3)
        gravity_global = np.array([0, 0, 9.81])

    positions = np.zeros((num_samples, 3))
    velocities = np.zeros((num_samples, 3))

    # Header for Debug Monitor
    print(f"\n{'Time':>8} | {'Resid. Accel X':>14} | {'Y':>8} | {'Z':>8} | Status")
    print("-" * 60)

    for i in range(1, num_samples):
        dt = time[i] - time[i-1]
        if dt <= 0 or dt > 0.2: continue 

        # --- UPDATE ORIENTATION ---
        clean_gyro = gyro_raw[i] - gyro_bias
        delta_rot = R.from_rotvec(clean_gyro * dt)
        current_rot = current_rot * delta_rot

        # --- GRAVITY REMOVAL ---
        # Rotate the raw acceleration into our gravity-aligned world frame
        accel_world = current_rot.apply(accel_raw[i])
        # Subtract the gravity vector we established at the start
        linear_accel = accel_world - gravity_global

        # --- STILLNESS DETECTOR ---
        if i > STILLNESS_WINDOW:
            recent_accels = accel_raw[i-STILLNESS_WINDOW:i]
            accel_mags = np.linalg.norm(recent_accels, axis=1)
            accel_std = np.std(accel_mags)
            # Use linear_accel to see if we're actually moving in our world frame
            is_still = (accel_std < 0.05) and (np.linalg.norm(linear_accel) < ACCEL_THRESHOLD)
        else:
            is_still = False

        # --- DRIFT DEBUG MONITOR ---
        if i < 150 and i % 20 == 0:
            status = "STILL" if is_still else "MOVE"
            print(f"{time[i]-time[0]:8.2f} | {linear_accel[0]:14.4f} | {linear_accel[1]:8.4f} | {linear_accel[2]:8.4f} | {status}")

        # --- INTEGRATION ---
        if is_still:
            velocities[i] = np.array([0.0, 0.0, 0.0])
        else:
            velocities[i] = (velocities[i-1] + linear_accel * dt) * VELOCITY_DAMPING

        positions[i] = positions[i-1] + velocities[i] * dt

    # 3. Plotting
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Filtered Path', lw=2)
    ax.scatter(0, 0, 0, color='green', s=100, label='Start')
    
    ax.set_title("IMU Motion Reconstruction (Dynamic Alignment + ZUPT)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    plt.show()

# Run the visualizer
process_imu_json('data2.json')