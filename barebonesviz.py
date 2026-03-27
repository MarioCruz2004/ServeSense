import json
import numpy as np
import matplotlib.pyplot as plt

def process_imu_16g_fixed(file_path):
    # 1. Load Data
    with open(file_path, 'r') as f:
        raw_data = json.load(f)
    
    readings = np.array(raw_data['sensor_readings'])
    
    # --- THE CORRECTION STEP ---
    # 1. Convert raw bits to 'gs' by dividing by the 16g scale factor (2048)
    # 2. Convert 'gs' to m/s^2 by multiplying by 9.81
    accel_ms2 = (readings[:, 0:3] / 2048.0) * 9.81
    
    time = readings[:, 6] / 1e6           
    
    # --- BIAS REMOVAL (Essential for "Still" sensors) ---
    # Even at 16g, there's a tiny offset. We subtract the first 10 samples 
    # so the starting velocity is truly zero.
    bias = np.mean(accel_ms2[:10], axis=0)
    accel_final = accel_ms2 - bias
    
    num_samples = len(time)
    positions = np.zeros((num_samples, 3))
    velocities = np.zeros((num_samples, 3))

    for i in range(1, num_samples):
        dt = time[i] - time[i-1]
        if dt <= 0 or dt > 0.5: continue

        # Physics Integration
        velocities[i] = velocities[i-1] + accel_final[i] * dt
        positions[i] = positions[i-1] + velocities[i] * dt

    # 3. Plotting
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Corrected Path')
    ax.set_title("16g Scaled & Bias-Corrected Motion")
    plt.show()

process_imu_16g_fixed('data4.json')