import serial
import json
import time

# Gyroscope bias measured at rest
GYRO_BIAS_X = -2
GYRO_BIAS_Y = -4
GYRO_BIAS_Z = 2

# 1. Connect to the Feather
ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)
print("3 seconds until start")
time.sleep(3)  # Give the Feather a moment to reset after connecting

# 2. Tell the Feather to start
print("Sending 'g' trigger...")
data_points = []
recording = False

ser.write(b'g')

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip().replace('\r', '')
    #print(f"Cleaned line: '{line}'")

    if line == "START":
        recording = True
        data_points = []
        print("Recording data...")

    elif line == "END":
        recording = False
        with open('data.json', 'w') as f:
            json.dump({"sensor_readings": data_points}, f)
        print(f"Done! Saved {len(data_points)} points to data.json")
        break

    elif recording:
        try:
            # Parse comma-separated values like:
            # "399337890 ,-2 ,-4 ,2"
            parts = [p.strip() for p in line.split(',')]

            # Expecting: timestamp, gx, gy, gz
            if len(parts) != 4:
                continue

            timestamp = int(parts[0])
            gx = float(parts[1])
            gy = float(parts[2])
            gz = float(parts[3])

            # Remove gyroscope bias
            gx_corrected = gx - GYRO_BIAS_X
            gy_corrected = gy - GYRO_BIAS_Y
            gz_corrected = gz - GYRO_BIAS_Z

            data_points.append([timestamp, gx_corrected, gy_corrected, gz_corrected])

            print(
                f"raw: {[timestamp, gx, gy, gz]} -> "
                f"corrected: {[timestamp, gx_corrected, gy_corrected, gz_corrected]}"
            )

        except ValueError:
            pass  # Ignore non-numeric noise