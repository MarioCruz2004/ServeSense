import serial
import json
import time

# 1. Connect to the Feather
ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1) 
time.sleep(5) # Give the Feather a moment to reset after connecting

# 2. Tell the Feather to start
print("Sending 'g' trigger...")
data_points = []
recording = False

ser.write(b'g') 

while True:
    # line = ser.readline().decode('utf-8').strip()
    line = ser.readline().decode('utf-8').strip().replace('\r', '')
    print(f"Cleaned line: '{line}'")
    
    if line == "START":
        recording = True
        data_points = []
        print("Recording data...")
    elif line == "END":
        recording = False
        # Save to JSON file
        with open('data.json', 'w') as f:
            # json.dump({"sensor_readings": data_points}, f, indent=4)
            json.dump({"sensor_readings": data_points}, f)
        print(f"Done! Saved {len(data_points)} points to data.json")
        break
    elif recording:
        try:
            # 1. Split the line by spaces: ["43484375", "-1.00", "-0.02", ...]
            parts = line.split() 
            
            # 2. Convert all parts to floats and put them in a list
            numeric_parts = [float(p) for p in parts]
            
            # 3. Add this group of numbers to our main data list
            data_points.append(numeric_parts)

        except ValueError:
            pass # Ignore non-numeric noise