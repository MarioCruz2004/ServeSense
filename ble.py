import asyncio
import json
from bleak import BleakClient
import struct
import sys

# ---OUR SETTINGS ---
ADDRESS = "C012C31E-9321-BA99-12E1-F6838C322582" 
CHARACTERISTIC_UUID = "00005678-0000-1000-8000-00805f9b34fb"
WRITE_UUID  = "00009ABC-0000-1000-8000-00805f9b34fb"

SAMPLE_FORMAT = '<ffffffL'
SAMPLE_SIZE = struct.calcsize(SAMPLE_FORMAT)

data_points = []
recording = False

file_index = 1
MAX_FILES = 10
records_completed = 0  # Tracks total successful saves

def notification_handler(sender, data):
    global recording, data_points, file_index, records_completed
    
    if len(data) < 10:
        try:
            msg = data.decode('utf-8').strip()
            if msg == "START":
                recording = True
                data_points = []
                print(f"\n[BLE] --- Recording Started (Target: data{file_index}.json) ---")
                return

            elif msg == "END":
                recording = False
                
                # Construct the filename dynamically
                filename = f"data{file_index}.json"
                
                with open(filename, 'w') as f:
                    json.dump({"sensor_readings": data_points}, f)
                
                print(f"\n[BLE] --- Saved {len(data_points)} samples to {filename} ---")
                
                # Increment the index for next time (1 to 10)
                # (file_index % 10) returns 0-9, so we add 1 to get 1-10
                file_index = (file_index % MAX_FILES) + 1
                records_completed += 1
                
                print(f"[SYSTEM] Next recording will save to data{file_index}.json")
                return
        except UnicodeDecodeError:
            pass
    # 2. Handle Binary Data
    if recording:
        num_samples = len(data) // SAMPLE_SIZE
        for i in range(num_samples):
            start = i * SAMPLE_SIZE
            sample_bytes = data[start : start + SAMPLE_SIZE]
            try:
                unpacked = struct.unpack(SAMPLE_FORMAT, sample_bytes)
                data_points.append(list(unpacked))
            except Exception as e:
                print(f"Unpack Error: {e}")
        print(f"[BLE] Received {num_samples} samples (Total: {len(data_points)})", end='\r')

async def user_input_loop(client):
    """Loop to handle keyboard commands without blocking BLE."""
    print("\n--- Command Menu ---")
    print("Type 'g' to Request Data")
    print("Type 'c' to Calibrate Gyroscope")
    print("Type 'a' to Calibrate Accelerometer (Not Needed To Be Done By User)")
    print("Type 's' : Buffer Status | Ctrl+C : Exit")
    print("Press Ctrl+C to Exit")
    print("--------------------")

    loop = asyncio.get_event_loop()
    
    while True:
        # This allows the BLE notify handler to keep running while we wait for typing
        cmd = await loop.run_in_executor(None, sys.stdin.readline)
        cmd = cmd.strip().lower()

        if cmd == 'g':
            print("Sending 'g' (Request Data)...")
            await client.write_gatt_char(WRITE_UUID, b'g', response=False)
        elif cmd == 'c':
            print("Sending 'c' (Calibrate Gryoscope)...")
            await client.write_gatt_char(WRITE_UUID, b'c', response=False)
        elif cmd == 'a':
            print("Sending 'a' (Calibrate Accelerometer)...")
            await client.write_gatt_char(WRITE_UUID, b'a', response=False)
        elif cmd == 's':
            # Display Status
            status = "Ready" if not recording else "RECORDING"
            print(f"\n--- SYSTEM STATUS ---")
            print(f"Connection:  Connected")
            print(f"State:       {status}")
            print(f"Next File:   data{file_index}.json")
            print(f"Total Saves: {records_completed}")
            if records_completed >= MAX_FILES:
                print("Buffer Note: Ring buffer is full; oldest files are being overwritten.")
            print("----------------------")
        elif cmd == '':
            continue
        else:
            print(f"Unknown command: {cmd}")

async def main():
    try:
        async with BleakClient(ADDRESS) as client:
            print(f"Connected: {client.is_connected}")

            # Start notifications immediately
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
            
            # Start the interactive loop
            await user_input_loop(client)

    except asyncio.CancelledError:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # exit on Ctrl+C
        print("\nProgram closed by user.")
        sys.exit(0)
