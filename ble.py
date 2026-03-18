import asyncio
import json
from bleak import BleakClient
import struct

# --- YOUR SETTINGS ---
ADDRESS = "C012C31E-9321-BA99-12E1-F6838C322582" 
CHARACTERISTIC_UUID = "00005678-0000-1000-8000-00805f9b34fb"
WRITE_UUID  = "00009ABC-0000-1000-8000-00805f9b34fb"

SAMPLE_FORMAT = '<ffffffL'
SAMPLE_SIZE = struct.calcsize(SAMPLE_FORMAT) # This will be 28

data_points = []
recording = False
stop_event = None

def notification_handler(sender, data):
    global recording, data_points
    
    # 1. Handle Control Signals (Text)
    if len(data) < 10:
        try:
            msg = data.decode('utf-8').strip()
            if msg == "START":
                recording = True
                data_points = []
                print("--- Recording Started ---")
                return
            elif msg == "END":
                recording = False
                with open('data.json', 'w') as f:
                    json.dump({"sensor_readings": data_points}, f)
                print(f"--- Done! Saved {len(data_points)} total samples ---")
                stop_event.set()
                return
        except UnicodeDecodeError:
            pass

    # 2. Handle Multi-Sample Binary Data
    if recording:
        # Determine how many 28-byte samples are in this specific packet
        num_samples = len(data) // SAMPLE_SIZE
        
        if num_samples > 0:
            for i in range(num_samples):
                # Extract a 28-byte slice for one sample
                start = i * SAMPLE_SIZE
                end = start + SAMPLE_SIZE
                sample_bytes = data[start:end]
                
                try:
                    # Unpack this specific slice
                    unpacked = struct.unpack(SAMPLE_FORMAT, sample_bytes)
                    
                    # Add to our main list (matching your original format)
                    data_points.append(list(unpacked))
                except Exception as e:
                    print(f"Error unpacking sample {i}: {e}")
            
            print(f"Processed packet: {num_samples} samples added (Total: {len(data_points)})")

async def main():
    global stop_event
    stop_event = asyncio.Event()
    
    async with BleakClient(ADDRESS) as client:
        print(f"Connected to GATT Server: {client.is_connected}")
        
        # 1. Get both characteristics to check their specific properties
        notify_char = client.services.get_characteristic(CHARACTERISTIC_UUID)
        write_char = client.services.get_characteristic(WRITE_UUID)

        print(f"Notify Prop: {notify_char.properties}")
        print(f"Write Prop: {write_char.properties}")

        if "notify" in notify_char.properties:
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        else:
            print("ERROR: Notify UUID does not support 'Notify'.")
            return

        # 2. Use the WRITE characteristic's properties for the 'g' command
        # Force response=False to prevent the handshake hang
        print(f"Sending 'g' to {WRITE_UUID} (without response)...")
        
        await client.write_gatt_char(WRITE_UUID, b'g', response=False)

        print("Command sent! Waiting for notification handler to trigger...")
        await stop_event.wait()

if __name__ == "__main__":
    asyncio.run(main())