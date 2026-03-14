import asyncio
import struct
from bleak import BleakClient, BleakScanner

# 1. Update these to match your Arduino code
TARGET_NAME = "FeatherSS"
# On macOS, use the UUID address found during a scan. On Windows/Linux, use MAC address.
ADDRESS = "C012C31E-9321-BA99-12E1-F6838C322582" 

SERVICE_UUID = "00001234-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "00005678-0000-1000-8000-00805f9b34fb"

# 2. Updated notification handler for binary data
def notification_handler(characteristic, data):
    """
    Unpacks the 28-byte packet:
    6 Floats (4 bytes each) + 1 Uint32 (4 bytes)
    Format '<ffffffI' means: Little-endian, 6 floats, 1 unsigned int
    """
    print(f"Received {len(data)} bytes")
    try:
        # Unpack the binary data
        # f = float (4 bytes), I = unsigned int (4 bytes)
        decoded_data = struct.unpack('<ffffffI', data)
        
        ax, ay, az = decoded_data[0:3]
        gx, gy, gz = decoded_data[3:6]
        timestamp  = decoded_data[6]

        print(f"Time: {timestamp}us | Accel: {ax:.2f}, {ay:.2f}, {az:.2f} | Gyro: {gx:.2f}, {gy:.2f}, {gz:.2f}")
    
    except Exception as e:
        print(f"Failed to unpack data: {e}")

async def run_ble_client():
    print("Scanning for ServeSense...")

    disconnect_event = asyncio.Event()

    def handle_disconnect(client):
        print("Device disconnected unexpectedly.")
        disconnect_event.set()

    try:
        # Prefer scanning by name first, since macOS uses a UUID-style address.
        device = await BleakScanner.find_device_by_name(TARGET_NAME, timeout=10.0)

        # Fallback to the known address if name-based discovery fails.
        if device is None:
            print("Name-based scan failed. Trying known address...")
            device = await BleakScanner.find_device_by_address(ADDRESS, timeout=10.0)

        if device is None:
            print("Device not found. Check if Feather is powered on and advertising.")
            return

        print(f"Attempting to connect to {device.address}...")

        async with BleakClient(
            device,
            disconnected_callback=handle_disconnect,
            services=[SERVICE_UUID],
        ) as client:
            print("Successfully connected!")

            # Give the BLE stack a brief moment to settle before enabling notifications.
            await asyncio.sleep(0.5)

            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
            print("Listening for data... Press Ctrl+C to stop.")

            try:
                await disconnect_event.wait()
            finally:
                if client.is_connected:
                    try:
                        await client.stop_notify(CHARACTERISTIC_UUID)
                    except Exception as e:
                        print(f"Warning: failed to stop notifications cleanly: {e}")

    except asyncio.CancelledError:
        raise
    except Exception as e:
        print(f"Connection failed: {e}")

async def discover():
    print("Scanning for ServeSense advertising data...")
    devices = await BleakScanner.discover(return_adv=True)
    for d, adv in devices.values():
        if d.name == "ServeSense" or d.address == "C012C31E-9321-BA99-12E1-F6838C322582":
            print(f"\nFound target: {d.name} [{d.address}]")
            print(f"Advertised Services: {adv.service_uuids}")

if __name__ == "__main__":
    # asyncio.run(discover()) #Used to discover the device
    try:
        asyncio.run(run_ble_client())
    except KeyboardInterrupt:
        print("\nDisconnected.")