import asyncio
from bleak import BleakScanner, BleakClient

TARGET_NAME = "Feather" # Make sure this matches your Feather's advertised name

async def find_and_connect():
    print(f"Searching for devices named '{TARGET_NAME}'...")
    
    # 1. Scan for 5 seconds
    devices = await BleakScanner.discover()
    
    target_device = None
    for d in devices:
        # We check if 'Feather' is in the name (case-insensitive)
        if d.name and TARGET_NAME.lower() in d.name.lower():
            target_device = d
            break

    if target_device:
        print(f"Found it!")
        print(f"Name: {target_device.name}")
        print(f"Current macOS UUID: {target_device.address}")
        print("-" * 30)
        
        # 2. Try to connect using the found device
        async with BleakClient(target_device) as client:
            if client.is_connected:
                print("Successfully connected!")
                # Once connected, you can proceed with your notifications here
            else:
                print("Found the device, but failed to connect.")
    else:
        print(f"Could not find any device named '{TARGET_NAME}'.")
        print("Available devices nearby:")
        for d in devices:
            print(f" - {d.name}: {d.address}")

if __name__ == "__main__":
    asyncio.run(find_and_connect())