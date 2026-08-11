# ESP32-C3 BLDC PID Controller (BLE)

Bluetooth Low Energy control interface for the BLDC PID controller.

## Features

- All features from `control/` (200Hz PID, soft-start, EMA filtering)
- BLE interface (no WiFi issues)
- Status notifications every 500ms
- Simple command interface

## BLE Interface

**Device Name:** `BLDC-PID`

**Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

### Characteristics

| Characteristic | UUID | Type | Description |
|----------------|------|------|-------------|
| Status | `beb5483e-36e1-4688-b7f5-ea07361b26a8` | Read/Notify | JSON: `run`, `rpm`, `target`, `pwm`, `sens` (gain scale from optional GPIO2/3 trim) |
| Control | `beb5483e-36e1-4688-b7f5-ea07361b26a9` | Write | Send "start" or "stop" |
| Target | `beb5483e-36e1-4688-b7f5-ea07361b26aa` | Read/Write | Set target RPM (e.g. "1440") |

## Usage

### Android (nRF Connect)

1. Install nRF Connect
2. Scan for "BLDC-PID"
3. Connect
4. Find the service and characteristics
5. Enable notifications on Status characteristic
6. Write "start" to Control to start
7. Write "1800" to Target to set 1800 RPM
8. Write "stop" to Control to stop

### iOS (LightBlue)

Same process as Android using LightBlue app.

### Python (bleak)

```python
import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "BLDC-PID"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
STATUS_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CONTROL_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9"
TARGET_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26aa"

async def main():
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    async with BleakClient(device) as client:
        # Start
        await client.write_gatt_char(CONTROL_UUID, b"start")

        # Set target RPM
        await client.write_gatt_char(TARGET_UUID, b"1800")

        # Read status
        status = await client.read_gatt_char(STATUS_UUID)
        print(status.decode())

asyncio.run(main())
```

## Hardware

Same as `control/`: GPIO 0 (RPM with divider if 5V), GPIO 1 (PWM to ESC), GPIO 2 (sensitivity pot ADC), GPIO 3 (pull LOW to enable trim). The pot scales Kp/Ki/Kd only; target RPM is still set over BLE via the Target characteristic (or initial `DEFAULT_TARGET_RPM` until written).
