# ESP32-C3 BLDC PID Controller

High-performance PID controller for BLDC motors leveraging ESP32-C3 hardware peripherals, FreeRTOS real-time control, and optional Bluetooth Low Energy interface.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Hardware Setup](#hardware-setup)
- [Firmware Variants](#firmware-variants)
  - [Standard Controller](#1-standard-controller-control)
  - [BLE Controller](#2-ble-controller-control_ble)
  - [Motor Simulator](#3-motor-simulator-simulator)
- [Software Architecture](#software-architecture)
- [Configuration](#configuration)
- [BLE Interface](#ble-interface)
- [Troubleshooting](#troubleshooting)

## Overview

This project implements a production-ready PID controller for BLDC motors on the ESP32-C3 platform. It features hardware-accelerated PWM, interrupt-driven RPM sensing, real-time control loops, and optional wireless control via BLE.

### Key Features

- **200Hz Control Loop**: High-frequency PID updates via dedicated FreeRTOS task
- **Hardware-Accelerated PWM**: ESP32 LEDC peripheral generating smooth 5kHz motor control signals
- **Interrupt-Driven RPM**: High-precision pulse counting using IRAM-resident ISR
- **Sliding Window Filter**: Advanced RPM filtering for stable control input
- **Soft-Start Logic**: 1.5-second power ramp prevents current spikes and mechanical stress
- **FPU Utilization**: Full floating-point PID calculations for maximum precision
- **BLE Control** (optional): Wireless interface for start/stop and parameter adjustment
- **Motor Simulator**: Virtual motor for safe testing and PID tuning

## Project Structure

```
esp32-c3/
├── control/              # Standard PID controller (always-on)
│   ├── control.ino       # Main entry point and control loop
│   ├── config.h          # Hardware pins and motor settings
│   ├── config_common.h   # PID and control loop constants
│   ├── ledc_driver.h     # ESP32 LEDC PWM driver
│   ├── pcnt_driver.h     # Interrupt-based pulse counting
│   ├── pid_common.h      # Floating-point PID algorithm
│   └── rpm_common.h      # RPM filtering utilities
│
├── control_ble/          # BLE-enabled controller (start/stop control)
│   ├── control_ble.ino   # Main entry with BLE callbacks
│   ├── config.h          # Hardware configuration
│   ├── config_common.h   # Shared PID constants
│   ├── ledc_driver.h     # PWM driver
│   ├── pcnt_driver.h     # Pulse counter
│   ├── pid_common.h      # PID algorithm
│   └── rpm_common.h      # RPM filter with sliding window
│
├── simulator/            # Virtual motor for testing
│   └── simulator.ino     # Physics engine and signal generator
│
└── README.md             # This documentation
```

## Quick Start

### For Real Motor

1. **Choose Firmware**: Use `control/` for always-on operation, or `control_ble/` for wireless control
2. **Hardware Setup**: Connect RPM sensor to GPIO 0 (with voltage divider if 5V) and ESC to GPIO 1
3. **Configure**: Review `config.h` and `config_common.h` for motor-specific parameters
4. **Upload**: Flash using Arduino IDE or PlatformIO
5. **Monitor**: Open Serial Monitor at 115200 baud to verify startup

### For Testing Without Motor

1. **Flash Simulator**: Upload `simulator/simulator.ino` to a second ESP32-C3
2. **Flash Controller**: Upload `control/control.ino` to the main ESP32-C3
3. **Connect**: Wire GPIO 1 (controller) to GPIO 0 (simulator), and vice versa
4. **Monitor**: Both boards will communicate as if a real motor is connected

## Hardware Setup

### Required Components

- **ESP32-C3 SuperMini** or any ESP32-C3 development board
- **BLDC Motor** (e.g., 8-pole motor)
- **Electronic Speed Controller (ESC)**
- **Voltage Divider** (if Hall sensor outputs 5V)

### Pin Connections

| Component | ESP32-C3 Pin | Description |
|-----------|--------------|-------------|
| RPM Input | GPIO 0 | Pulse signal from motor Hall sensor |
| PWM Output | GPIO 1 | PWM control signal to ESC |
| Power | 5V / VIN | 5V power supply |
| Ground | GND | Common ground |

### Voltage Divider for 5V Hall Sensors

The ESP32-C3 GPIOs are NOT 5V tolerant. If your Hall sensor outputs 5V, you MUST use a voltage divider:

```
5V Hall Signal ----[ R1: 2.2kΩ ]---- GPIO 0
                             |
                             +----[ R2: 3.3kΩ ]---- GND
```

**Calculation**: Vout = 5V × (3.3kΩ / (2.2kΩ + 3.3kΩ)) = 3V (safe for ESP32)

If your sensor already outputs 3.3V logic, connect directly to GPIO 0.

## Firmware Variants

### 1. Standard Controller (`control/`)

**Use Case**: Production environments requiring continuous motor control without external intervention.

**Behavior**: 
- Motor control starts immediately on power-up
- No start/stop commands needed
- Minimal overhead, maximum performance

**Features**:
- 200Hz PID control loop
- Hardware PWM output (5kHz)
- Interrupt-driven RPM sensing
- Sliding window filter for stable RPM readings
- Soft-start ramp (1.5s)

**Upload**: Flash `control/control.ino`

### 2. BLE Controller (`control_ble/`)

**Use Case**: Applications requiring wireless control, remote monitoring, or dynamic parameter adjustment.

**Behavior**:
- Motor control via BLE commands (start/stop)
- Real-time status updates every 500ms
- Thread-safe shared state with spinlock protection

**Additional Features**:
- All features from standard controller
- BLE interface (no WiFi interference)
- Remote RPM target adjustment
- JSON status notifications

**Upload**: Flash `control_ble/control_ble.ino`

### 3. Motor Simulator (`simulator/`)

**Use Case**: Safe PID tuning and testing without real motor hardware.

**Behavior**:
- Receives PWM input from controller
- Simulates motor physics (inertia, acceleration)
- Outputs RPM pulses back to controller

**Features**:
- Configurable max RPM and inertia
- Optional noise injection for filter testing
- Safe bench testing environment
- Serial debug output

**Hardware**: Requires two ESP32-C3 boards connected GPIO-to-GPIO

**Upload**: Flash `simulator/simulator.ino` to second board

## Software Architecture

### Control Loop Architecture

The controller runs in a dedicated FreeRTOS task with highest priority (`configMAX_PRIORITIES - 1`), ensuring strict 5ms (200Hz) timing regardless of other system activities.

```
┌─────────────────────────────────────┐
│   Main Loop (Low Priority)          │
│   - Serial communication             │
│   - BLE status updates (if enabled)  │
└─────────────────────────────────────┘
           ↓ (yields CPU)
┌─────────────────────────────────────┐
│   Control Task (Highest Priority)   │
│   ┌─────────────────────────────┐   │
│   │ 1. Read RPM (pcnt_driver)   │   │
│   │ 2. Filter RPM (sliding win) │   │
│   │ 3. Compute PID               │   │
│   │ 4. Map to PWM                │   │
│   │ 5. Apply Soft-Start          │   │
│   │ 6. Output PWM (ledc_driver)  │   │
│   └─────────────────────────────┘   │
│   vTaskDelayUntil(5ms)               │
└─────────────────────────────────────┘
```

### Core Drivers

**`pcnt_driver.h`**
- Interrupt-driven pulse counting (ESP32-C3 has no hardware PCNT)
- Minimum integration window (40ms default) for low quantization noise
- At 1440 RPM @ 4 PPR: ~4 pulses per sample vs ~0.5 with 5ms window

**`ledc_driver.h`**
- Configures ESP32 LEDC peripheral for 5kHz PWM
- 8-bit resolution (0-255) for smooth control
- Direct hardware output, no software overhead

**`pid_common.h`**
- Floating-point PID implementation
- Anti-windup with integral clamping
- Low-pass filtered derivative to reduce noise amplification

**`rpm_common.h`**
- Sliding window (24 samples) with IIR smoothing
- Decoupled from control period for stable averaging
- Real-time filtering with low latency

## Configuration

### Motor Settings (`config.h`)

```cpp
#define RPM_INPUT_PIN   0   // Hall sensor input
#define PWM_OUTPUT_PIN  1   // ESC control output
#define PULSES_PER_REV  4   // Set to motor pole count / 2
#define RPM_SAMPLE_MIN_US 40000  // 40ms integration window
```

**PULSES_PER_REV**: For an 8-pole motor, use 4. Adjust based on your motor specifications.

**RPM_SAMPLE_MIN_US**: Minimum time to accumulate pulses before computing RPM. Longer = less fluctuation, higher latency. 40000 (40ms) yields ~4 pulses at 1440 RPM.

### PID Tuning (`config_common.h`)

```cpp
#define DEFAULT_TARGET_RPM 1440.0  // Target speed
#define DEFAULT_KP         0.150   // Proportional gain
#define DEFAULT_KI         0.080   // Integral gain
#define DEFAULT_KD         0.015   // Derivative gain
```

**Tuning Guidelines**:
1. Start with Kp only (set Ki and Kd to 0)
2. Increase Kp until oscillation occurs, then reduce by 30%
3. Add Ki slowly to eliminate steady-state error
4. Add small Kd if overshoot occurs

### Control Parameters

```cpp
#define CONTROL_LOOP_HZ     200    // 200Hz update rate
#define PWM_MIN_THRESHOLD   45     // Minimum PWM for motor start
#define SOFT_START_DURATION_MS 1500 // Ramp-up time
```

### PWM Limits

```cpp
#define PWM_MIN_VALUE       0      // 0% throttle
#define PWM_MAX_VALUE       255    // 100% throttle
```

## BLE Interface

**Available in**: `control_ble/` only

### Connection Details

- **Device Name**: `BLDC-PID`
- **Service UUID**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

### Characteristics

| Name | UUID | Type | Description |
|------|------|------|-------------|
| Status | `beb5483e-36e1-4688-b7f5-ea07361b26a8` | Read/Notify | JSON status: `{"run":1,"rpm":1440,"target":1440,"pwm":128}` |
| Control | `beb5483e-36e1-4688-b7f5-ea07361b26a9` | Write | Commands: `"start"` or `"stop"` |
| Target | `beb5483e-36e1-4688-b7f5-ea07361b26aa` | Read/Write | Set target RPM: `"1800"` |

### Mobile Control (nRF Connect / LightBlue)

1. Install nRF Connect (Android) or LightBlue (iOS)
2. Scan for `BLDC-PID` device
3. Connect to device
4. Navigate to service `4fafc201-...`
5. Enable notifications on Status characteristic
6. Write `start` to Control characteristic
7. Write `1800` to Target characteristic to set 1800 RPM
8. Write `stop` to Control characteristic to stop

### Python Control (bleak)

```python
import asyncio
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "BLDC-PID"
CONTROL_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9"
TARGET_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26aa"
STATUS_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

async def control_motor():
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    
    async with BleakClient(device) as client:
        await client.write_gatt_char(CONTROL_UUID, b"start")
        await client.write_gatt_char(TARGET_UUID, b"1800")
        
        status = await client.read_gatt_char(STATUS_UUID)
        print(f"Status: {status.decode()}")
        
        await asyncio.sleep(10)
        await client.write_gatt_char(CONTROL_UUID, b"stop")

asyncio.run(control_motor())
```

## Troubleshooting

### No RPM Signal

**Symptoms**: RPM reading stays at 0, motor doesn't respond

**Solutions**:
- Verify voltage divider connections (if using 5V sensor)
- Check Hall sensor power supply
- Confirm `PULSES_PER_REV` matches motor specifications
- Test Hall sensor output with oscilloscope or multimeter
- Ensure GND is common between ESP32 and sensor

### Motor Stutters on Start

**Symptoms**: Motor jerks, fails to spin smoothly at startup

**Solutions**:
- Increase `PWM_MIN_THRESHOLD` in `config_common.h` (try 60-80)
- Extend `SOFT_START_DURATION_MS` to 2000-3000ms
- Check ESC calibration (some ESCs need throttle range calibration)
- Verify power supply can handle startup current

### Unstable Speed / Oscillation

**Symptoms**: RPM fluctuates significantly, motor speed hunts

**Solutions**:
- Reduce `DEFAULT_KP` by 30-50% (too much proportional gain)
- Decrease `DEFAULT_KD` if high-frequency oscillation occurs
- Reduce `DEFAULT_KI` if slow oscillation with overshoot
- Check for electrical noise on RPM input (add 0.1µF capacitor near GPIO 0)
- Verify stable power supply (voltage drops cause instability)

### BLE Connection Issues

**Symptoms**: Cannot find device or connection drops

**Solutions**:
- Ensure BLE is enabled on mobile device
- Move closer to ESP32 (BLE range is limited)
- Restart ESP32 if device name doesn't appear
- Check Serial Monitor for BLE initialization errors
- Some phones limit simultaneous BLE connections (disconnect other devices)

### Incorrect RPM Reading

**Symptoms**: RPM reading is 2x or 0.5x expected value

**Solutions**:
- Adjust `PULSES_PER_REV` in `config.h`
- For 8-pole motor: typically 4 pulses/rev
- For 12-pole motor: typically 6 pulses/rev
- Use tachometer to verify actual RPM and calibrate accordingly

### Motor Runs Away (Overspeeds)

**Symptoms**: Motor accelerates beyond target RPM, doesn't stabilize

**Solutions**:
- IMMEDIATE: Power off system
- Check wiring polarity (RPM input might be reading inverted signal)
- Verify PID output isn't inverted (`DEFAULT_KP` should be positive)
- Reduce all PID gains and retune from scratch
- Add `PID_OUTPUT_MAX` limit in `config_common.h`

### Serial Monitor Shows No Output

**Symptoms**: No debug messages appear after upload

**Solutions**:
- Verify Serial Monitor baud rate is 115200
- Check USB cable supports data (not power-only)
- Try different USB port or cable
- Some ESP32-C3 boards require manual reset after upload
- Comment out `Serial.begin()` if not needed for production

## Performance Notes

### Timing Accuracy

The FreeRTOS task scheduler guarantees 5ms ±1µs control loop timing under normal conditions. Serial debug output may add jitter if used excessively.

### Memory Usage

- Standard controller: ~30KB flash, ~4KB RAM
- BLE controller: ~580KB flash, ~35KB RAM (BLE stack overhead)
- Simulator: ~25KB flash, ~3KB RAM

### Power Consumption

- Idle (no motor): ~40mA @ 3.3V
- Active control: ~60mA @ 3.3V
- BLE active: ~80mA @ 3.3V (advertising + notifications)

## Safety Considerations

1. **Voltage Protection**: Always use voltage divider for 5V Hall sensors
2. **Power Supply**: Ensure adequate current capacity for ESP32 and ESC
3. **Thermal**: Monitor ESC temperature during extended high-speed operation
4. **Emergency Stop**: Keep power disconnection easily accessible
5. **Testing**: Use simulator before connecting real motor

## License

This project is provided as-is for educational and commercial use.

## Contributing

Improvements welcome. Focus areas:
- Advanced filtering algorithms
- Auto-tuning PID implementation
- Additional motor types support
- Performance optimizations
