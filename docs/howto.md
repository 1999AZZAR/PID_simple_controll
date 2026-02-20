# Assembly and Setup Guide

## Bill of Materials

### Core Components
*   **Microcontroller**: Arduino Uno, ATtiny85, or ESP32-C3.
*   **Motor**: 3-Phase BLDC with Hall Sensors.
*   **Driver**: PWM-compatible ESC.
*   **Power**:
    *   **Logic**: 5V (USB or Regulator).
    *   **Motor**: As required by motor/ESC (e.g., 12V, 24V).

## Wiring Guide

### 1. Arduino Uno (Development)
| Component | Uno Pin | Notes |
| :--- | :--- | :--- |
| **Hall Signal** | Pin 2 | Interrupt capable |
| **PWM Output** | Pin 9 | To ESC Signal |
| **Mode Switch** | Pin 3 | Connect to GND for Tuning Mode |
| **Pots (Opt)** | A0-A3 | For manual tuning |

### 2. ATtiny85 (Production)
| Component | ATtiny Pin | Physical Pin |
| :--- | :--- | :--- |
| **Hall Signal** | PB3 | Pin 2 |
| **PWM Output** | PB0 | Pin 5 |
| **VCC** | VCC | Pin 8 |
| **GND** | GND | Pin 4 |

### 3. ESP32-C3
**Warning**: Requires 3.3V Logic.
*   **Hall Signal**: Connect to GPIO 0 via Voltage Divider (5V -> 3.3V).
*   **PWM Output**: Connect GPIO 1 to ESC.

## Installation Steps

### Step 1: Software Setup
1.  Install **Arduino IDE**.
2.  Install Board Managers:
    *   **ATtiny**: "ATTinyCore" by Spence Konde.
    *   **ESP32**: "esp32" by Espressif.

### Step 2: Wiring
1.  Connect Hall Sensor Signal to the microcontroller Input pin.
2.  Connect ESC Signal to the Output pin.
3.  **Crucial**: Connect Ground (GND) of Microcontroller, ESC, and Power Supply together.

### Step 3: PID Tuning (Arduino Uno)
1.  Upload `arduino_uno` sketch.
2.  Open **Serial Plotter** (115200 baud).
3.  Set Mode Switch to Tuning (GND).
4.  Adjust Potentiometers:
    *   **Kp**: Increase until motor oscillates, then reduce.
    *   **Ki**: Increase to remove steady error.
    *   **Kd**: Increase slightly to dampen response.
5.  Record values and update `config.h`.

### Step 4: Final Deployment (ATtiny85)
1.  Copy tuned PID values to `attiny85/config.h` (v1) or code (v2).
2.  Connect Arduino Uno as ISP.
3.  Burn Bootloader (**16MHz PLL**).
4.  Upload sketch.
5.  Install chip in final circuit.

## Safety Checklist
*   [ ] Motor secured firmly.
*   [ ] Propeller/Load removed during initial tests.
*   [ ] Voltage levels verified (especially for ESP32).
*   [ ] Emergency power disconnect available.
