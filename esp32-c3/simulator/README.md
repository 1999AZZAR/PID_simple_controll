# ESP32-C3 BLDC Motor Simulator

This project turns a second ESP32-C3 into a **Virtual Motor & ESC**. It allows you to safely test your main PID controller without connecting a real high-power motor or ESC.

## Why use this?
*   **Safe**: No high voltage, no spinning blades, no magic smoke.
*   **Debug**: See exactly what the PID controller is sending (PWM) and receiving (RPM).
*   **Tune**: Adjust the "motor physics" (inertia, max RPM) to stress-test your PID tuning.

## Hardware Setup
You need **two** ESP32-C3 boards.

1.  **Controller Board**: Flashed with the main `esp32_c3` firmware.
2.  **Simulator Board**: Flashed with this `esp32_c3_simulator` firmware.

### Wiring (Direct Connection)
Connect the two boards directly. Since both use 3.3V logic, **no voltage divider is needed**.

| Controller (Main) | Simulator (This Board) | Function |
| :--- | :--- | :--- |
| **GND** | **GND** | Common Ground |
| **GPIO 1** (PWM Output) | **GPIO 0** (PWM Input) | Controller sends throttle signal |
| **GPIO 0** (RPM Input) | **GPIO 1** (RPM Output) | Simulator sends Hall sensor pulses |

## Configuration
Inside `esp32_c3_simulator.ino`, you can tweak the physics:

*   `MAX_RPM`: Top speed at 100% PWM (default: 3000).
*   `INERTIA`: How heavy the flywheel is (0.01 = heavy, 0.1 = light).
*   `NOISE_AMOUNT`: Adds random jitter to RPM to test filter stability.

## Usage
1.  Flash the Simulator board.
2.  Flash the Controller board.
3.  Connect them as shown above.
4.  Open Serial Monitor on the **Controller** to see PID performance.
5.  Open Serial Monitor on the **Simulator** (different port) to see the physics engine state.
