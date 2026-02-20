# BLDC Motor PID Controller

A robust PID control system for maintaining specific RPM for BLDC motors. This project offers three implementation variants tailored for different stages of development and deployment:

1.  **Arduino Uno**: Development, testing, and tuning.
2.  **ATtiny85**: Production deployment (Low cost, minimal footprint).
3.  **ESP32-C3**: High-performance embedded application (FreeRTOS, 32-bit).

## Implementations

### 1. Arduino Uno (`arduino_uno/`)
**Status**: Development & Tuning
*   **Processor**: ATmega328P (16MHz)
*   **Features**: Serial Plotter interface, Real-time Potentiometer Tuning.
*   **Use Case**: Initial motor characterization, determining PID constants.

### 2. ATtiny85 (`attiny85/`)
**Status**: Production
*   **Processor**: ATtiny85 (16MHz PLL)
*   **Features**: Minimal component count, optimized fixed-point/float math, Watchdog protection.
*   **Variants**:
    *   **v1**: Direct logic port from Arduino Uno.
    *   **v2**: Advanced timer capture implementation.
*   **Use Case**: Final product deployment.

### 3. ESP32-C3 (`esp32_c3/`)
**Status**: Modern Embedded
*   **Processor**: RISC-V (160MHz)
*   **Features**: FreeRTOS task management, Hardware FPU, Wi-Fi/BLE capability.
*   **Use Case**: High-precision applications requiring connectivity or RTOS.

## Quick Start

### Hardware Requirements
*   **Microcontroller**: Arduino Uno, ATtiny85, or ESP32-C3.
*   **Motor**: 3-Phase BLDC Motor (Sensored).
*   **Driver**: ESC (Electronic Speed Controller) with PWM input.
*   **Sensor**: Hall Effect Sensor (Built-in or External).

### Installation
1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/your-repo/PID_simple_controll.git
    ```
2.  **Select Platform**: Navigate to the respective directory (`arduino_uno`, `attiny85`, etc.).
3.  **Configure**: Edit `config.h` to match your motor's pole count and target RPM.
4.  **Flash**: Upload using Arduino IDE or PlatformIO.

## Documentation
*   [Assembly Guide](howto.md): Step-by-step wiring and setup instructions.
*   [Troubleshooting](mitigation.md): Diagnosis and solutions for common issues.
*   [Comparison](COMPARISON.md): detailed feature comparison between platforms.

## License
MIT License
