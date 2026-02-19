# Platform Comparison: Arduino Uno vs ATtiny85 vs ESP32-C3

This document compares the three hardware implementations of the BLDC PID Controller to help you choose the right platform for your needs.

| Feature | Arduino Uno (Development) | ATtiny85 (Production) | ESP32-C3 (Modern Embedded) |
| :--- | :--- | :--- | :--- |
| **Processor** | ATmega328P (AVR) | ATtiny85 (AVR) | ESP32-C3 (RISC-V) |
| **Clock Speed** | 16 MHz | 8 MHz (Internal) | 160 MHz |
| **Architecture** | 8-bit | 8-bit | 32-bit |
| **Flash Memory** | 32 KB | 8 KB | 4 MB |
| **SRAM** | 2 KB | 512 B | 400 KB |
| **Floating Point** | Software (Slow) | Software (Slow) | **Hardware FPU (Fast)** |
| **OS / Multitasking**| Superloop | Superloop | **FreeRTOS** |
| **PID Loop Freq** | ~50-200 Hz | ~200 Hz | **1000+ Hz (Capable)** |
| **RPM Filter** | EMA (Alpha 0.25) | EMA (Alpha 0.25) | EMA (Alpha 0.25) |
| **RPM Measurement** | Interrupt (16-bit Timer) | Interrupt (millis) | **GPIO ISR (Micros)** |
| **PWM Resolution** | 8-bit (0-255) | 8-bit (0-255) | **10-14 bit (Configurable)** |
| **Input Voltage** | 7-12V (VIN) / 5V (USB) | 2.7-5.5V | 3.3V (5V tolerant pins?) **NO** |
| **Logic Level** | 5V | 5V | **3.3V** (Needs Divider) |
| **Cost** | ~$20.00 | ~$2.00 | ~$3.00 (SuperMini) |
| **Physical Size** | Large (68x53mm) | Tiny (DIP-8) | Small (22x18mm) |
| **Best For** | Prototyping, Tuning | Simple, Low-Cost, Rugged | High-Performance, Compact |

## Detailed Breakdown

### 1. Arduino Uno (`arduino_uno`)
*   **Role**: The "Reference" implementation.
*   **Strengths**: Easy to use, 5V logic compatible, serial plotter for tuning, hardware floating point support (simulated).
*   **Weaknesses**: Large size, relatively expensive, slower 8-bit processor.
*   **Use Case**: Initial setup, determining PID constants, testing motors.

### 2. ATtiny85 (`attiny85`)
*   **Role**: The "Minimalist" production version.
*   **Strengths**: Extremely cheap, tiny, robust.
*   **Weaknesses**: Harder to program (needs ISP), limited pins, low memory, no serial debug.
*   **Use Case**: Mass production where cost and space are critical, and parameters are already known.

### 3. ESP32-C3 SuperMini (`esp32_c3`)
*   **Role**: The "Modern" high-performance embedded version.
*   **Strengths**:
    *   **Performance**: 160MHz processor with FPU makes PID math trivial.
    *   **Stability**: FreeRTOS ensures the control loop runs at exact intervals regardless of other tasks.
    *   **Cost/Size**: Cheaper than Uno, barely larger than ATtiny, but vastly more powerful.
    *   **Future Proof**: WiFi/BLE capable (disabled by default) for future OTA or telemetry.
*   **Weaknesses**: **3.3V Logic** requires a voltage divider for 5V Hall sensors.
*   **Use Case**: Production systems requiring high precision, smooth control, or potential future connectivity.

## Feature Parity

All three versions now support:
*   **Boosted Soft-Start**: Starts at 18% PWM (45/255) to overcome static friction, ramping up over 1.5 seconds.
*   **EMA Filtering**: Exponential Moving Average (Alpha 0.25) for stable RPM readings without lag.
*   **Target RPM**: Locked to 1440 RPM (configurable in code).
*   **Protection**: Anti-windup for PID integrator.
