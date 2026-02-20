# Platform Comparison

This document details the technical differences between the available implementations.

| Feature | Arduino Uno | ATtiny85 v1 | ATtiny85 v2 | ESP32-C3 |
| :--- | :--- | :--- | :--- | :--- |
| **Type** | Development | Production (Logic Port) | Production (Optimized) | Embedded RTOS |
| **Processor** | ATmega328P | ATtiny85 | ATtiny85 | RISC-V |
| **Clock** | 16 MHz | 16 MHz (PLL) | 16 MHz (PLL) | 160 MHz |
| **Math** | Float | Float | Integer | Float (FPU) |
| **RPM Measure** | `micros()` | `micros()` | Timer1 Ticks | GPIO ISR |
| **Filtering** | EMA | Median + EMA | Moving Average | EMA |
| **Logic** | 5V | 5V | 5V | 3.3V |
| **Safety** | Emergency Stop | Emergency Stop | Watchdog | FreeRTOS Tasks |

## Detailed Breakdown

### Arduino Uno
*   **Best For**: Tuning, debugging, and initial setup.
*   **Pros**: Serial Plotter, abundant I/O, easy USB connection.
*   **Cons**: Large physical footprint, overkill for simple motor control.

### ATtiny85 v1 (Recommended)
*   **Best For**: Standard production.
*   **Pros**: Runs the exact same logic as the Uno dev board. If it works on Uno, it works here.
*   **Cons**: Floating point math is slower (but sufficient at 16MHz).

### ATtiny85 v2
*   **Best For**: High-speed motors or resource-critical applications.
*   **Pros**: Extremely efficient integer math and hardware timer measurement.
*   **Cons**: Code is more complex and hardware-specific.

### ESP32-C3
*   **Best For**: Smart devices, IoT integration.
*   **Pros**: Massive processing power, FreeRTOS stability, connectivity.
*   **Cons**: Requires 3.3V/5V logic level shifting.
