#ifndef CONFIG_H
#define CONFIG_H

// Hardware Pins (ESP32-C3 SuperMini)
#define RPM_INPUT_PIN   0   // GPIO 0 (Requires Voltage Divider 5V->3.3V)
#define PWM_OUTPUT_PIN  1   // GPIO 1

// Motor Configuration
#define PULSES_PER_REV  4   // 8-pole BLDC motor (4 pulses per rev)

// Soft-Start
#define SOFT_START_DURATION_MS  1500  // 1.5 seconds
#define EMA_ALPHA               0.25  // Smoothing factor

#endif // CONFIG_H
