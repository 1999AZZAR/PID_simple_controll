#ifndef CONFIG_H
#define CONFIG_H

// Hardware Pins (ESP32-C3 SuperMini)
#define RPM_INPUT_PIN       0   // GPIO 0 (Requires Voltage Divider 5V->3.3V)
#define PWM_OUTPUT_PIN      1   // GPIO 1
#define POT_ENABLE_PIN      3   // GPIO 3 - Pull LOW to enable pot target mode (INPUT_PULLUP)
#define POT_TARGET_RPM_PIN  2   // GPIO 2 (ADC1_CH2) - Potentiometer for target RPM

// Target RPM Range (when pot is enabled)
#define TARGET_RPM_MIN  1000
#define TARGET_RPM_MAX  3000

// Motor Configuration
#define PULSES_PER_REV  4   // 8-pole BLDC motor (4 pulses per rev)

// RPM measurement: min integration window (us). 40ms = ~4 pulses @ 1440 RPM.
#define RPM_SAMPLE_MIN_US  40000

// Soft-Start
#define SOFT_START_DURATION_MS  1500  // 1.5 seconds

#endif // CONFIG_H
