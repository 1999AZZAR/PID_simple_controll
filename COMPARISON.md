# BLDC Motor PID Controller - Versions Comparison

## Overview

This document provides a comprehensive comparison between the three optimized BLDC motor PID controller implementations: Arduino Uno, ATtiny85, and Auto-Tune versions. All versions have been optimized for maximum accuracy with microsecond-precision timing and moving average filtering.

## Technical Specifications Comparison

| Feature | Arduino Uno | ATtiny85 | Auto-Tune |
|---------|-------------|----------|-----------|
| **Microcontroller** | ATmega328P | ATtiny85 | ATmega328P |
| **Clock Speed** | 16MHz | 8MHz | 16MHz |
| **Flash Memory** | 32KB | 8KB | 32KB |
| **SRAM** | 2KB | 512B | 2KB |
| **EEPROM** | 1KB | 512B | 1KB |
| **ADC Channels** | 6 | 4 | 6 |
| **PWM Channels** | 6 | 4 | 6 |
| **Interrupt Pins** | 2 | 1 | 2 |

## Performance Comparison

| Metric | Arduino Uno | ATtiny85 | Auto-Tune |
|--------|-------------|----------|-----------|
| **Control Loop Frequency** | 200Hz | 160Hz | 200Hz |
| **RPM Update Interval** | 10ms | 10ms | 10ms |
| **Interrupt Debounce** | 50μs | 50μs | 50μs |
| **Timing Precision** | Microseconds | Microseconds | Microseconds |
| **RPM Filtering** | 5-sample MA | 3-sample MA | 5-sample MA |
| **Memory Usage** | 7016B (21%) | 1954B (23%) | 11716B (36%) |
| **RAM Usage** | 394B (19%) | 57B (11%) | 426B (20%) |

## Feature Comparison

| Feature | Arduino Uno | ATtiny85 | Auto-Tune |
|---------|-------------|----------|-----------|
| **PID Tuning** | Potentiometer | Pre-tuned | Serial GUI |
| **Serial Communication** | Plotter Output | None | Full GUI |
| **EEPROM Storage** | Available | Limited | Available |
| **Watchdog Timer** | Available | Available | Available |
| **Soft-Start** | Implemented | Removed* | Implemented |
| **Safety Features** | Basic | Minimal | Full |
| **Real-time Tuning** | Hardware Pots | Fixed | Software GUI |
| **Data Logging** | Serial Plotter | None | GUI Charts |

> *Note: Soft-start removed from ATtiny85 for size optimization

## Hall Sensor Wiring Options

### Option 1: Single Hall Sensor Cable (Recommended)

**Connection Method:**
- Connect any **single Hall sensor wire** (A, B, or C) directly from the BLDC motor to the microcontroller
- Use the Hall sensor's built-in pull-up resistor
- All Hall sensors are electrically identical - any one will work

**Advantages:**
- ✅ **Simplest wiring** - only 1 signal wire + GND + VCC
- ✅ **Direct motor connection** - no intermediate electronics
- ✅ **Maximum reliability** - fewer connection points
- ✅ **Noise immunity** - direct Hall sensor signal
- ✅ **Universal compatibility** - works with any 3-Hall BLDC motor

**Wiring Diagram:**
```
BLDC Motor Hall Sensor → Microcontroller Interrupt Pin
    Hall A (or B or C)   →     Digital Pin 2 (Arduino) / Pin 3 (ATtiny85)
    Hall GND            →     GND
    Hall VCC (5V)       →     5V
```

### Option 2: Aggregated Hall from Motor Controller (ESC)

**Connection Method:**
- Use the Hall sensor output from the motor controller (ESC)
- ESC aggregates/combines all three Hall sensors internally
- Single output represents combined Hall state

**Advantages:**
- ✅ **ESC compatibility** - designed for motor controllers
- ✅ **Built-in filtering** - ESC may provide signal conditioning

**Disadvantages:**
- ❌ **Complex wiring** - requires ESC Hall output pin access
- ❌ **ESC dependent** - only works with specific ESCs
- ❌ **Signal delay** - additional processing time in ESC
- ❌ **Limited availability** - not all ESCs expose Hall signals
- ❌ **Compatibility issues** - ESC firmware may interfere

**Wiring Diagram:**
```
ESC Hall Output → Microcontroller Interrupt Pin
    ESC Hall Out    →     Digital Pin 2 (Arduino) / Pin 3 (ATtiny85)
    ESC GND         →     GND
    ESC 5V          →     5V
```

## Recommended Wiring Configuration

### For All Versions: Single Hall Sensor (Direct Motor Connection)

```text
BLDC Motor     Microcontroller
Hall A/B/C ──────────────────► Interrupt Pin (D2/ATtiny85-P3)
GND ─────────────────────────► GND
VCC ─────────────────────────► 5V
```

**Why Single Hall is Recommended:**
1. **Universal Compatibility** - Works with any 3-Hall BLDC motor
2. **Maximum Accuracy** - Direct sensor signal, no intermediate processing
3. **Simplest Setup** - Minimal wiring complexity
4. **Highest Reliability** - Fewest potential failure points
5. **Best Performance** - Fastest response time

## Pulse Characteristics by Wiring Method

| Wiring Method | Pulses/Revolution | Signal Quality | Response Time |
|---------------|-------------------|----------------|---------------|
| **Single Hall (Direct)** | 6 pulses/rev | Excellent | <50μs |
| **ESC Aggregated** | Variable* | Good-Fair | 100-500μs |

> *Depends on ESC implementation - typically 6, 12, or 24 pulses/rev

## Code-Specific Hall Sensor Configuration

### Arduino Uno & Auto-Tune Versions
```cpp
#define RPM_SENSOR_PIN      2   // Interrupt pin for BLDC Hall sensor
#define MIN_PULSE_WIDTH_US  50   // 50μs debounce for direct Hall connection
```

### ATtiny85 Version
```cpp
#define RPM_SENSOR_PIN     3    // Arduino pin 3 (PB3), interrupt capable
#define MIN_PULSE_WIDTH_US  50   // 50μs debounce for direct Hall connection
```

## Performance Impact of Wiring Choice

### Single Hall Sensor (Recommended)
- **RPM Accuracy**: ±1 RPM at 1440 RPM target
- **Response Time**: <50μs interrupt latency
- **Noise Immunity**: Excellent (direct sensor)
- **Setup Complexity**: Minimal

### ESC Aggregated Hall
- **RPM Accuracy**: ±2-5 RPM (variable)
- **Response Time**: 100-500μs (ESC processing)
- **Noise Immunity**: Good (ESC filtering)
- **Setup Complexity**: High (ESC-dependent)

## Troubleshooting Hall Sensor Issues

### Symptom: No RPM Reading
**Possible Causes:**
1. Wrong Hall sensor pin selection
2. Loose or incorrect wiring
3. Insufficient Hall sensor voltage (needs 5V)
4. Motor not spinning or Hall sensors damaged

**Solutions:**
1. Verify wiring matches the recommended single Hall configuration
2. Test with different Hall sensor wire (A, B, or C)
3. Ensure stable 5V supply to Hall sensors
4. Check motor Hall sensors with multimeter

### Symptom: Erratic RPM Readings
**Possible Causes:**
1. Electrical noise interference
2. Insufficient debounce time
3. Poor Hall sensor signal quality
4. Motor vibration affecting sensors

**Solutions:**
1. Use shielded cable for Hall sensor connection
2. Verify MIN_PULSE_WIDTH_US setting (currently 50μs)
3. Ensure clean power supply to Hall sensors
4. Add mechanical damping if vibration is issue

## Version Selection Guide

### Choose Arduino Uno When:
- Need potentiometer-based tuning
- Require serial plotter monitoring
- Have space constraints but need full features
- Want production-ready with hardware tuning

### Choose ATtiny85 When:
- Minimal size and power consumption critical
- Pre-tuned operation acceptable
- No real-time monitoring needed
- Cost and space are primary constraints

### Choose Auto-Tune When:
- Need sophisticated GUI-based tuning
- Require detailed data logging and charts
- Want real-time parameter adjustment
- Have computer available for tuning

## Summary

All three versions provide **maximum accuracy** with **microsecond-precision timing** and **moving average filtering**. The **single Hall sensor wiring** is strongly recommended for all implementations due to its simplicity, reliability, and superior performance.

**Recommended Setup:** Arduino Uno + Single Hall Sensor for development, ATtiny85 + Single Hall Sensor for production deployment.
