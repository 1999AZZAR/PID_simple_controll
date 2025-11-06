# BLDC PID Controller - ATtiny85 Production Version

This is the production-ready version of the BLDC PID controller running on ATtiny85 microcontroller. All tuning is done on Arduino first, then optimal values are hardcoded for reliable production operation.

## Production Design Philosophy

- **Zero User Controls**: No potentiometers, switches, or adjustments needed
- **Minimal Connections**: Only 2 pins required (RPM sensor + PWM output)
- **Pre-tuned Operation**: Uses optimal PID gains determined during Arduino development
- **Reliable Deployment**: No calibration needed, just plug and run

## Key Features

### Hardware Simplicity
- **2 Connections Only**: RPM sensor input + PWM output to ESC
- **No External Components**: No potentiometers, switches, or LEDs needed
- **Direct Operation**: Powers on and maintains 1440 RPM automatically

### Robust Control
- **Anti-windup Protection**: Prevents integrator runaway
- **100Hz Control Loop**: Responsive motor control
- **Interrupt-based RPM**: Accurate speed measurement
- **Production-hardened**: Optimized for reliability

## Hardware Setup

### Minimal Connections Required

| Physical Pin | Function | Connection |
|-------------|----------|------------|
| 1 (RST) | Reset | Programming only |
| 2 (PB3) | RPM Sensor | Hall/optical/tachometer signal |
| 3 (PB4) | Not Connected | - |
| 4 (GND) | Ground | Common ground |
| 5 (PB0) | PWM to ESC | Motor controller input |
| 6 (PB1) | Not Connected | - |
| 7 (PB2) | Not Connected | - |
| 8 (VCC) | Power | 5V supply |

### Power & Connections
- **ATtiny85 Supply**: 2.7-5.5V (recommended 5V)
- **Motor/ESC Supply**: Separate supply with common ground
- **RPM Sensor**: Connect signal to pin 2, power to 5V, ground to GND
- **ESC Input**: Connect pin 5 to ESC signal input
- **Total Connections**: Only 2 signal wires + power/ground!

## Software Setup

### Arduino IDE Configuration
1. Install ATtiny support: **File → Preferences → Additional Boards Manager URLs**
   ```
   https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
   ```
2. **Tools → Board → ATtiny25/45/85**
3. **Tools → Processor → ATtiny85**
4. **Tools → Clock → 8 MHz (internal)**
5. **Tools → Programmer → Arduino as ISP**

### Programming
1. Connect Arduino as ISP to ATtiny85
2. Upload code using **Sketch → Upload Using Programmer**
3. Remove programming connections
4. Connect motor control hardware

## Deployment Process

### Step 1: Tune on Arduino First
- Use the Arduino version with potentiometers and Serial Plotter
- Tune PID gains for optimal 1440 RPM performance
- Record the final Kp, Ki, Kd values

### Step 2: Transfer Tuned Values
Update these constants in the ATtiny85 code with your Arduino-tuned values:
```cpp
#define PRODUCTION_TARGET_RPM 1440.0  // Usually stays 1440
#define PRODUCTION_KP         0.5     // Your tuned proportional gain
#define PRODUCTION_KI         0.1     // Your tuned integral gain
#define PRODUCTION_KD         0.01    // Your tuned derivative gain
```

### Step 3: Upload to ATtiny85
- Program the ATtiny85 with Arduino as ISP
- Only 2 connections needed: RPM sensor + PWM output
- No potentiometers or switches required

### Step 4: Production Operation
- Power on and the system automatically maintains 1440 RPM
- No user intervention needed
- Reliable, calibrated operation

## Why ATtiny85 for Production?

### Advantages
- **Minimal Hardware**: Only 2 connections needed
- **Low Cost**: ~$1-2 vs $20+ for Arduino
- **Low Power**: ~10mA vs 40mA for Arduino
- **Small Size**: 8-pin DIP package
- **Reliability**: No moving parts (potentiometers) to fail
- **Embedded Ready**: Perfect for integration into products

## Performance Considerations

### Timing Accuracy
- Custom Timer1-based millisecond counter
- 100Hz control loop maintained
- Interrupt-based RPM counting

### PWM Output
- 8-bit resolution (0-255)
- ~1kHz frequency (Timer0 with prescaler 8)
- May need frequency adjustment for specific ESC

### Memory Usage
- Optimized variable usage
- No floating-point operations in interrupts
- Careful use of stack space

## Troubleshooting

### Motor Not Starting
- **PWM Connection**: Verify physical pin 5 connected to ESC signal input
- **ESC Compatibility**: Ensure ESC accepts ~1kHz PWM frequency
- **Power Supply**: Check separate power for motor/ESC with common ground
- **PID Gains**: Verify tuned constants are correctly entered

### No RPM Reading
- **Sensor Connection**: Confirm signal wire on physical pin 2
- **Sensor Power**: Verify 5V and GND connections to sensor
- **Sensor Type**: Ensure correct PULSES_PER_REV constant (usually 1)
- **Interrupt**: Check that PB3 is not damaged

### Unstable Control
- **Tuned Values**: Double-check Arduino-tuned gains are correct
- **Load Changes**: PID should handle load variations automatically
- **Anti-windup**: Integral limits may need adjustment (±50 might be too low)
- **Timing**: Control loop should be exactly 100Hz

### Programming Issues
- **ISP Connection**: Verify MOSI/MISO/SCK/RST connections
- **Board Selection**: Confirm ATtiny85, 8MHz internal clock
- **Fuses**: Check low=0xE2, high=0xDF for 8MHz operation

## Code Optimization Tips

### Reduce Code Size
- Use `#define` instead of `const`
- Remove unused functions
- Optimize mathematical operations

### Improve Performance
- Minimize floating-point calculations
- Use integer math where possible
- Optimize interrupt routines

## Comparison: Arduino vs ATtiny85

| Feature | Arduino | ATtiny85 |
|---------|---------|----------|
| Flash Memory | 32KB | 8KB |
| SRAM | 2KB | 512B |
| EEPROM | 1KB | 512B |
| PWM Pins | 6 | 2 |
| ADC Pins | 6 | 4 |
| Serial Debug | Yes | No |
| Programming | USB | ISP |
| Power Usage | ~40mA | ~10mA |
| Cost | $20+ | $1-2 |

## Advanced Features

### Low-Power Operation
```cpp
// Add sleep mode between control loops
#include <avr/sleep.h>

void enterSleep() {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}
```

### EEPROM Parameter Storage
```cpp
#include <avr/eeprom.h>

void saveParameters() {
    eeprom_write_float((float*)0, kp);
    eeprom_write_float((float*)4, ki);
    eeprom_write_float((float*)8, kd);
}

void loadParameters() {
    kp = eeprom_read_float((float*)0);
    ki = eeprom_read_float((float*)4);
    kd = eeprom_read_float((float*)8);
}
```

## Migration from Arduino

1. **PID Gains**: Transfer directly from Arduino tuning
2. **Control Logic**: Same algorithm, different implementation
3. **Hardware**: Redesign for ATtiny85 pin constraints
4. **Testing**: Start with known good Arduino parameters
5. **Optimization**: Fine-tune for ATtiny85 timing characteristics

This ATtiny85 version maintains the core PID functionality while adapting to the microcontroller's limitations, providing a compact and efficient solution for embedded motor control applications.
