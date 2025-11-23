# PID Controller Troubleshooting & Mitigation Guide

## Quick Start Guide (For Beginners)

**Don't panic!** Most PID controller issues have simple fixes. Start here if you're new to electronics.

### What You'll Need:
- **Multimeter** (voltage tester) - $10-20
- **LED** (any color) + **220Ω resistor** - for testing connections
- **Paper clips or jumper wires** - for testing circuits
- **Patience and methodical approach**

### Basic Testing Steps (Do These First):
1. **Power Check**: Measure voltage at Arduino power pins (should be 5V)
2. **Connection Check**: Touch multimeter leads to verify wires are connected
3. **LED Test**: Connect LED to PWM output to see if signal is working
4. **Serial Monitor**: Open Arduino Serial Monitor to see error messages

## Basic Concepts (Understanding Your System)

### What is a PID Controller?
Think of it like cruise control in a car:
- **P (Proportional)**: Like pressing gas harder when going too slow
- **I (Integral)**: Like remembering you were slow and keeping extra pressure
- **D (Derivative)**: Like easing off gas before you overshoot your speed

### Key Components:
- **Arduino Uno/ATTiny85**: The "brain" that does calculations
- **BLDC Motor**: The spinning part you want to control
- **Hall Sensor**: Speed detector (counts rotations)
- **ESC**: Electronic Speed Controller (motor power amplifier)
- **Power Supply**: Provides electricity to everything

### Common Terms:
- **RPM**: Rotations Per Minute (how fast motor spins)
- **PWM**: Pulse Width Modulation (how we control motor power)
- **Interrupt**: Special signal that interrupts normal code execution
- **Pulse**: Electrical signal from Hall sensor each rotation

## Overview

This guide covers troubleshooting the BLDC Motor PID Controller system. Whether you're a beginner or expert, follow the step-by-step instructions to diagnose and fix issues.

## Step-by-Step Diagnosis Process

### Step 1: Safety First
WARNING: ALWAYS disconnect power before touching wires!
- Unplug USB cable
- Disconnect motor power supply
- Wait 30 seconds for capacitors to discharge

### Step 2: Visual Inspection
Look for these common issues:
- [ ] Loose wires or poor connections
- [ ] Burnt components (black marks)
- [ ] Bent pins on Arduino/ATTiny85
- [ ] Wrong wires connected to wrong pins
- [ ] Power supply LED not lit

### Step 3: Power Testing
Use multimeter to check:
- [ ] Arduino 5V pin = 5.0V (±0.1V)
- [ ] Arduino VIN pin = 7-12V (if using external power)
- [ ] Motor power supply = correct voltage for your motor

### Step 4: LED Testing
Test each connection point:
- Connect LED + resistor to Arduino 5V → LED should light
- Connect LED + resistor to Arduino PWM pin → LED should blink during operation

## Known Issues (With Easy Fixes)

### 1. Motor Won't Start (Most Common Issue)

**Symptoms:**
- Motor doesn't spin at all
- Arduino LED lights but motor is silent
- ESC beeps but motor doesn't move

**Quick Fixes:**
1. Check power connections to ESC
2. Verify PWM wire from Arduino pin 9 to ESC signal input
3. Test with LED on PWM pin - should blink during operation

**Beginner Solution:**
```bash
# Connect LED test circuit:
Arduino Pin 9 → 220Ω resistor → LED → GND

# If LED blinks: PWM signal is working
# If LED stays off: Arduino code has problem
```

### 2. RPM Reading Wrong (4907 RPM vs 1440 Target)

**What it means:** Your controller thinks motor is spinning 3.41x faster than it actually is.

**Beginner Explanation:**
- Hall sensor sends electrical pulses as motor spins
- Controller counts these pulses to calculate speed
- Wrong count = wrong speed calculation

**Quick Test:**
1. Manually spin motor slowly (1 rotation per second)
2. Count LED flashes (if connected to Hall sensor pin)
3. Should see 6 flashes per rotation for 3-Hall motors

### 3. Arduino Uno Only Works With USB Unplugged

**What it means:** Two power sources are fighting each other.

**Beginner Explanation:**
Arduino Uno has a built-in power regulator. When you connect both USB power AND external power, they conflict like two people trying to drive the same car.

**Visual Guide:**
```
WRONG - DON'T DO THIS:
Computer USB ────→ Arduino Uno ←──── Motor Power Supply
                   (5V from USB + 12V to VIN = FIGHT!)

RIGHT - CHOOSE ONE:
Option A: Computer USB ────→ Arduino Uno (disconnect motor power)
Option B: Motor Power Supply ────→ Arduino Uno (disconnect USB)
```

### 4. ATTiny85 Won't Program

**Symptoms:**
- "Programmer not responding" error
- "Device signature" error
- ATTiny85 gets hot during programming

**Beginner Solution:**
1. Add 10uF capacitor between Arduino Reset pin and GND
2. Check all 6 ISP wires are connected correctly
3. Use separate power supply for ATTiny85 (not Arduino power)
4. Try different ATTiny85 chip (current one might be damaged)

### 5. Motor Spins But Speed is Unstable

**Symptoms:**
- Motor speed jumps around randomly
- Can't hold steady RPM
- Works sometimes, fails other times

**Common Causes:**
- Loose Hall sensor wires
- Electrical noise from motor
- Wrong PID settings

### 6. Serial Monitor Shows No Data

**Symptoms:**
- Arduino Serial Monitor window is blank
- Can't send commands to controller

**Beginner Fixes:**
1. Check USB cable is properly connected
2. Verify correct COM port selected in Arduino IDE
3. Confirm baud rate is 115200 in Serial Monitor
4. Try different USB port on computer

## Detailed Troubleshooting (Step-by-Step)

### RPM Measurement Issues

#### Issue 1: Wrong Pulse Count (Most Common RPM Problem)

**What it means:** Your Hall sensor is sending more/fewer pulses than the code expects.

**For Beginners:** Imagine counting people entering a room. If you count each person twice, your total will be 2x too high.

**Visual Test (No Oscilloscope Needed):**
```
Step 1: Connect LED to Hall sensor pin
Arduino Pin 2 → 220Ω resistor → LED → GND

Step 2: Spin motor by hand VERY slowly (1 rotation per second)
Step 3: Count LED flashes per rotation
Step 4: Compare with expected:
- 3-Hall motor (42BLF20-22.0223): 6 flashes per rotation
- 6-Hall motor: 12 flashes per rotation
```

**Beginner Fix:**
```cpp
// Change this line in config.h:
#define PULSES_PER_REV 6    // Change this number based on your test

// If your test showed:
// 6 flashes = use 6
// 12 flashes = use 12
// 18 flashes = use 18
```

**Advanced Testing (With Oscilloscope):**
1. Connect oscilloscope probe to Hall sensor wire
2. Set trigger to "rising edge"
3. Spin motor one full rotation
4. Count pulses on screen
5. Update code with correct number

**Why This Happens:**
- Different motor types have different Hall sensor configurations
- 3-Hall motors: 6 pulses per mechanical revolution
- 6-Hall motors: 12 pulses per mechanical revolution
- Some motors have internal gearing that multiplies pulse count

#### Issue 2: Hall Sensor Signal Problems

**Symptoms:**
- RPM reading jumps randomly
- Motor speed unstable
- Controller shows "0 RPM" intermittently

**Beginner Explanation:**
Hall sensors send electrical signals when magnets pass by. These signals can be "bouncy" like a ball that bounces when it hits the ground.

**Visual Understanding:**
```
Good Signal:    ──┐   ──┐   ──┐   ──┐   (Clean pulses)
Bad Signal:     ─╥─╨─╥─╨─╥─╨─╥─╨─ (Bouncy/noisy)
```

**Beginner Fixes:**

**Fix A: Check Wiring**
1. Hall sensor has 3 wires: Power (+5V), Ground (GND), Signal
2. Connect Power to Arduino 5V pin
3. Connect Ground to Arduino GND
4. Connect Signal to Arduino Pin 2 (Uno) or Pin 2 (ATTiny85 physical)

**Fix B: Add Signal Filtering**
```cpp
// In config.h, increase debounce time:
#define MIN_PULSE_WIDTH_US 500  // Increase from 100 to 500

// This ignores signals shorter than 500 microseconds
```

**Fix C: Hardware Filtering (If Software Doesn't Work)**
```
Hall Sensor Signal Wire ──► 10kΩ resistor ──► Arduino Pin 2
                                    │
                                    ├─► 0.1uF capacitor ──► GND
                                    │
                                    └─► 1kΩ resistor ──► GND (optional)
```

**Testing Signal Quality:**
1. Connect LED to signal pin as described above
2. Spin motor slowly - LED should flash cleanly once per magnet
3. If LED flickers randomly, you have noise problems
4. Try twisting Hall sensor wires together to reduce interference

#### Issue 3: Electrical Noise Problems

**Symptoms:**
- RPM readings change without motor speed changing
- Motor stops unexpectedly
- Random resets or erratic behavior

**Beginner Explanation:**
Motors create electrical "noise" like radio interference. This noise can confuse the Arduino into thinking it received Hall sensor signals when it didn't.

**Quick Test:**
1. Turn off motor power but keep Arduino running
2. Check if RPM reading is still changing
3. If it changes with motor off, you have electrical noise

**Beginner Fixes:**

**Fix A: Separate Power Supplies**
```
Use separate power supplies:
Computer USB → Arduino Uno (logic power)
Battery Pack → Motor + ESC (motor power)

Don't connect Arduino GND to motor GND at same point
```

**Fix B: Add Filtering Capacitors**
```
At Arduino power pins:
5V pin ───► 10uF capacitor ───► GND
VIN pin ───► 100uF capacitor ───► GND

At motor power wires:
Motor + ───► 0.1uF capacitor ───► Motor GND
```

**Fix C: Use Shielded Cables**
- Use shielded cable for Hall sensor wires
- Connect shield to Arduino GND only at Arduino end
- Keep Hall sensor wires away from motor wires

**Fix D: Add Ferrite Beads**
```
Motor power wire ──► Ferrite bead ──► ESC
Hall sensor wires ──► Through ferrite bead ──► Arduino
```

#### Issue 4: Wrong Tachometer Measurement

**Symptoms:**
- Laser tachometer shows different RPM than controller
- Ratio between readings is constant

**Beginner Explanation:**
Your laser tachometer might be measuring a different part of the motor system than the Hall sensors.

**Visual Guide:**
```
Motor Output Shaft ──► What laser tachometer sees
     ↓
Internal Gears ──► What Hall sensors measure
     ↓
Final Drive ──► What you actually want to control
```

**Common Problems:**
1. **Gear Reduction:** Motor has internal gears (e.g., 10:1 ratio)
2. **Belt/Pulley Drive:** Motor connected via belt to final shaft
3. **Different Shafts:** Tachometer measuring wrong shaft

**Fix:**
- Measure RPM at same shaft where Hall sensors are located
- Calculate gear ratios and adjust expectations
- Verify tachometer is pointing at rotating surface, not stationary parts

#### Cause 3: Electrical Noise and EMI
**Description**: Electromagnetic interference causing false pulse triggers.

**Symptoms**:
- Inconsistent RPM readings
- Motor speed varies without load changes
- False emergency stops

**Mitigation**:
```cpp
// Increase debounce filtering
#define MIN_PULSE_WIDTH_US 500  // Increase from 100 to 500us

// Hardware solutions:
// 1. Add RC filter (10kΩ + 0.1uF) to Hall sensor input
// 2. Use shielded cables for Hall sensor connections
// 3. Separate power supplies for controller and motor if possible
// 4. Add ferrite beads on power lines
```

#### Cause 4: Motor Shaft vs Sensor Alignment
**Description**: Tachometer measuring different shaft than Hall sensors.

**Symptoms**:
- RPM discrepancy between controller and external measurement
- Ratio not matching expected pulse count

**Mitigation**:
- Ensure laser tachometer targets same shaft as motor output shaft
- Verify gear ratios if motor has internal gearing
- Check for belt/pulley speed differences

### Power Supply Issues (Very Common!)

#### Issue 1: Arduino Uno Power Fight (Your Main Problem)

**What happens:** Arduino Uno has a built-in voltage regulator that converts input voltage to 5V. When you connect both USB (5V) and external power (7-12V), they fight each other.

**Visual Explanation:**
```
Arduino Uno Voltage Regulator:
External Power (12V) ──► Regulator ──► 5V for Arduino
USB Power (5V) ────────► Also tries to make 5V ──► FIGHT!

Result: Regulator overheats, Arduino behaves strangely
```

**Beginner Fix (Choose ONE power source):**

**Option A: USB Power Only**
1. Connect Arduino to computer with USB cable
2. **DO NOT** connect any wires to VIN or external 5V pins
3. Motor gets power from separate battery/ESC power

**Option B: External Power Only**
1. **Disconnect USB cable** from computer
2. Connect 7-12V DC power supply to Arduino VIN pin
3. Make sure power supply can provide 500mA minimum

**Testing Which Option Works:**
```bash
# Test Option A (USB power):
1. Connect USB only
2. Open Arduino Serial Monitor
3. Type: GET PARAMS
4. If you see data, USB power works

# Test Option B (External power):
1. Disconnect USB
2. Connect external power to VIN
3. Try Serial Monitor again
```

**Why This Confuses Beginners:**
- Arduino Uno can accept power multiple ways
- Most people connect everything "just in case"
- Regulator can't handle two power sources simultaneously

#### Issue 2: Power Supply Too Weak

**Symptoms:**
- Motor starts but slows down under load
- Arduino resets unexpectedly
- Voltage drops when motor accelerates

**Beginner Test:**
1. Measure voltage at Arduino VIN pin with motor off
2. Start motor and measure voltage again
3. If voltage drops more than 0.5V, power supply is too weak

**Quick Fix:**
- Use power supply rated for 2x your motor's current draw
- Example: If motor draws 2A, use 4A minimum power supply
- Add large capacitor (1000uF+) across power terminals

**Visual Guide:**
```
Weak Power Supply:    Motor starts → Voltage drops → Arduino resets
Strong Power Supply:  Motor starts → Voltage stable → Works fine

Add capacitor:        Power Supply ───► 1000uF capacitor ───► Motor
```

#### Issue 3: Power Supply Noise

**Symptoms:**
- Arduino behaves erratically
- False Hall sensor readings
- Random motor stops

**Beginner Explanation:**
Switching power supplies create electrical "noise" that confuses microcontrollers.

**Fix Options:**

**Option A: Use Linear Power Supply**
- Replace switching supply with linear regulated supply
- More expensive but much cleaner power

**Option B: Add LC Filter**
```
Power Supply ──► Inductor (100uH) ──► Large Capacitor (1000uF) ──► Load
                │
                └─► Small Capacitor (0.1uF) ──► GND
```

**Option C: Separate Power Supplies**
```
Clean Power → Arduino Uno (logic circuits)
Noisy Power → Motor + ESC (power circuits)

Connect GND wires at single point only
```

**Testing for Noise:**
1. Use battery instead of wall power supply
2. If problem goes away, you had power supply noise
3. Add capacitors as shown above

#### Cause 2: Insufficient Power Supply Capacity
**Description**: Power supply cannot provide required current for motor operation.

**Symptoms**:
- Motor stalls under load
- Voltage drops during acceleration
- Controller resets unexpectedly

**Mitigation**:
- Use power supply rated for 2x expected current
- Add large capacitor (1000uF+) across power rails
- Monitor voltage with multimeter during operation
- Consider separate supplies for logic and motor power

#### Cause 3: Power Supply Noise
**Description**: Switching noise from motor power supply affects microcontroller.

**Symptoms**:
- Erratic microcontroller behavior
- False interrupt triggers
- Analog readings unstable

**Mitigation**:
- Use linear regulated power supply instead of switching
- Add LC filter on power input
- Use separate power supplies for controller and motor
- Add decoupling capacitors (10uF + 0.1uF) at microcontroller power pins

### ATTiny85 Issues (Advanced Users)

#### Issue 1: Can't Upload Code (Programming Fails)

**Symptoms:**
- "Programmer is not responding" error
- "Yikes! Invalid device signature" error
- ATTiny85 gets hot during programming attempt

**Beginner Explanation:**
ATTiny85 needs special programming setup. You can't just upload code like Arduino Uno - you need an ISP (In-System Programmer).

**Step-by-Step Setup:**

**Step 1: Set Up Arduino as Programmer**
1. Open Arduino IDE
2. File → Examples → ArduinoISP → ArduinoISP
3. Upload ArduinoISP sketch to your Arduino Uno
4. Arduino Uno is now a programmer!

**Step 2: Connect ATTiny85 to Arduino**
```
Arduino Uno Pins → ATTiny85 Physical Pins

Pin 10 (SS) → ATTiny85 Pin 1 (Reset)
Pin 11 (MOSI) → ATTiny85 Pin 5 (MOSI)
Pin 12 (MISO) → ATTiny85 Pin 6 (MISO)
Pin 13 (SCK) → ATTiny85 Pin 7 (SCK)
GND → ATTiny85 Pin 4 (GND)
5V → ATTiny85 Pin 8 (VCC)

CRITICAL: Add 10uF capacitor between Arduino Reset and GND!
```

**Step 3: Configure Arduino IDE**
```
Board: "ATTiny25/45/85"
Processor: "ATTiny25/45/85"
Clock: "8 MHz (internal)"
Programmer: "Arduino as ISP"
```

**Common Mistakes:**
- [ ] Forgot 10uF capacitor on Arduino Reset pin
- [ ] Wrong ISP cable connections (check each wire!)
- [ ] Motor still connected (disconnect during programming)
- [ ] Wrong board selected in Arduino IDE
- [ ] Using wrong COM port

**Testing Programmer:**
1. Select "ATTiny25/45/85" board
2. Try "Burn Bootloader" first
3. If successful, try uploading your code
4. If fails, check all connections again

#### Issue 2: ATTiny85 Not Working After Programming

**Symptoms:**
- Code uploads successfully but motor doesn't work
- ATTiny85 gets hot in circuit
- Random behavior

**Beginner Fixes:**

**Fix A: Check Power Supply**
- ATTiny85 needs stable 5V
- Don't use Arduino Uno power when ATTiny85 is programmed
- Use separate 5V regulator for ATTiny85

**Fix B: Verify Pin Connections**
```
ATTiny85 Physical Pins (IMPORTANT - not Arduino pin numbers!):
Pin 8 (VCC) → 5V power
Pin 4 (GND) → Ground
Pin 2 (PB3) → Hall sensor signal
Pin 5 (PB0) → ESC PWM input
```

**Fix C: Fuse Settings Check**
- Wrong fuse settings can make ATTiny85 run at wrong speed
- Use "Burn Bootloader" to set correct fuses
- Don't manually change fuses unless you know what you're doing

**Recovery Procedure:**
1. Remove ATTiny85 from circuit
2. Program on breadboard with known working setup
3. Test with LED before putting back in motor circuit
4. Check each connection individually

#### Cause 2: ATTiny85 Pin Limitations
**Description**: ATTiny85 has limited pins causing hardware conflicts.

**Symptoms**:
- PWM output conflicts with other functions
- No debugging capabilities
- Limited I/O options

**Mitigation**:
```cpp
// ATTiny85 pin assignments - DO NOT CHANGE
#define RPM_SENSOR_PIN PB3  // Physical pin 2 (interrupt capable)
#define PWM_OUTPUT_PIN PB0  // Physical pin 5 (PWM capable)

// Available pins: PB0, PB1, PB2, PB3, PB4, PB5
// Reserved: Reset (PB5), VCC (PB1), GND (PB4)
```

### PID Control Issues

#### Cause 1: Incorrect PID Tuning
**Description**: PID gains not suitable for motor/load combination.

**Symptoms**:
- Oscillating motor speed
- Slow response to load changes
- Motor stalls or runs away

**Mitigation**:
```cpp
// Production defaults (conservative tuning)
#define PRODUCTION_KP 0.5
#define PRODUCTION_KI 0.1
#define PRODUCTION_KD 0.01

// Tuning procedure:
// 1. Start with KI=0, KD=0, increase KP until oscillation
// 2. Reduce KP by 50%, increase KI for steady-state error
// 3. Add small KD to dampen oscillations
// 4. Test under actual load conditions
```

#### Cause 2: Integral Windup
**Description**: Integral term accumulates excessively during saturation.

**Symptoms**:
- Long recovery time after load changes
- Overshoot after error correction
- Unstable behavior after saturation

**Mitigation**:
```cpp
// Anti-windup protection
#define INTEGRAL_WINDUP_MIN -100
#define INTEGRAL_WINDUP_MAX 100

// In PID computation:
integral += ki * error;
integral = constrain(integral, INTEGRAL_WINDUP_MIN, INTEGRAL_WINDUP_MAX);
```

### Safety System Issues

#### Cause 1: Emergency Stop False Triggers
**Description**: Safety system activates inappropriately.

**Symptoms**:
- Motor stops unexpectedly
- Controller enters emergency mode
- PWM output drops to zero

**Mitigation**:
```cpp
// Adjust safety thresholds based on motor characteristics
#define EMERGENCY_STOP_TIMEOUT_MS 5000  // Increase if motor slow to start
#define RPM_CALC_INTERVAL 100           // Decrease for faster RPM updates

// Emergency stop conditions:
// 1. No pulses received for timeout period
// 2. Motor running too fast (>2x target RPM)
// 3. High PWM but no RPM (stalled motor)
```

## Troubleshooting Flowchart

```
Start Diagnosis
     │
     ▼
Power Issues?
     │
     ├── Yes → Check Arduino power supply (single source only)
     │       → Verify voltage levels (7-12V VIN, stable 5V)
     │       → Test with known good power supply
     │
     ▼
RPM Reading Issues?
     │
     ├── Yes → Count actual Hall sensor pulses per revolution
     │       → Verify interrupt pin connections
     │       → Check for electrical noise (add filtering)
     │       → Confirm tachometer measures correct shaft
     │
     ▼
Programming Issues?
     │
     ├── Yes → Verify ISP connections (Arduino as programmer)
     │       → Check ATTiny85 fuse settings
     │       → Ensure clean power during programming
     │       → Test with known working ATTiny85
     │
     ▼
PID Control Issues?
     │
     ├── Yes → Check PID gains (start with conservative values)
     │       → Verify target RPM is achievable
     │       → Test without load first
     │       → Monitor PID components separately
     │
     ▼
Hardware Issues?
     │
     ├── Yes → Check all wiring connections
     │       → Verify motor specifications match code
     │       → Test ESC independently
     │       → Check for component damage
     │
     ▼
Issue Resolved?
     │
     ├── Yes → Document solution for future reference
     │
     └── No → Escalate to advanced debugging
             → Use oscilloscope for signal analysis
             → Check component datasheets
             → Consider hardware redesign
```

## Prevention Measures

### Design Phase
- [ ] Select microcontroller with sufficient I/O pins
- [ ] Choose power supply with 2x required capacity
- [ ] Design with debugging capabilities
- [ ] Include test points for oscilloscope access
- [ ] Plan for EMI shielding from motor

### Implementation Phase
- [ ] Use shielded cables for Hall sensors
- [ ] Add decoupling capacitors at power inputs
- [ ] Implement proper grounding scheme
- [ ] Include current limiting on power supplies
- [ ] Test with dummy load before actual motor

### Testing Phase
- [ ] Verify pulse counting with oscilloscope
- [ ] Test power supply combinations safely
- [ ] Characterize motor behavior under load
- [ ] Validate PID tuning across operating range
- [ ] Document all working configurations

## Emergency Recovery

### Soft Reset (Software)
```cpp
// Arduino Uno serial command:
MODE PRODUCTION    // Reset to known state
RESET INTEGRAL     // Clear PID integral term
SET TARGET 1440    // Set known target RPM
```

### Hard Reset (Hardware)
- Disconnect all power sources
- Wait 30 seconds for capacitors to discharge
- Reconnect single power source
- Upload fresh firmware if needed

### ATTiny85 Recovery
- Remove ATTiny85 from circuit
- Program with ArduinoISP using fresh firmware
- Test on breadboard before reinstalling
- Check fuse settings if programming fails

## FAQ - Frequently Asked Questions

### Q: Motor spins but not at target RPM. What's wrong?
**A:** Check your Hall sensor pulse count. Most beginners use wrong pulse count. Do the LED flash test above.

### Q: Arduino works with USB but not with external power?
**A:** This is the power supply conflict! Use ONE power source only. Don't connect both USB and external power.

### Q: Getting "avrdude: programmer is not responding" error?
**A:** For ATTiny85 programming: Check the 10uF capacitor on Arduino Reset pin and verify all 6 ISP wires.

### Q: RPM reading changes when I touch wires?
**A:** You have electrical noise. Add filtering capacitors and use shielded cables for Hall sensors.

### Q: Motor works sometimes but not others?
**A:** Loose connections. Check all wires with multimeter continuity test.

### Q: Serial Monitor shows nothing?
**A:** Wrong COM port, wrong baud rate (should be 115200), or Arduino not powered correctly.

### Q: ATTiny85 programs but doesn't work?
**A:** Check fuse settings and power supply. ATTiny85 needs stable 5V separate from Arduino.

### Q: Motor stalls under load?
**A:** Power supply too weak. Use supply rated for 2x motor current and add large capacitor.

### Q: PID gains don't work well?
**A:** Start with conservative values: KP=0.5, KI=0.1, KD=0.01. Tune one parameter at a time.

### Q: Getting emergency stop errors?
**A:** Increase timeout or check Hall sensor wiring. Emergency stop triggers if no pulses received.

## Basic Wiring Guide (For Beginners)

### Arduino Uno Basic Setup:

```
Computer USB ──────────────────────→ Arduino Uno USB Port
                                     │
Arduino Pins:                       │
├── Pin 2 ──► Hall Sensor Signal     │
├── Pin 9 ──► ESC PWM Signal         │
├── 5V ─────► Hall Sensor Power      │
└── GND ────► Hall Sensor Ground     │
                                     │
Motor Power Supply ──► ESC Power Input │
ESC Motor Output ──► BLDC Motor       │
```

### ATTiny85 Basic Setup:

```
ATTiny85 Physical Pins:
├── Pin 8 (VCC) ──► 5V Power
├── Pin 4 (GND) ──► Ground
├── Pin 2 (PB3) ──► Hall Sensor Signal
└── Pin 5 (PB0) ──► ESC PWM Signal

Motor Connections:
├── Hall Sensor Power ──► 5V
├── Hall Sensor Ground ──► GND
├── ESC Power ──► Motor Power Supply
└── ESC Motor ──► BLDC Motor
```

### Testing Each Connection:

**Power Test:**
1. Arduino 5V pin should read 5.0V
2. Arduino VIN should read 7-12V (if using external power)

**Signal Test:**
1. Connect LED to PWM pin - should blink during operation
2. Connect LED to Hall pin - should flash when motor spins

**Motor Test:**
1. Disconnect Hall sensor, connect motor directly to ESC
2. ESC should beep and motor should respond to throttle

## Troubleshooting Checklist (Print This!)

### Before Starting:
- [ ] Safety glasses on
- [ ] Power disconnected
- [ ] Multimeter available
- [ ] Arduino IDE installed

### Power Checks:
- [ ] Arduino 5V pin = 5.0V (±0.1V)
- [ ] Arduino VIN = 7-12V (if used)
- [ ] Motor power supply = correct voltage
- [ ] No USB + external power conflict

### Wiring Checks:
- [ ] Hall sensor: Power, Ground, Signal all connected
- [ ] PWM wire from Arduino Pin 9 to ESC
- [ ] All connections secure (wiggle test)
- [ ] No short circuits between wires

### Software Checks:
- [ ] Correct board selected in Arduino IDE
- [ ] Correct COM port selected
- [ ] Baud rate = 115200 in Serial Monitor
- [ ] Code uploaded successfully

### Motor Tests:
- [ ] ESC powers on (beeps)
- [ ] Motor spins with direct ESC control
- [ ] Hall sensor shows pulses when spinning
- [ ] PID parameters reasonable

### Advanced Checks:
- [ ] Pulse count verified (LED flash test)
- [ ] No electrical noise (capacitors added)
- [ ] PID tuned for your motor
- [ ] Emergency stop timeout appropriate

## Tools You'll Need

### Essential (Buy First):
- **Multimeter**: $15-25 (measures voltage, continuity)
- **Arduino Uno**: $20 (for programming/testing)
- **Jumper Wires**: $5 (for connections)
- **LED + 220Ω resistor**: $1 (for signal testing)

### Helpful (Buy Next):
- **Oscilloscope**: $50-200 (for signal analysis)
- **Power Supply**: $30 (adjustable voltage/current)
- **Logic Analyzer**: $10-50 (for debugging signals)

### Nice to Have:
- **Soldering Iron**: $15 (for permanent connections)
- **Breadboard**: $5 (for prototyping)
- **Ferrite Beads**: $5 (for noise filtering)

## Documentation and Support

### Log Keeping (Important!)
Keep records of:
- [ ] Working hardware configurations
- [ ] Successful PID tunings (KP, KI, KD values)
- [ ] Component specifications and sources
- [ ] Problem symptoms and solutions
- [ ] Test results and measurements

### When to Ask for Help
**Seek help if:**
- Multiple symptoms occur simultaneously
- Smoke or burning smell from components
- Performance requirements can't be met
- Safety concerns with current setup
- You've tried basic fixes for 2+ hours

**Don't hesitate to ask for help - better safe than sorry!**

---

**Last Updated**: November 2025
**Version**: 2.0 (Beginner-Friendly Edition)
**Author**: PID Controller Development Team

## Learning Resources

### Beginner Electronics:
- [Arduino Official Tutorials](https://www.arduino.cc/en/Tutorial/HomePage)
- [SparkFun Electronics Tutorials](https://learn.sparkfun.com/tutorials)
- [Adafruit Learning System](https://learn.adafruit.com)

### Motor Control Specific:
- [BLDC Motor Fundamentals](https://www.monolithicpower.com/bldc-motor-fundamentals)
- [PID Controller Theory](https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/pid.pdf)
- [Hall Sensor Applications](https://www.allegromicro.com/en/insights-and-innovations/technical-documents/hall-effect-sensing-guide)

### Community Support:
- [Arduino Forum](https://forum.arduino.cc)
- [Reddit r/arduino](https://www.reddit.com/r/arduino)
- [Stack Exchange Electrical Engineering](https://electronics.stackexchange.com)

---

**Remember:** Electronics can be frustrating, but every expert was once a beginner. Take breaks, double-check connections, and celebrate small victories!
