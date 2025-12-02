# Contributing to BLDC Motor PID Controller

Thank you for your interest in contributing to the BLDC Motor PID Controller project! This document provides guidelines and information for contributors.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How to Contribute](#how-to-contribute)
- [Development Setup](#development-setup)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Submitting Changes](#submitting-changes)
- [Reporting Issues](#reporting-issues)
- [Documentation](#documentation)

## Code of Conduct

This project follows a code of conduct to ensure a welcoming environment for all contributors. By participating, you agree to:

- Be respectful and inclusive
- Focus on constructive feedback
- Accept responsibility for mistakes
- Show empathy towards other contributors
- Help create a positive community

## How to Contribute

There are several ways you can contribute to this project:

### Types of Contributions

1. **Bug Reports**: Report bugs and issues
2. **Feature Requests**: Suggest new features or improvements
3. **Code Contributions**: Submit pull requests with fixes or enhancements
4. **Documentation**: Improve documentation, tutorials, or examples
5. **Testing**: Test the code and report issues or suggest test cases

### Getting Started

1. Fork the repository on GitHub
2. Clone your fork locally
3. Create a new branch for your changes
4. Make your changes following the guidelines below
5. Test your changes thoroughly
6. Submit a pull request

## Development Setup

### Prerequisites

- **Arduino IDE** (version 1.8.19 or later) or **Arduino CLI**
- **ATTinyCore** for Arduino IDE (for ATtiny85 development)
- **AVR-GCC** toolchain (if using command line)
- **Git** for version control

### Environment Setup

1. **Install Arduino IDE:**
   ```bash
   # Ubuntu/Debian
   sudo apt-get install arduino

   # Or download from https://www.arduino.cc/en/software
   ```

2. **Install ATTinyCore:**
   - Open Arduino IDE
   - Go to File → Preferences → Additional Boards Manager URLs
   - Add: `https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json`
   - Tools → Board → Boards Manager → Search for "attiny" → Install

3. **Using Arduino CLI:**
   ```bash
   # Install Arduino CLI
   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

   # Install cores
   arduino-cli core install arduino:avr
   arduino-cli core install attiny:avr
   ```

### Building the Project

#### Arduino Uno Version
```bash
cd arduino_uno
arduino-cli compile --fqbn arduino:avr:uno arduino_uno.ino
```

#### ATtiny85 Version
```bash
cd attiny85
arduino-cli compile --fqbn attiny:avr:ATtinyX5:cpu=attiny85,clock=internal8 attiny85.ino
```

## Coding Standards

### C/C++ Style Guidelines

#### Naming Conventions
- **Variables**: `camelCase` (e.g., `pulseCount`, `currentRPM`)
- **Functions**: `camelCase` (e.g., `calculateRPM()`, `readPotentiometer()`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `CONTROL_LOOP_HZ`, `PID_OUTPUT_MAX`)
- **Types**: `PascalCase` for structs/classes (rare in Arduino)

#### Code Structure
- Use meaningful variable and function names
- Add comments for complex logic
- Keep functions focused on single responsibilities
- Use consistent indentation (2 or 4 spaces, match existing code)

#### Example Code Style
```c++
// Good: Clear naming and structure
volatile unsigned long pulseCount = 0;
unsigned long lastRPMCalcTime = 0;

float calculateRPM() {
    // Atomic read to avoid race conditions
    noInterrupts();
    unsigned long pulsesNow = pulseCount;
    interrupts();

    // Calculate RPM with proper units
    float rpm = (pulsesNow * 60000.0) / (timeDiff * PULSES_PER_REV);
    return rpm;
}

// Avoid: Unclear abbreviations or magic numbers
float calcRPM() {
    noInterrupts();
    unsigned long p = pulseCount;  // What does 'p' mean?
    interrupts();

    return (p * 60000) / (t * 6);  // Magic numbers
}
```

### Arduino-Specific Guidelines

#### Pin Definitions
```c++
// Good: Clear pin purpose and mode
#define RPM_SENSOR_PIN      2   // Interrupt pin for BLDC Hall sensor
#define PWM_OUTPUT_PIN      9   // PWM output to ESC
#define MODE_SWITCH_PIN     3   // Digital input for mode selection

// Use INPUT_PULLUP for switches/sensors
pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
```

#### Memory Management
- Be mindful of limited RAM (2KB on Arduino Uno, 512B on ATtiny85)
- Avoid large global arrays
- Use PROGMEM for large constant data
- Minimize string usage in loops

#### Interrupt Service Routines (ISRs)
- Keep ISRs short and fast
- Use volatile for shared variables
- Avoid function calls within ISRs
- Use atomic operations for multi-byte variables

### File Organization

```
BLDC_PID_Controller/
├── arduino_uno/                 # Arduino Uno implementation
│   ├── arduino_uno.ino         # Main sketch
│   ├── hardware_schematic.md   # Hardware setup
│   └── README.md               # Implementation details
├── attiny85/                   # ATtiny85 implementation
│   ├── attiny85.ino           # Main sketch
│   ├── ATTiny85_hardware_schematic.md
│   └── README.md
├── assets/
│   ├── 42BLF.pdf              # Motor datasheet
│   ├── High-level component diagram.png  # System diagrams
│   ├── Main control flow.png
│   └── Serial command sequence.png
├── CONTRIBUTING.md            # This file
├── README.md                  # Project overview
└── LICENSE                    # License information
```

## Testing

### Unit Testing
- Test individual functions with known inputs/outputs
- Verify PID calculations with manual calculations
- Test edge cases (zero RPM, maximum RPM, etc.)

### Integration Testing
- Test complete motor control loop
- Verify serial commands work correctly
- Test EEPROM parameter storage/loading
- Validate PWM output characteristics

### Hardware Testing Checklist

#### Basic Functionality
- [ ] Power supply connections (5V, GND)
- [ ] Hall sensor wiring (any A/B/C wire to input pin)
- [ ] PWM output to ESC signal pin
- [ ] Common ground between all components

#### Motor Control Testing
- [ ] Motor spins at low RPM without load
- [ ] PID maintains target RPM under varying load
- [ ] Serial plotting shows stable control
- [ ] PID control stable (no oscillations)

#### Parameter Tuning
- [ ] Potentiometer mode changes parameters
- [ ] Serial commands adjust PID values
- [ ] EEPROM saves/loads parameters
- [ ] Different pulse counts work correctly

#### Edge Cases
- [ ] Motor stall recovery
- [ ] Power interruption recovery
- [ ] Extreme temperature operation
- [ ] Electrical noise immunity

### Automated Testing
```bash
# Compile both versions
arduino-cli compile --fqbn arduino:avr:uno arduino_uno/arduino_uno.ino
arduino-cli compile --fqbn attiny:avr:ATtinyX5:cpu=attiny85,clock=internal8 attiny85/attiny85.ino

# Check for compilation errors
echo "Compilation successful!"
```

## Submitting Changes

### Pull Request Process

1. **Fork** the repository
2. **Create a feature branch** from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes** following the coding standards
4. **Test thoroughly** - both compilation and hardware testing
5. **Update documentation** if needed
6. **Commit with clear messages**:
   ```bash
   git commit -m "feat: add configurable pulse count per revolution

   - Add runtime-configurable PULSES_PER_REV parameter
   - Store in EEPROM for persistence
   - Add serial command SET PULSES <value>
   - Update documentation and help text

   Fixes #123"
   ```
7. **Push to your fork**:
   ```bash
   git push origin feature/your-feature-name
   ```
8. **Create a Pull Request** on GitHub

### Commit Message Guidelines

Follow conventional commit format:

```
type(scope): description

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Testing related changes
- `chore`: Maintenance tasks

Examples:
```
feat(pid): add anti-windup protection for integral term
fix(isr): prevent race conditions in pulse counting
docs(readme): update PWM specification details
```

## Reporting Issues

### Bug Reports

When reporting bugs, please include:

1. **Clear title** describing the issue
2. **Steps to reproduce** the problem
3. **Expected behavior** vs actual behavior
4. **Environment details**:
   - Arduino board type and version
   - Motor and ESC model
   - Arduino IDE version
   - Operating system
5. **Code snippets** if relevant
6. **Serial output** or error messages
7. **Hardware connections** diagram if possible

### Feature Requests

For feature requests, please include:

1. **Clear description** of the proposed feature
2. **Use case** - why is this feature needed?
3. **Implementation ideas** if you have any
4. **Impact assessment** - how does this affect existing functionality?

### Issue Templates

Use the following templates when creating issues:

#### Bug Report Template
```
## Bug Report

**Description:**
[Brief description of the bug]

**Steps to Reproduce:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Expected Behavior:**
[What should happen]

**Actual Behavior:**
[What actually happens]

**Environment:**
- Board: [Arduino Uno/ATtiny85]
- IDE Version: [1.8.x]
- OS: [Ubuntu 20.04/Windows 10]

**Additional Context:**
[Any other relevant information]
```

## Documentation

### Documentation Standards

- Use Markdown for all documentation files
- Keep README files in each implementation directory
- Include code examples where helpful
- Update documentation when code changes

### Documentation Files

- `README.md`: Project overview and getting started
- `arduino_uno/README.md`: Arduino Uno implementation details
- `attiny85/README.md`: ATtiny85 implementation details
- `arduino_uno/hardware_schematic.md`: Hardware setup guide
- `attiny85/ATTiny85_hardware_schematic.md`: ATtiny85 hardware guide
- `CONTRIBUTING.md`: This file

### Updating Documentation

When making changes that affect documentation:

1. Update relevant README files
2. Add code comments for new functions
3. Update hardware schematics if pinouts change
4. Test documentation instructions on a fresh setup

---

Thank you for contributing to the BLDC Motor PID Controller project! Your contributions help make this a better tool for the embedded systems community.

For questions or discussions, please open an issue on GitHub.
