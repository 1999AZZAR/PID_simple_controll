# BLDC Motor PID Controller - PC-Powered Auto-Tune

A modern, powerful PID tuning application that leverages your **PC's computational power** for superior motor control stability and easier tuning.

## 🚀 Key Features

### One-Click Auto-Tuning
- **Ziegler-Nichols** method with automatic oscillation detection
- **Multiple tuning variants** for different response characteristics
- **Step response analysis** for system identification
- Automatically finds optimal Kp, Ki, Kd parameters

### PC-Side Signal Processing
- **Kalman filtering** for smooth, noise-free RPM readings
- Real-time **hunting detection** (oscillation detection)
- **Stability scoring** (0-100%) with visual feedback
- Advanced filtering that outperforms Arduino-only solutions

### Real-Time Analysis
- Live **overshoot** and **settling time** measurement
- **Step response recording** with analysis
- Automatic **tuning suggestions** based on system behavior
- Export data to CSV for further analysis

### Modern Dark-Theme Interface
- Beautiful Catppuccin-inspired dark theme
- Real-time 4-panel plotting
- Slider + spinbox controls for precise adjustment
- Tuning history with one-click apply

## 📸 Screenshots

The GUI provides:
- **Live Data Tab**: Real-time plots of RPM, Error, PID Output, and Stability Score
- **Analysis Tab**: System analysis report with tuning suggestions
- **Quick Actions**: One-click motor control and auto-tuning
- **Real-Time Metrics**: Instant feedback on system performance

## 🔧 Installation

### 1. Install Python Dependencies

```bash
cd auto_tune
pip install -r requirements.txt
```

Required packages:
- PyQt6 (GUI framework)
- matplotlib (plotting)
- pyserial (Arduino communication)
- numpy (data processing)
- scipy (optional, for advanced analysis)

### 2. Upload Arduino Firmware

1. Open `code/code.ino` in Arduino IDE
2. Select your Arduino board and port
3. Upload the sketch

### 3. Run the GUI

```bash
python control.py
```

## 🎯 Quick Start Guide

### First Time Setup

1. **Connect your Arduino** via USB
2. **Launch the GUI**: `python control.py`
3. **Select port** from the dropdown (Arduino ports shown with 🔌)
4. Click **Connect**

### Basic Motor Control

1. Set your **Target RPM** (default: 1440)
2. Set **Pulses/Rev** for your motor (check motor specs)
3. Click **Motor ON** to start
4. Watch the real-time plots!

### One-Click Auto-Tune (Recommended!)

1. Connect and verify motor is working
2. Click **🎯 One-Click Auto-Tune**
3. Wait ~30 seconds while the system:
   - Starts the motor
   - Gradually increases Kp to find oscillation point
   - Calculates optimal PID parameters
   - Applies the tuned values
4. Done! Monitor stability score

### Manual Tuning

Use the sliders/spinboxes for:
- **Kp**: Proportional gain (start low: 0.1-0.3)
- **Ki**: Integral gain (very small: 0.005-0.02)
- **Kd**: Derivative gain (damping: 0.01-0.05)

Watch the **Stability Score** - aim for >80%

## 📊 Understanding the Display

### Real-Time Metrics

| Metric | Description | Good Value |
|--------|-------------|------------|
| Current RPM | Raw sensor reading | Close to target |
| Filtered RPM | Kalman-filtered value | Smoother, more accurate |
| Error | Target - Current | Near 0 |
| Overshoot | Peak above target | < 10% |
| Stability | Overall health score | > 80% |
| Status | STABLE / SETTLING / HUNTING | STABLE |

### Status Indicators

- 🟢 **STABLE**: System is well-tuned
- 🟡 **SETTLING**: System is adjusting (normal after changes)
- 🔴 **HUNTING**: System is oscillating - reduce Kp!

## 🔬 Advanced Features

### Step Response Analysis

1. Enable motor with current parameters
2. Go to **Analysis** tab
3. Click **📊 Record Step Response**
4. Wait 5-10 seconds
5. Click **🔬 Analyze & Suggest**

The analysis will show:
- Rise time
- Overshoot percentage
- Settling time
- Tuning recommendations

### Tuning History

Every auto-tune result is saved to the history table. Click any row to instantly apply those parameters.

### Data Export

Click **💾 Export Data** to save a CSV with:
- Timestamp, Target RPM, Raw RPM, Filtered RPM
- Error, PID Output, Stability Score

## ⚙️ Configuration Reference

### Pulses Per Revolution (PPR)

Common values:
| Motor Type | PPR |
|------------|-----|
| 8-pole (4 pole pairs) | 4 |
| 6-pole (3 pole pairs) | 3 |
| 3-Hall with OR gate | 24 |

### PID Parameter Ranges

| Parameter | Typical Range | Effect |
|-----------|---------------|--------|
| Kp | 0.1 - 1.0 | Higher = faster response, but can cause oscillation |
| Ki | 0.001 - 0.05 | Higher = eliminates steady-state error, but slow |
| Kd | 0.005 - 0.05 | Higher = more damping, reduces overshoot |

## 🚑 Troubleshooting

### Problem: Hunting / Oscillation
**Symptoms**: Motor speed swings up and down, Status shows "HUNTING"

**Solutions**:
1. Reduce Kp by 20-30%
2. Increase Kd slightly
3. Try the auto-tune with "No Overshoot" variant

### Problem: Sluggish Response
**Symptoms**: Takes too long to reach target, Stability shows "SETTLING" for a long time

**Solutions**:
1. Increase Kp slightly
2. Increase Ki (but be careful!)

### Problem: Steady-State Error
**Symptoms**: RPM stabilizes but not at target

**Solutions**:
1. Increase Ki slightly
2. Verify PPR setting matches your motor

### Problem: No Data Received
**Symptoms**: Plots are empty, "No data from Arduino" warning

**Solutions**:
1. Check USB connection
2. Verify correct port selected
3. Check baud rate (should be 115200)
4. Close Arduino IDE Serial Monitor

## 🏗️ How It Works

### PC-Side Processing

The PC handles computationally intensive tasks:

1. **Kalman Filter**: Uses predictive filtering to smooth noisy RPM readings
2. **Stability Analysis**: Calculates metrics from signal characteristics
3. **Auto-Tuning Algorithms**: Implements Z-N, Cohen-Coon, IMC methods
4. **Real-Time Plotting**: matplotlib with hardware acceleration

### Arduino Responsibilities

The Arduino handles time-critical tasks:

1. **Hall Sensor ISR**: Microsecond-accurate pulse timing
2. **PWM Output**: Direct ESC control
3. **Serial Communication**: Status updates at 10-20Hz

### Communication Protocol

```
PC → Arduino:
  SET_KP 0.5
  SET_KI 0.02
  SET_KD 0.01
  SET_TARGET_RPM 1440
  SET_PULSES_PER_REV 4
  ENABLE_MOTOR 1
  RESET_CONTROLLER
  GET_STATUS

Arduino → PC:
  STATUS:timestamp,target,current,error,pid_out,kp,ki,kd,pwm,ppr,enabled
```

## 📋 Version History

### v2.0 (December 2025)
- Complete rewrite with PC-powered processing
- One-click auto-tuning
- Kalman filtering
- Dark theme UI
- Real-time stability analysis

### v1.0 (November 2025)
- Initial PyQt6 interface
- Basic serial communication
- Manual PID tuning

## 📄 License

MIT License - See LICENSE file in repository root.

## 👥 Authors

- **azzar budiyanto** - Hardware design and testing
- **azzar persona (AI assistant)** - Software development
