# BLDC Motor PID Controller - PC-Powered Web Interface

Modern browser-based PID tuning with Kalman filtering, auto-tuning, and real-time stability analysis - No backend server required!

## Key Features

### PC-Powered Processing (in your browser!)
- Kalman Filtering: Smooth, noise-free RPM readings computed client-side
- One-Click Auto-Tune: Automatic PID optimization using Ziegler-Nichols method
- Hunting Detection: Real-time oscillation detection and alerts
- Stability Scoring: 0-100% system health metric

### Direct Web Serial Connection
- Connects directly to Arduino via Web Serial API
- No Python backend needed
- Works on any compatible browser (Chrome, Edge, Opera)

## Interface Overview

The web interface provides:
- System Status Bar: Real-time metrics including raw/filtered RPM, error, and stability
- Live Charts: Speed (with Kalman filtering), Error, and Stability Score
- One-Click Auto-Tune: Automatic PID parameter optimization
- Tuning History: Quick access to previous tuning results
- Real-Time Analysis: Suggestions based on system behavior

## Quick Start

### 1. Upload Arduino Firmware

```bash
cd ../code
# Using Arduino IDE or arduino-cli
arduino-cli compile --fqbn arduino:avr:uno .
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno .
```

### 2. Start Web Server

```bash
# Simple Python server
python3 -m http.server 8000

# Or Node.js
npx http-server -p 8000
```

### 3. Open in Browser

Open `http://localhost:8000/static/` in:
- Chrome 89+
- Edge 89+
- Opera 75+

### 4. Connect & Control

1. Click "Connect to Arduino"
2. Grant serial port permission
3. Select your Arduino port
4. Set Pulses Per Revolution (PPR) for your motor
5. Click "Enable Motor"
6. Watch the Kalman-filtered data!

## One-Click Auto-Tune

The auto-tune feature uses the Ziegler-Nichols method:

1. Click "One-Click Auto-Tune"
2. System automatically:
   - Enables motor with conservative gains
   - Gradually increases Kp
   - Detects oscillation point (ultimate gain Ku)
   - Calculates ultimate period Tu
   - Computes optimal Kp, Ki, Kd
3. New parameters applied automatically!

### Tuning Methods Available

| Method | Best For |
|--------|----------|
| Z-N No Overshoot | Motor control (default) |
| Z-N Classic | Faster response |

## Understanding the Display

### Status Indicators

| Status | Color | Meaning |
|--------|-------|---------|
| STABLE | Green | System well-tuned |
| SETTLING | Yellow | Adjusting to target |
| HUNTING | Red | Oscillation detected - reduce Kp! |

### Real-Time Metrics

- Raw RPM: Direct sensor reading
- Filtered RPM: Kalman-filtered (smoother, more accurate)
- Error: Target - Current RPM
- Stability: 0-100% health score

### Kalman Filter Benefits

The browser-based Kalman filter provides:
- Superior noise rejection compared to moving average
- Predictive filtering for smoother readings
- Lower latency than Arduino-based filtering

## Analysis Features

Click "Analyze & Suggest" to get:
- Current system metrics
- Overshoot percentage
- Oscillation frequency
- Tuning recommendations

### Example Suggestions

> WARNING: Hunting Detected
> System is oscillating. Reduce Kp by 20-30% and/or increase Kd.

> SUCCESS: System Well-Tuned
> Stability score: 85%. Fine-tune with Kp for response, Kd for damping.

## Troubleshooting

### Browser Shows "Not Supported"
- Use Chrome, Edge, or Opera
- Firefox/Safari don't support Web Serial API

### No Data After Connecting
- Arduino resets when serial opens - wait 2-3 seconds
- Click "Request Status" to manually request data
- Check Arduino power and USB connection

### Hunting/Oscillation
1. Reduce Kp by 20-30%
2. Increase Kd slightly
3. Use Auto-Tune with "No Overshoot" mode

### Connection Lost
- Arduino may have reset
- Click "Reconnect" button
- Check USB cable

## File Structure

```
web_app/
├── static/
│   ├── index.html    # Main interface
│   ├── app.js        # Kalman filter, auto-tune, charts
│   └── style.css     # Catppuccin dark theme
└── README.md         # This file
```

## Theme

Uses Catppuccin Mocha color palette for a beautiful dark theme that's easy on the eyes during extended tuning sessions.

## Serial Protocol

```
Browser → Arduino:
  SET_KP 0.3
  SET_KI 0.02
  SET_KD 0.02
  SET_TARGET_RPM 1440
  SET_PULSES_PER_REV 4
  ENABLE_MOTOR 1
  RESET_CONTROLLER
  GET_STATUS

Arduino → Browser:
  STATUS:timestamp,target,current,error,pid_out,kp,ki,kd,pwm,ppr,enabled
```

## Comparison: Web vs Desktop

| Feature | Web Interface | Desktop (PyQt6) |
|---------|---------------|-----------------|
| Installation | None (browser) | pip install |
| Kalman Filter | JavaScript | Python |
| Auto-Tune | Client-side | PC-side |
| Charts | Chart.js | Matplotlib |
| Platform | Any (Chrome/Edge) | Windows/Mac/Linux |
| Server | None needed | None needed |

## License

MIT License - See LICENSE file in repository root.

## Authors

- azzar budiyanto - Hardware design and testing
- azzar persona (AI assistant) - Software development

---

Zero Backend - All processing in your browser!
