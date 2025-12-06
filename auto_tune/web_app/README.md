# BLDC Motor PID Controller - Web Serial Direct Control

**Modern, clean interface for direct browser-to-Arduino connection using Web Serial API** - No backend server required!

## 🌐 Live Web Serial Control

This version connects directly to your Arduino using the Web Serial API, providing real-time hardware control from your browser. No server setup needed!

### ✨ Key Features
- **Direct Hardware Control**: Browser connects directly to Arduino serial port
- **Real-time PID Control**: Live parameter adjustment affects actual motor
- **No Backend Required**: Pure client-side application
- **Cross-platform**: Works on any device with compatible browser
- **Secure**: Requires explicit user permission for serial access

## 🚀 Quick Start

### 1. Hardware Setup
```bash
# Upload Arduino firmware
cd ../code
arduino-cli compile --fqbn arduino:avr:uno .
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno .
```

### 2. Browser Requirements
- **Chrome 89+** (Recommended)
- **Edge 89+**
- **Opera 75+**
- **Localhost**: No HTTPS required for development

### 3. Launch Control Interface
```bash
# Start local web server
python3 -m http.server 8000

# Or use any web server
# npx http-server -p 8000
```

### 4. Connect & Control
1. Open `http://localhost:8000/static/` in compatible browser
2. Click **"Connect to Arduino"**
3. Grant serial port permission when prompted
4. Select your Arduino port
5. **Real-time motor control begins!**

## 🔌 Web Serial API

### Browser Support
```javascript
// Check if Web Serial is supported
if (!('serial' in navigator)) {
    console.log('Web Serial API not supported');
}
```

### Connection Process
1. **Request Port**: `navigator.serial.requestPort()`
2. **Open Port**: `port.open({ baudRate: 115200 })`
3. **Read Data**: Continuous serial data reading
4. **Send Commands**: Direct serial command transmission

### Permission Flow
```
User clicks "Connect" → Browser shows port selection → User grants permission → Serial connection established → Real-time control
```

## 🎛️ Control Interface

### PID Parameters (Real-time)
- **Target RPM**: Set desired motor speed
- **Kp, Ki, Kd**: Live gain adjustment
- **Pulses/Rev**: Encoder configuration

### Motor Control
- **Enable/Disable**: Motor power control
- **Reset Controller**: PID state reset

### Data Visualization
- **Speed Chart**: Target vs actual RPM
- **Error Chart**: Control error over time
- **PID Output**: Controller response
- **Parameter Tracking**: Gain values history

## 📊 Real-time Data Flow

```
Arduino → Serial Port → Web Serial API → JavaScript Parser → Chart Updates → UI Display
```

### Data Format
```
STATUS:timestamp,target_rpm,current_rpm,error,pid_output,kp,ki,kd,pwm,ppr,motor_enabled
```

## 🛠️ Technical Architecture

### Client-Side Only
```
HTML/CSS/JS + Chart.js + Bootstrap 5 + Web Serial API
```

### Serial Communication
- **Baud Rate**: 115200 (matches Arduino)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### Data Processing
- **Real-time Parsing**: Continuous serial data processing
- **Chart Updates**: 10Hz visualization refresh
- **Parameter Sync**: Bidirectional control updates

## 🔧 Browser Compatibility

### ✅ Supported Browsers
| Browser | Version | Status |
|---------|---------|--------|
| Chrome | 89+ | ✅ Full Support |
| Edge | 89+ | ✅ Full Support |
| Opera | 75+ | ✅ Full Support |

### ❌ Unsupported Browsers
- Firefox (No Web Serial support)
- Safari (No Web Serial support)
- Mobile Safari (No Web Serial support)

### Development Notes
- **Localhost**: Works without HTTPS
- **HTTPS Required**: For production deployment
- **Permissions**: Must be granted by user
- **Single Port**: Only one serial connection per page

## 🚀 Deployment Options

### Local Development
```bash
# Python server
python3 -m http.server 8000

# Node.js server
npx http-server -p 8000

# PHP server
php -S localhost:8000
```

### Production Deployment
- Requires **HTTPS** for Web Serial API
- Use web servers like Apache, Nginx, or cloud hosting
- Ensure proper CORS and security headers

## 🐛 Troubleshooting

### Connection Issues
```javascript
// Check console for these errors:
"NotFoundError" // No port selected
"NotAllowedError" // Permission denied
"InvalidStateError" // Port already open
```

### Arduino Communication
- **Verify firmware** is uploaded correctly
- **Check serial port** permissions (`sudo chmod 666 /dev/ttyACM0`)
- **Monitor Arduino output** in serial monitor
- **Verify baud rate** matches (115200)

### Browser Issues
- **Enable experimental features** in Chrome flags
- **Check developer console** for JavaScript errors
- **Verify HTTPS** for production use
- **Test on supported browsers** only

## 📁 Project Structure

```
web_app/
├── static/
│   ├── index.html    # Main control interface
│   ├── app.js        # Web Serial logic + charts
│   └── style.css     # Bootstrap + custom styles
└── data/             # Parameter storage (if needed)
```

## 🔄 Comparison with Other Versions

| Version | Connection | Server | Real Hardware |
|---------|------------|--------|---------------|
| **Web Serial** | Direct Serial | ❌ None | ✅ Yes |
| **Full Web App** | WebSocket | ✅ FastAPI | ✅ Yes |
| **Desktop App** | PySerial | ❌ Local | ✅ Yes |
| **Static Demo** | Simulated | ❌ None | ❌ No |

## 🎨 Modern UI Design

### Clean & Intuitive Interface
- **System Status Bar**: Real-time connection, motor, and RPM display
- **Panel-Based Layout**: Organized control sections
- **Modern Typography**: Clear hierarchy and readability
- **Responsive Design**: Works on desktop, tablet, and mobile
- **Minimal Emojis**: Professional appearance
- **Smooth Animations**: Subtle transitions and hover effects

### Enhanced User Experience
- **Visual Status Indicators**: Color-coded connection and motor states
- **Contextual Controls**: Buttons change based on current state
- **Smart Error Handling**: Clear messages and recovery options
- **Accessibility**: Keyboard navigation and screen reader friendly

## 🎯 Use Cases

### Perfect For:
- **Educational Projects**: Learn PID control with real hardware
- **Rapid Prototyping**: Quick Arduino testing and tuning
- **Remote Control**: Network-accessible motor control
- **Development Testing**: No server setup required
- **Portfolio Projects**: Impressive real-time control demo

### Best Practices:
- **Local Development**: Use for initial testing
- **HTTPS Production**: Required for deployed versions
- **User Permissions**: Handle gracefully
- **Error Recovery**: Implement connection recovery
- **Cross-browser**: Provide fallbacks

## 🚀 Getting Started

1. **Upload Arduino firmware**
2. **Open interface in supported browser**
3. **Connect to Arduino**
4. **Tune PID parameters**
5. **Control your motor!**

---

**🎉 Zero Backend Architecture - Direct Browser Hardware Control!**

**Questions?** Check the troubleshooting section or browser console for detailed error messages.
