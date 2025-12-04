# Python GUI PID Controller - System Flowchart Documentation

## Executive Summary

This flowchart documents the complete operational flow of the PyQt6-based Python GUI application for BLDC motor PID controller configuration and real-time monitoring. The system provides professional parameter tuning, live visualization, and bidirectional communication with Arduino-based motor controllers.

## System Overview

### Primary Function
The Python GUI PID controller implements a modern, responsive interface for configuring and monitoring Arduino-based BLDC motor controllers, providing real-time parameter adjustment and live performance visualization.

### Key Features
- **Modern PyQt6 Interface**: Professional GUI with dual-control system (sliders + spinboxes)
- **Real-time Serial Communication**: Robust bidirectional communication with Arduino
- **Live Data Visualization**: Four synchronized matplotlib plots with automatic updates
- **Parameter Management**: Save/load configurations and data export capabilities
- **PPR Testing**: Built-in pulse counting verification for accurate RPM calculation
- **Cross-Platform Compatibility**: Windows, macOS, and Linux support

## Flowchart Structure

### Main Components

#### 1. Application Initialization (`__init__()` function)

**GUI Framework Setup:**
```
Python GUI Application Start
├── Import Dependencies (PyQt6, matplotlib, pyserial, numpy)
├── Initialize QApplication Instance
├── Create Main Window (PIDControllerGUI)
├── Setup Current Parameters Dictionary
│   ├── target_rpm: 1440.0
│   ├── kp: 0.25, ki: 0.015, kd: 0.003
│   ├── pulses_per_rev: 18
│   └── motor_enabled: True
├── Initialize Serial Connection Monitoring
├── Setup Plot Update Optimization
└── Create Central Widget with Splitter Layout
```

#### 2. GUI Construction Pipeline

**Interface Building Sequence:**
```
create_control_panel()
├── Serial Connection Group
│   ├── Port Selection ComboBox
│   ├── Baud Rate ComboBox (115200 default)
│   ├── Connect/Disconnect Button
│   └── Refresh Ports Button
├── PID Tuning Group
│   ├── Target RPM Spinbox (0-5000 range)
│   ├── PID Parameter Controls (Kp, Ki, Kd)
│   │   ├── Slider + Spinbox Dual Control
│   │   └── Real-time Value Synchronization
│   └── PPR Spinbox (1-100 range)
├── Motor Control Group
│   ├── Enable/Disable Motor Toggle Button
│   └── Reset Controller Button
├── Control Group
│   └── Test PPR Button
└── Status Label (Connection Feedback)

create_plot_panel()
├── Matplotlib Figure Canvas (4 subplots)
├── Navigation Toolbar
├── Plot Setup (Motor Speed, Error, PID Output, Gains)
└── Data Storage Arrays (25-point buffer)
```

#### 3. Serial Communication System

**Worker Thread Architecture:**
```
SerialWorker Thread (QThread)
├── Connection Establishment
│   ├── Port/Baud Parameter Validation
│   ├── Serial Port Opening (exclusive access)
│   └── Connection Status Emission
├── Data Reception Loop
│   ├── Raw Data Reading (buffered)
│   ├── Line Parsing (newline delimited)
│   ├── STATUS Data Recovery
│   │   ├── Partial Line Reconstruction
│   │   └── Corrupted Data Filtering
│   └── Data Queue Emission
├── Command Transmission
│   ├── Parameter Validation
│   └── Error Handling
└── Cleanup on Thread Exit
```

## Operational Flow

### Main Application Loop

**Qt Event-Driven Processing:**
```
Main Event Loop (QApplication.exec())
├── User Interface Events
│   ├── Parameter Adjustments (sliders/spinboxes)
│   ├── Button Clicks (connect, motor control)
│   ├── Menu Actions (save/load/export)
│   └── Window Events (resize, close)
├── Serial Data Processing (10Hz timer)
│   ├── Queue Data Retrieval
│   ├── STATUS Message Parsing
│   └── Plot Data Updates (5Hz filtered)
├── Plot Updates (matplotlib)
│   ├── Data Point Management (25-point buffer)
│   ├── Real-time Plot Refresh
│   └── Performance Optimization
└── Connection Monitoring (timeout handling)
```

### Serial Communication Protocol

#### Command Transmission Flow
```
Parameter Change Event → Command Formatting → Serial Transmission → Arduino Processing → Status Response → GUI Update → Plot Refresh
```

#### Data Reception Pipeline
```
Serial Data → Buffer Processing → Line Extraction → Format Validation → STATUS Parsing → Parameter Updates → Plot Data Addition → Visual Refresh
```

## PID Control Integration

### Parameter Synchronization
**Bidirectional Parameter Flow:**
```
GUI Parameter Change
├── Immediate Local Update (UI feedback)
├── Serial Command Transmission
│   ├── SET_KP <value>
│   ├── SET_KI <value>
│   ├── SET_KD <value>
│   ├── SET_TARGET_RPM <value>
│   └── SET_PULSES_PER_REV <value>
├── Arduino EEPROM Storage
├── STATUS Response Reception
└── GUI State Synchronization
```

### Real-time Monitoring System

**Four-Plot Visualization:**
```
Motor Speed Plot (Target vs Current RPM)
├── Red Line: Target RPM (setpoint)
├── Blue Line: Current RPM (process variable)
└── Performance Indicator: Steady-state accuracy

Control Error Plot (RPM difference)
├── Green Line: Error = Target - Current
├── Zero Crossing: Optimal performance
└── Stability Indicator: Oscillation analysis

PID Output Plot (Controller effort)
├── Magenta Line: PID algorithm output
├── Range: -1000 to +1000 (scaled)
└── Control Signal: PWM duty cycle precursor

PID Gains Plot (Parameter tracking)
├── Red Line: Kp (proportional gain)
├── Green Line: Ki (integral gain)
├── Blue Line: Kd (derivative gain)
└── Parameter History: Tuning session analysis
```

## Data Management Architecture

### Configuration Persistence
**JSON-Based Parameter Storage:**
```
Parameter Save Operation
├── Current Parameters Collection
├── JSON Serialization
├── File Dialog (user selection)
├── File Writing with Error Handling
└── Success Confirmation Dialog

Parameter Load Operation
├── File Selection Dialog
├── JSON Deserialization
├── Parameter Validation
├── GUI Control Updates
├── Arduino Command Transmission
└── State Synchronization Confirmation
```

### Data Export System
**CSV Export for Analysis:**
```
Data Export Process
├── Plot Data Validation (non-empty check)
├── CSV Header Generation
│   ├── Time, Target_RPM, Current_RPM, Error
│   └── PID_Output, Kp, Ki, Kd
├── Data Point Iteration
├── Formatted Row Writing
├── File Completion
└── Success Feedback
```

## Error Handling & Recovery

### Critical Error Conditions
- **Serial Connection Loss**: Automatic reconnection attempts
- **Data Parsing Errors**: Graceful fallback to last valid values
- **Plot Update Failures**: Exception handling with recovery
- **Parameter Validation**: Range checking and constraint enforcement
- **Thread Communication**: Signal/slot error recovery

### Recovery Mechanisms
- **Connection Timeout**: 10-second monitoring with status updates
- **Partial Data Recovery**: STATUS line reconstruction from fragments
- **GUI State Preservation**: Parameter retention across disconnections
- **Automatic Reconnection**: Port availability monitoring
- **Graceful Degradation**: Continued operation with cached parameters

## Performance Characteristics

### Timing Specifications
- **GUI Update Rate**: 10Hz (100ms intervals)
- **Plot Refresh Rate**: 4-5Hz (filtered updates)
- **Serial Processing**: 20Hz (50ms intervals)
- **Data Buffer**: 25 points (optimized memory usage)
- **Connection Monitoring**: 1Hz (status checks)

### Resource Utilization
- **Memory Usage**: 50-100MB (Python + PyQt6 + matplotlib)
- **CPU Usage**: 5-15% (depends on plot complexity)
- **Disk I/O**: Minimal (parameter files, data export)
- **Network**: Serial communication only

## Testing & Validation

### Serial Testing Framework
**Built-in Testing Capabilities:**
```
Connection Testing
├── Port Availability Verification
├── Baud Rate Compatibility Check
├── Basic Command/Response Validation
└── Communication Reliability Assessment

Parameter Testing
├── PID Gain Setting Verification
├── Target RPM Command Testing
├── PPR Configuration Validation
└── Motor Control Command Testing

PPR Calculation Testing
├── 5-Second Pulse Counting Period
├── RPM Calculation from Pulse Count
├── PPR Accuracy Verification
├── Motor Speed Comparison
└── Formula Validation: RPM = (pulses × 60) / (time_seconds × PPR)
```

### System Validation Procedures
**Comprehensive Testing Protocol:**
1. **Hardware Connection**: Arduino Uno with BLDC motor/ESC
2. **Serial Communication**: Port detection and connection establishment
3. **Parameter Setting**: PID gains and target RPM configuration
4. **Motor Control**: Enable/disable and reset functionality
5. **Data Acquisition**: Real-time plotting and data integrity
6. **Performance Analysis**: Stability, response time, steady-state error
7. **Data Export**: CSV generation and external analysis compatibility

## Integration Points

### Hardware Dependencies
- **Arduino Uno**: Firmware-compatible controller with serial interface
- **BLDC Motor**: 3-Hall sensor configuration for RPM feedback
- **ESC**: PWM input compatible (0-255 range, 490Hz frequency)
- **Serial Interface**: USB-to-serial conversion for communication

### Software Interfaces
- **PyQt6 Framework**: Cross-platform GUI development
- **Matplotlib**: Real-time plotting and data visualization
- **PySerial**: Serial communication protocol handling
- **NumPy**: Numerical data processing and analysis

### Operating System Integration
- **Windows**: COM port detection and access
- **macOS**: `/dev/tty.*` device handling and permissions
- **Linux**: `/dev/ttyACM*` and `/dev/ttyUSB*` management

## Maintenance & Troubleshooting

### Common Issues

#### GUI Startup Problems
- **Missing Dependencies**: `pip install PyQt6 matplotlib pyserial numpy`
- **Python Version**: Ensure Python 3.7+ compatibility
- **Import Errors**: Virtual environment activation or path issues
- **Display Issues**: Qt platform plugin problems on Linux

#### Serial Communication Issues
- **Port Not Found**: Check Arduino connection and driver installation
- **Permission Denied**: Linux dialout group membership required
- **Busy Port**: Close Arduino IDE or other serial applications
- **Baud Rate Mismatch**: Verify 115200 baud rate configuration

#### Plotting Performance Issues
- **Slow Updates**: Reduce plot buffer size (currently 25 points)
- **High CPU Usage**: Decrease update frequency or disable plots
- **Memory Growth**: Clear plot data periodically during long sessions
- **Display Freezing**: matplotlib backend compatibility issues

### Diagnostic Procedures

#### Serial Communication Diagnostics
```python
# Manual serial testing
import serial
try:
    ser = serial.Serial('COM3', 115200, timeout=1)  # Windows
    # ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Linux
    ser.write(b'GET_STATUS\n')
    response = ser.readline()
    print(f"Arduino Response: {response}")
    ser.close()
except Exception as e:
    print(f"Serial Error: {e}")
```

#### Performance Monitoring
```python
# Monitor GUI performance
import psutil
import os

process = psutil.Process(os.getpid())
print(f"Memory Usage: {process.memory_info().rss / 1024 / 1024:.1f} MB")
print(f"CPU Usage: {process.cpu_percent()}%")
```

#### Log Analysis
- **Serial Logs**: Monitor Arduino communication patterns
- **Error Logs**: Track GUI exceptions and recovery attempts
- **Performance Logs**: Analyze timing and resource usage
- **Data Logs**: Verify STATUS message integrity

## Development History

### Version Evolution
- **Initial Implementation**: Basic PyQt5 interface with serial communication
- **PyQt6 Migration**: Modern framework adoption with improved styling
- **Performance Optimization**: Plot buffering and update rate optimization
- **Testing Integration**: Built-in serial testing and PPR verification
- **Cross-Platform Enhancement**: Comprehensive OS-specific installation guides

### Code Quality Standards
- **Memory Efficiency**: Optimized data structures and garbage collection
- **Real-time Performance**: Deterministic GUI response and serial processing
- **Error Resilience**: Comprehensive exception handling and recovery
- **User Experience**: Professional interface with intuitive controls
- **Maintainability**: Clear code structure and comprehensive documentation

---

## Technical Reference

### Configuration Constants
```python
# GUI Configuration
class PIDControllerGUI:
    PLOT_MAX_POINTS = 25          # Maximum data points in plots
    SERIAL_TIMEOUT = 10           # Connection timeout (seconds)
    UPDATE_INTERVAL = 100         # GUI update rate (ms)
    PLOT_UPDATE_DIVIDER = 5       # Plot update frequency divider

    # Parameter Ranges
    TARGET_RPM_RANGE = (0, 5000)
    KP_RANGE = (0.0, 5.0)
    KI_RANGE = (0.0, 1.0)
    KD_RANGE = (0.0, 0.1)
    PPR_RANGE = (1, 100)
```

### Function Call Hierarchy
```
__init__()
├── setupUi() - Interface creation
├── setupConnections() - Signal/slot connections
├── SerialWorker() - Communication thread
└── QTimer setup - Data processing timers

process_serial_data()
├── Queue data retrieval
├── STATUS message parsing
├── Parameter synchronization
└── Plot data updates

update_plot()
├── Data point management
├── Plot clearing and redrawing
├── Axis scaling and labeling
└── Performance optimization
```

### Data Structures
```python
# Current Parameters Dictionary
self.current_params = {
    'target_rpm': float,
    'kp': float, 'ki': float, 'kd': float,
    'pulses_per_rev': int,
    'motor_enabled': bool
}

# Plot Data Storage
self.time_data = []          # Timestamp array
self.target_rpm_data = []    # Target RPM history
self.current_rpm_data = []   # Current RPM history
self.error_data = []         # Error signal history
self.pid_output_data = []    # PID output history
self.kp_data = []           # Kp gain history
self.ki_data = []           # Ki gain history
self.kd_data = []           # Kd gain history
```

---

*This flowchart documentation provides complete system visibility for the Python GUI PID controller, enabling effective development, testing, maintenance, and user support of the BLDC motor speed control system.*

**Author**: azzar budiyanto
**Co-Author**: azzar persona (AI assistant)
**Date**: December 2025
**Documentation Standard**: Professional Enterprise Flowchart
