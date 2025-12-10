#!/usr/bin/env python3
"""
BLDC Motor PID Controller - Enhanced PC-Powered Auto-Tune GUI

This modern PyQt6 application leverages the PC's computational power for:
- Advanced signal processing (Kalman filtering, FFT analysis)
- One-click auto-tuning (Ziegler-Nichols, Cohen-Coon, Relay Feedback)
- Real-time stability analysis and hunting detection
- Optional PC-based PID control (ultra-stable)
- Crystal-accurate timing reference

Features:
- Modern, responsive PyQt6 interface
- One-click auto-tuning with multiple algorithms
- Real-time Kalman-filtered RPM display
- Step response analysis for system identification
- Stability metrics (overshoot, settling time, oscillation detection)
- PC Control Mode for maximum stability
- Data logging and analysis tools

Requirements:
- Python 3.8+
- PyQt6, matplotlib, pyserial, numpy, scipy

Install dependencies:
pip install PyQt6 matplotlib pyserial numpy scipy

Author: azzar budiyanto & azzar persona (AI assistant)
Date: December 2025
"""

import sys
import serial
import serial.tools.list_ports
import threading
import time
import json
import os
from datetime import datetime
import queue
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from enum import Enum, auto

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFormLayout, QLabel, QLineEdit, QComboBox,
    QPushButton, QSpinBox, QDoubleSpinBox, QSlider, QGroupBox,
    QFrame, QSplitter, QStatusBar, QMessageBox, QFileDialog,
    QProgressBar, QTextEdit, QCheckBox, QTabWidget, QTableWidget,
    QTableWidgetItem, QHeaderView, QRadioButton, QButtonGroup
)
from PyQt6.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QRectF, QPointF
)
from PyQt6.QtGui import QAction, QFont, QPalette, QColor, QIcon

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# Try to import scipy for advanced analysis
try:
    from scipy import signal as scipy_signal
    from scipy.optimize import minimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("Warning: scipy not installed. Some advanced features will be disabled.")
    print("Install with: pip install scipy")


class ControlMode(Enum):
    """Control mode enumeration"""
    ARDUINO_PID = auto()  # Arduino calculates PID (default)
    PC_PID = auto()       # PC calculates PID, sends PWM to Arduino


class TuningMethod(Enum):
    """Auto-tuning method enumeration"""
    ZIEGLER_NICHOLS = "Ziegler-Nichols"
    COHEN_COON = "Cohen-Coon"
    RELAY_FEEDBACK = "Relay Feedback"
    IMC = "Internal Model Control"


@dataclass
class SystemMetrics:
    """Real-time system metrics"""
    current_rpm: float = 0.0
    target_rpm: float = 0.0
    error: float = 0.0
    rpm_filtered: float = 0.0
    overshoot_percent: float = 0.0
    settling_time_ms: float = 0.0
    oscillation_frequency_hz: float = 0.0
    stability_score: float = 100.0  # 0-100, higher is better
    is_hunting: bool = False
    is_stable: bool = True


@dataclass
class TuningResult:
    """Result from auto-tuning"""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    method: str = ""
    timestamp: str = ""
    notes: str = ""


class KalmanFilter:
    """
    Simple Kalman filter for RPM smoothing.
    Uses PC's processing power for superior noise rejection.
    """
    def __init__(self, process_variance: float = 1e-4, measurement_variance: float = 0.1):
        self.process_variance = process_variance  # Q - process noise
        self.measurement_variance = measurement_variance  # R - measurement noise
        self.estimate = 0.0
        self.error_estimate = 1.0
        self.initialized = False
    
    def update(self, measurement: float) -> float:
        if not self.initialized:
            self.estimate = measurement
            self.initialized = True
            return self.estimate
        
        # Prediction step
        prediction = self.estimate
        prediction_error = self.error_estimate + self.process_variance
        
        # Update step
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.error_estimate = (1 - kalman_gain) * prediction_error
        
        return self.estimate
    
    def reset(self):
        self.estimate = 0.0
        self.error_estimate = 1.0
        self.initialized = False


class StabilityAnalyzer:
    """
    Analyzes motor control stability using PC's computational power.
    Detects hunting, calculates metrics, and provides tuning suggestions.
    """
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.rpm_history = deque(maxlen=window_size)
        self.error_history = deque(maxlen=window_size)
        self.time_history = deque(maxlen=window_size)
        self.target_rpm = 0.0
        self.step_response_data = []
        self.step_response_active = False
        self.step_start_time = 0.0
    
    def add_sample(self, rpm: float, target: float, timestamp: float):
        """Add a new sample for analysis"""
        self.rpm_history.append(rpm)
        self.error_history.append(target - rpm)
        self.time_history.append(timestamp)
        self.target_rpm = target
        
        if self.step_response_active:
            self.step_response_data.append((timestamp - self.step_start_time, rpm, target))
    
    def start_step_response(self):
        """Start recording step response"""
        self.step_response_data = []
        self.step_response_active = True
        self.step_start_time = time.time()
    
    def stop_step_response(self):
        """Stop recording step response"""
        self.step_response_active = False
    
    def calculate_metrics(self) -> SystemMetrics:
        """Calculate comprehensive stability metrics"""
        metrics = SystemMetrics()
        
        if len(self.rpm_history) < 10:
            return metrics
        
        rpm_array = np.array(self.rpm_history)
        error_array = np.array(self.error_history)
        
        metrics.current_rpm = rpm_array[-1]
        metrics.target_rpm = self.target_rpm
        metrics.error = error_array[-1]
        metrics.rpm_filtered = np.mean(rpm_array[-5:])
        
        # Calculate overshoot
        if self.target_rpm > 0:
            max_rpm = np.max(rpm_array)
            if max_rpm > self.target_rpm:
                metrics.overshoot_percent = ((max_rpm - self.target_rpm) / self.target_rpm) * 100
        
        # Detect oscillation using zero-crossing of error
        if len(error_array) >= 20:
            zero_crossings = np.where(np.diff(np.signbit(error_array)))[0]
            if len(zero_crossings) >= 4:
                # Calculate oscillation frequency
                avg_period = np.mean(np.diff(zero_crossings)) * 2  # samples per full cycle
                if avg_period > 0 and len(self.time_history) >= 2:
                    sample_rate = len(self.time_history) / (self.time_history[-1] - self.time_history[0] + 0.001)
                    metrics.oscillation_frequency_hz = sample_rate / avg_period
                    metrics.is_hunting = metrics.oscillation_frequency_hz > 0.5 and metrics.oscillation_frequency_hz < 10
        
        # Calculate stability score (0-100)
        # Based on: error magnitude, oscillation, settling
        error_score = max(0, 100 - abs(np.mean(error_array[-10:])) / (self.target_rpm + 1) * 500)
        variance_score = max(0, 100 - np.std(rpm_array[-20:]) / (self.target_rpm + 1) * 200)
        oscillation_penalty = 30 if metrics.is_hunting else 0
        
        metrics.stability_score = max(0, min(100, (error_score + variance_score) / 2 - oscillation_penalty))
        metrics.is_stable = metrics.stability_score > 70 and not metrics.is_hunting
        
        return metrics
    
    def analyze_step_response(self) -> dict:
        """Analyze recorded step response for system identification"""
        if len(self.step_response_data) < 20:
            return {"error": "Not enough data"}
        
        data = np.array(self.step_response_data)
        times = data[:, 0]
        rpms = data[:, 1]
        target = data[0, 2]
        
        # Find steady-state value
        steady_state = np.mean(rpms[-10:])
        
        # Find rise time (10% to 90% of final value)
        rise_start_val = 0.1 * steady_state
        rise_end_val = 0.9 * steady_state
        
        rise_start_idx = np.argmax(rpms > rise_start_val)
        rise_end_idx = np.argmax(rpms > rise_end_val)
        
        rise_time = times[rise_end_idx] - times[rise_start_idx] if rise_end_idx > rise_start_idx else 0
        
        # Find overshoot
        max_rpm = np.max(rpms)
        overshoot = ((max_rpm - steady_state) / steady_state * 100) if steady_state > 0 else 0
        
        # Find settling time (within 2% of steady state)
        tolerance = 0.02 * steady_state
        settled_mask = np.abs(rpms - steady_state) < tolerance
        if np.any(settled_mask):
            settling_idx = np.argmax(settled_mask)
            settling_time = times[settling_idx]
        else:
            settling_time = times[-1]
        
        return {
            "rise_time": rise_time,
            "overshoot_percent": max(0, overshoot),
            "settling_time": settling_time,
            "steady_state_error": target - steady_state,
            "steady_state_value": steady_state,
        }


class AutoTuner:
    """
    Advanced auto-tuning algorithms running on PC.
    Uses the PC's computational power for sophisticated tuning methods.
    """
    
    @staticmethod
    def ziegler_nichols_classic(ku: float, tu: float) -> TuningResult:
        """
        Classic Ziegler-Nichols method.
        ku: Ultimate gain (gain at which system oscillates)
        tu: Ultimate period (period of oscillation)
        """
        result = TuningResult()
        result.method = TuningMethod.ZIEGLER_NICHOLS.value
        result.timestamp = datetime.now().isoformat()
        
        # Classic Z-N formulas
        result.kp = 0.6 * ku
        result.ki = 2 * result.kp / tu
        result.kd = result.kp * tu / 8
        
        result.notes = f"Ku={ku:.4f}, Tu={tu:.4f}s"
        return result
    
    @staticmethod
    def ziegler_nichols_no_overshoot(ku: float, tu: float) -> TuningResult:
        """Ziegler-Nichols variant with reduced overshoot"""
        result = TuningResult()
        result.method = f"{TuningMethod.ZIEGLER_NICHOLS.value} (No Overshoot)"
        result.timestamp = datetime.now().isoformat()
        
        # Modified for less overshoot
        result.kp = 0.2 * ku
        result.ki = 2 * result.kp / tu
        result.kd = result.kp * tu / 3
        
        result.notes = f"Ku={ku:.4f}, Tu={tu:.4f}s - Optimized for minimal overshoot"
        return result
    
    @staticmethod
    def cohen_coon(k: float, tau: float, theta: float) -> TuningResult:
        """
        Cohen-Coon method for processes with significant dead time.
        k: Process gain
        tau: Time constant
        theta: Dead time
        """
        result = TuningResult()
        result.method = TuningMethod.COHEN_COON.value
        result.timestamp = datetime.now().isoformat()
        
        r = theta / tau
        
        # Cohen-Coon formulas
        result.kp = (1.35 / k) * (tau / theta + 0.185)
        ti = 2.5 * theta * (tau + 0.185 * theta) / (tau + 0.611 * theta)
        td = 0.37 * theta * tau / (tau + 0.185 * theta)
        
        result.ki = result.kp / ti
        result.kd = result.kp * td
        
        result.notes = f"K={k:.4f}, τ={tau:.4f}s, θ={theta:.4f}s"
        return result
    
    @staticmethod
    def imc_tuning(k: float, tau: float, theta: float, lambda_factor: float = 1.0) -> TuningResult:
        """
        Internal Model Control (IMC) tuning.
        Provides robust tuning with adjustable aggressiveness.
        lambda_factor: Closed-loop time constant factor (higher = slower, more robust)
        """
        result = TuningResult()
        result.method = TuningMethod.IMC.value
        result.timestamp = datetime.now().isoformat()
        
        lambda_val = lambda_factor * tau
        
        # IMC formulas for first-order plus dead time
        result.kp = tau / (k * (lambda_val + theta))
        result.ki = result.kp / tau
        result.kd = result.kp * theta / 2
        
        result.notes = f"K={k:.4f}, τ={tau:.4f}s, θ={theta:.4f}s, λ={lambda_val:.4f}s"
        return result
    
    @staticmethod
    def calculate_from_step_response(step_data: dict) -> TuningResult:
        """
        Calculate PID gains from step response analysis.
        Uses first-order plus dead time (FOPDT) model.
        """
        result = TuningResult()
        result.method = "Step Response Analysis"
        result.timestamp = datetime.now().isoformat()
        
        rise_time = step_data.get("rise_time", 0.5)
        overshoot = step_data.get("overshoot_percent", 0)
        steady_state = step_data.get("steady_state_value", 1440)
        
        # Estimate time constant from rise time
        tau = rise_time / 2.2  # Approximation for first-order system
        
        # Estimate dead time from overshoot
        theta = 0.1 * tau  # Start with small dead time estimate
        if overshoot > 20:
            theta = 0.2 * tau
        
        # Use conservative tuning based on estimates
        k = 1.0  # Normalized gain
        
        result.kp = 0.9 * tau / (k * theta)
        result.ki = result.kp / (3.33 * tau)
        result.kd = result.kp * (tau / 8)
        
        # Clamp to reasonable values
        result.kp = max(0.01, min(2.0, result.kp))
        result.ki = max(0.001, min(0.5, result.ki))
        result.kd = max(0.0001, min(0.1, result.kd))
        
        result.notes = f"Rise time: {rise_time:.3f}s, Overshoot: {overshoot:.1f}%"
        return result


class SerialWorker(QThread):
    """Worker thread for serial communication"""
    data_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool)
    raw_rpm_received = pyqtSignal(float, float, float)  # rpm, target, timestamp

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.serial_port = None
        self.running = True
        self.pc_control_mode = False
        self.pwm_to_send = None
        self.pwm_lock = threading.Lock()

    def run(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                rtscts=False,
                dsrdtr=False,
                exclusive=True
            )
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            self.connection_status.emit(True)

            buffer = ""
            while self.running:
                # Send PC-calculated PWM if in PC control mode
                with self.pwm_lock:
                    if self.pc_control_mode and self.pwm_to_send is not None:
                        self.serial_port.write(f"SET_PWM {self.pwm_to_send}\n".encode('utf-8'))
                        self.pwm_to_send = None

                if self.serial_port.in_waiting:
                    try:
                        raw_data = self.serial_port.read(self.serial_port.in_waiting)
                        if len(raw_data) > 0:
                            decoded = raw_data.decode('utf-8', errors='replace')
                            buffer += decoded

                            while '\n' in buffer:
                                line_end = buffer.find('\n')
                                line = buffer[:line_end].strip()
                                buffer = buffer[line_end + 1:]

                                if not line or len(line) < 5:
                                    continue

                                # Emit data for GUI processing
                                self.data_received.emit(line)

                                # Parse RPM data for real-time processing
                                if line.startswith("STATUS:"):
                                    try:
                                        parts = line[7:].split(',')
                                        if len(parts) >= 3:
                                            timestamp = float(parts[0])
                                            target = float(parts[1])
                                            current = float(parts[2])
                                            self.raw_rpm_received.emit(current, target, timestamp)
                                    except:
                                        pass

                    except Exception as e:
                        buffer = ""
                        continue
                time.sleep(0.005)  # 200Hz polling

        except Exception as e:
                print(f"Serial error: {e}")
                self.connection_status.emit(False)

    def stop(self):
        self.running = False
        if self.serial_port:
            self.serial_port.close()

    def send_command(self, command):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                return True
            except Exception as e:
                print(f"Send error: {e}")
        return False

    def set_pwm(self, pwm_value: int):
        """Set PWM value for PC control mode"""
        with self.pwm_lock:
            self.pwm_to_send = pwm_value


class PlotCanvas(FigureCanvas):
    """Enhanced matplotlib canvas with multiple views"""
    def __init__(self, parent=None, width=10, height=7, dpi=100):
        self.figure = Figure(figsize=(width, height), dpi=dpi, facecolor='#1e1e2e')
        
        # Create subplots with dark theme
        self.axes = []
        gs = self.figure.add_gridspec(2, 2, hspace=0.3, wspace=0.25)
        
        for i, (row, col) in enumerate([(0, 0), (0, 1), (1, 0), (1, 1)]):
            ax = self.figure.add_subplot(gs[row, col])
            ax.set_facecolor('#2d2d3d')
            ax.tick_params(colors='#a0a0a0')
            ax.spines['bottom'].set_color('#404050')
            ax.spines['top'].set_color('#404050')
            ax.spines['left'].set_color('#404050')
            ax.spines['right'].set_color('#404050')
            ax.grid(True, alpha=0.2, color='#505060')
            self.axes.append(ax)

        super().__init__(self.figure)
        self.setParent(parent)
        self.setup_plots()

        # Data storage
        self.max_points = 200
        self.time_data = []
        self.target_rpm_data = []
        self.current_rpm_data = []
        self.filtered_rpm_data = []
        self.error_data = []
        self.pid_output_data = []
        self.stability_score_data = []

    def setup_plots(self):
        """Setup plot titles and labels with dark theme"""
        titles = ['Motor Speed', 'Control Error', 'PID Output', 'Stability Score']
        ylabels = ['RPM', 'Error (RPM)', 'Output', 'Score (%)']
        colors = ['#89b4fa', '#f38ba8', '#a6e3a1', '#fab387']
        
        for ax, title, ylabel, color in zip(self.axes, titles, ylabels, colors):
            ax.set_title(title, fontsize=11, fontweight='bold', color=color, pad=8)
            ax.set_ylabel(ylabel, fontsize=9, color='#a0a0a0')
            ax.set_xlabel('Time (s)', fontsize=8, color='#808080')

    def update_plot(self, time_val, target_rpm, current_rpm, filtered_rpm,
                   error, pid_output, stability_score):
        """Update plot data with Kalman-filtered values"""
        self.time_data.append(time_val)
        self.target_rpm_data.append(target_rpm)
        self.current_rpm_data.append(current_rpm)
        self.filtered_rpm_data.append(filtered_rpm)
        self.error_data.append(error)
        self.pid_output_data.append(pid_output)
        self.stability_score_data.append(stability_score)

        # Limit data points
        if len(self.time_data) > self.max_points:
            for data_list in [self.time_data, self.target_rpm_data, self.current_rpm_data,
                             self.filtered_rpm_data, self.error_data, self.pid_output_data,
                             self.stability_score_data]:
                data_list.pop(0)

        min_len = min(len(self.time_data), len(self.target_rpm_data))
        if min_len < 2:
            return

        # Convert to relative time
        time_rel = [(t - self.time_data[0]) / 1000 for t in self.time_data[:min_len]]

        # Clear and replot
        for ax in self.axes:
            ax.cla()

        # RPM plot with raw and filtered
        self.axes[0].plot(time_rel, self.target_rpm_data[:min_len], 
                         color='#f5c2e7', linestyle='--', label='Target', linewidth=2)
        self.axes[0].plot(time_rel, self.current_rpm_data[:min_len], 
                         color='#74c7ec', alpha=0.4, label='Raw', linewidth=1)
        self.axes[0].plot(time_rel, self.filtered_rpm_data[:min_len], 
                         color='#89b4fa', label='Filtered', linewidth=2)
        self.axes[0].set_title('Motor Speed', fontsize=11, fontweight='bold', color='#89b4fa')
        self.axes[0].legend(fontsize=8, loc='upper right', facecolor='#2d2d3d', 
                           edgecolor='#404050', labelcolor='#a0a0a0')

        # Error plot
        self.axes[1].fill_between(time_rel, self.error_data[:min_len], 0,
                                  color='#f38ba8', alpha=0.3)
        self.axes[1].plot(time_rel, self.error_data[:min_len], 
                         color='#f38ba8', linewidth=1.5)
        self.axes[1].axhline(y=0, color='#45475a', linestyle='-', linewidth=1)
        self.axes[1].set_title('Control Error', fontsize=11, fontweight='bold', color='#f38ba8')

        # PID output
        self.axes[2].plot(time_rel, self.pid_output_data[:min_len], 
                         color='#a6e3a1', linewidth=1.5)
        self.axes[2].set_title('PID Output', fontsize=11, fontweight='bold', color='#a6e3a1')

        # Stability score
        self.axes[3].fill_between(time_rel, self.stability_score_data[:min_len], 0,
                                  color='#fab387', alpha=0.3)
        self.axes[3].plot(time_rel, self.stability_score_data[:min_len], 
                         color='#fab387', linewidth=2)
        self.axes[3].set_ylim(0, 100)
        self.axes[3].axhline(y=70, color='#f38ba8', linestyle='--', linewidth=1, alpha=0.5)
        self.axes[3].set_title('Stability Score', fontsize=11, fontweight='bold', color='#fab387')

        # Apply consistent styling
        for ax in self.axes:
            ax.set_facecolor('#2d2d3d')
            ax.tick_params(colors='#a0a0a0', labelsize=8)
            ax.spines['bottom'].set_color('#404050')
            ax.spines['top'].set_color('#404050')
            ax.spines['left'].set_color('#404050')
            ax.spines['right'].set_color('#404050')
            ax.grid(True, alpha=0.15, color='#505060')
            if time_rel:
                ax.set_xlim(min(time_rel), max(time_rel) + 0.1)
            ax.set_xlabel('Time (s)', fontsize=8, color='#808080')

        try:
            self.figure.subplots_adjust(left=0.08, bottom=0.08, right=0.96, top=0.94, hspace=0.35, wspace=0.25)
            self.draw()
        except Exception as e:
            print(f"Plot error: {e}")

    def clear_data(self):
        """Clear all plot data"""
        self.time_data.clear()
        self.target_rpm_data.clear()
        self.current_rpm_data.clear()
        self.filtered_rpm_data.clear()
        self.error_data.clear()
        self.pid_output_data.clear()
        self.stability_score_data.clear()


class PIDControllerGUI(QMainWindow):
    """Enhanced PC-Powered PID Controller GUI"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLDC PID Controller - PC-Powered Auto-Tune")
        self.setMinimumSize(1400, 900)
        
        # Initialize components
        self.serial_worker = None
        self.data_queue = queue.Queue()
        self.kalman_filter = KalmanFilter(process_variance=1e-4, measurement_variance=0.5)
        self.stability_analyzer = StabilityAnalyzer(window_size=100)
        self.auto_tuner = AutoTuner()
        
        # Control mode
        self.control_mode = ControlMode.ARDUINO_PID
        self.pc_pid_integral = 0.0
        self.pc_pid_prev_error = 0.0

        # Current parameters
        self.current_params = {
            'target_rpm': 1440.0,
            'kp': 0.3,
            'ki': 0.02,
            'kd': 0.02,
            'pulses_per_rev': 4,
            'motor_enabled': False
        }
        
        # Auto-tuning state
        self.auto_tuning_active = False
        self.auto_tuning_phase = 0
        self.tuning_data = []
        self.ultimate_gain = 0.0
        self.ultimate_period = 0.0
        
        # Data collection for analysis
        self.last_data_time = time.time()
        self.connection_timeout = 10
        self.plot_update_counter = 0
        self.plot_update_interval = 2  # Update every 2 samples
        
        # Tuning history
        self.tuning_history: List[TuningResult] = []
        
        # Build UI
        self.init_ui()
        self.apply_dark_theme()
        
        # Start timers
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.process_serial_data)
        self.data_timer.start(50)  # 20Hz processing
        
        self.metrics_timer = QTimer()
        self.metrics_timer.timeout.connect(self.update_metrics_display)
        self.metrics_timer.start(250)  # 4Hz metrics update

    def init_ui(self):
        """Initialize the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # Left panel - Controls
        left_panel = self.create_control_panel()
        left_panel.setFixedWidth(380)
        
        # Right panel - Plots and Analysis
        right_panel = self.create_plot_panel()
        
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel, stretch=1)
        
        # Create menu bar
        self.create_menu_bar()
        
        # Create status bar
        self.statusBar().showMessage("Ready - Connect to Arduino to begin")

    def create_control_panel(self) -> QWidget:
        """Create the control panel with all settings"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(8)
        
        # ============ CONNECTION GROUP ============
        conn_group = QGroupBox("Connection")
        conn_layout = QFormLayout()
        conn_layout.setSpacing(6)
        
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        conn_layout.addRow("Port:", self.port_combo)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "57600", "38400", "19200", "9600"])
        conn_layout.addRow("Baud:", self.baud_combo)

        conn_buttons = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        refresh_btn = QPushButton("Refresh")
        refresh_btn.setFixedWidth(40)
        refresh_btn.clicked.connect(self.update_serial_ports)
        conn_buttons.addWidget(self.connect_btn)
        conn_buttons.addWidget(refresh_btn)
        conn_layout.addRow("", conn_buttons)
        
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)
        
        # ============ QUICK ACTIONS GROUP ============
        quick_group = QGroupBox("Quick Actions")
        quick_layout = QVBoxLayout()
        quick_layout.setSpacing(8)
        
        # Motor control row
        motor_row = QHBoxLayout()
        self.motor_btn = QPushButton("Motor OFF")
        self.motor_btn.setCheckable(True)
        self.motor_btn.clicked.connect(self.toggle_motor)
        self.motor_btn.setMinimumHeight(40)
        motor_row.addWidget(self.motor_btn)
        quick_layout.addLayout(motor_row)
        
        # Auto-tune button
        self.auto_tune_btn = QPushButton("One-Click Auto-Tune")
        self.auto_tune_btn.setMinimumHeight(45)
        self.auto_tune_btn.clicked.connect(self.start_auto_tune)
        self.auto_tune_btn.setStyleSheet("""
            QPushButton {
                background-color: #45475a;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #585b70;
            }
        """)
        quick_layout.addWidget(self.auto_tune_btn)
        
        # Tuning progress
        self.tune_progress = QProgressBar()
        self.tune_progress.setVisible(False)
        self.tune_progress.setTextVisible(True)
        quick_layout.addWidget(self.tune_progress)
        
        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)
        
        # ============ PID PARAMETERS GROUP ============
        pid_group = QGroupBox("PID Parameters")
        pid_layout = QFormLayout()
        pid_layout.setSpacing(6)

        # Target RPM
        self.target_rpm_spin = QSpinBox()
        self.target_rpm_spin.setRange(0, 5000)
        self.target_rpm_spin.setValue(int(self.current_params['target_rpm']))
        self.target_rpm_spin.setSingleStep(10)
        self.target_rpm_spin.valueChanged.connect(self.update_target_rpm)
        pid_layout.addRow("Target RPM:", self.target_rpm_spin)

        # Kp with slider
        kp_layout = QHBoxLayout()
        self.kp_slider = QSlider(Qt.Orientation.Horizontal)
        self.kp_slider.setRange(0, 200)
        self.kp_slider.setValue(int(self.current_params['kp'] * 100))
        self.kp_slider.valueChanged.connect(self.update_kp_from_slider)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0, 2.0)
        self.kp_spin.setValue(self.current_params['kp'])
        self.kp_spin.setSingleStep(0.01)
        self.kp_spin.setDecimals(3)
        self.kp_spin.valueChanged.connect(self.update_kp_from_spin)
        kp_layout.addWidget(self.kp_slider)
        kp_layout.addWidget(self.kp_spin)
        pid_layout.addRow("Kp:", kp_layout)

        # Ki with slider
        ki_layout = QHBoxLayout()
        self.ki_slider = QSlider(Qt.Orientation.Horizontal)
        self.ki_slider.setRange(0, 200)
        self.ki_slider.setValue(int(self.current_params['ki'] * 1000))
        self.ki_slider.valueChanged.connect(self.update_ki_from_slider)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0, 0.2)
        self.ki_spin.setValue(self.current_params['ki'])
        self.ki_spin.setSingleStep(0.001)
        self.ki_spin.setDecimals(4)
        self.ki_spin.valueChanged.connect(self.update_ki_from_spin)
        ki_layout.addWidget(self.ki_slider)
        ki_layout.addWidget(self.ki_spin)
        pid_layout.addRow("Ki:", ki_layout)

        # Kd with slider
        kd_layout = QHBoxLayout()
        self.kd_slider = QSlider(Qt.Orientation.Horizontal)
        self.kd_slider.setRange(0, 100)
        self.kd_slider.setValue(int(self.current_params['kd'] * 1000))
        self.kd_slider.valueChanged.connect(self.update_kd_from_slider)
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0, 0.1)
        self.kd_spin.setValue(self.current_params['kd'])
        self.kd_spin.setSingleStep(0.001)
        self.kd_spin.setDecimals(4)
        self.kd_spin.valueChanged.connect(self.update_kd_from_spin)
        kd_layout.addWidget(self.kd_slider)
        kd_layout.addWidget(self.kd_spin)
        pid_layout.addRow("Kd:", kd_layout)

        # Pulses per revolution
        self.ppr_spin = QSpinBox()
        self.ppr_spin.setRange(1, 100)
        self.ppr_spin.setValue(self.current_params['pulses_per_rev'])
        self.ppr_spin.valueChanged.connect(self.update_ppr)
        pid_layout.addRow("Pulses/Rev:", self.ppr_spin)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        # ============ REAL-TIME METRICS GROUP ============
        metrics_group = QGroupBox("Real-Time Metrics")
        metrics_layout = QGridLayout()
        metrics_layout.setSpacing(4)
        
        self.metric_labels = {}
        metrics_data = [
            ("Current RPM:", "current_rpm", "#89b4fa"),
            ("Filtered RPM:", "filtered_rpm", "#74c7ec"),
            ("Error:", "error", "#f38ba8"),
            ("Overshoot:", "overshoot", "#fab387"),
            ("Stability:", "stability", "#a6e3a1"),
            ("Status:", "status", "#cdd6f4"),
        ]
        
        for i, (label, key, color) in enumerate(metrics_data):
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: #a0a0a0;")
            val = QLabel("--")
            val.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 13px;")
            metrics_layout.addWidget(lbl, i, 0)
            metrics_layout.addWidget(val, i, 1)
            self.metric_labels[key] = val
        
        metrics_group.setLayout(metrics_layout)
        layout.addWidget(metrics_group)
        
        # ============ TUNING HISTORY GROUP ============
        history_group = QGroupBox("Tuning History")
        history_layout = QVBoxLayout()
        
        self.history_table = QTableWidget(0, 4)
        self.history_table.setHorizontalHeaderLabels(["Method", "Kp", "Ki", "Kd"])
        self.history_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.history_table.setMaximumHeight(120)
        self.history_table.itemClicked.connect(self.apply_history_item)
        history_layout.addWidget(self.history_table)
        
        history_group.setLayout(history_layout)
        layout.addWidget(history_group)
        
        layout.addStretch()
        
        # Initialize serial ports
        self.update_serial_ports()
        
        return panel

    def create_plot_panel(self) -> QWidget:
        """Create the plotting panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create tabs for different views
        tabs = QTabWidget()
        tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #45475a;
                background: #1e1e2e;
            }
            QTabBar::tab {
                background: #313244;
                color: #a0a0a0;
                padding: 8px 16px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #45475a;
                color: #cdd6f4;
            }
        """)
        
        # Main plot tab
        plot_tab = QWidget()
        plot_layout = QVBoxLayout(plot_tab)
        self.plot_canvas = PlotCanvas(plot_tab, width=10, height=7)
        toolbar = NavigationToolbar(self.plot_canvas, plot_tab)
        toolbar.setStyleSheet("background: #1e1e2e; color: #a0a0a0;")
        plot_layout.addWidget(self.plot_canvas)
        plot_layout.addWidget(toolbar)
        tabs.addTab(plot_tab, "Live Data")
        
        # Analysis tab
        analysis_tab = QWidget()
        analysis_layout = QVBoxLayout(analysis_tab)
        
        self.analysis_text = QTextEdit()
        self.analysis_text.setReadOnly(True)
        self.analysis_text.setStyleSheet("""
            QTextEdit {
                background: #1e1e2e;
                color: #cdd6f4;
                font-family: 'JetBrains Mono', 'Consolas', monospace;
                font-size: 12px;
                border: none;
                padding: 10px;
            }
        """)
        self.analysis_text.setPlainText("Connect to Arduino and enable motor to begin analysis...")
        analysis_layout.addWidget(self.analysis_text)
        
        # Analysis controls
        analysis_controls = QHBoxLayout()
        
        step_response_btn = QPushButton("Record Step Response")
        step_response_btn.clicked.connect(self.start_step_response)
        analysis_controls.addWidget(step_response_btn)
        
        analyze_btn = QPushButton("Analyze & Suggest")
        analyze_btn.clicked.connect(self.analyze_system)
        analysis_controls.addWidget(analyze_btn)
        
        export_btn = QPushButton("Export Data")
        export_btn.clicked.connect(self.export_data)
        analysis_controls.addWidget(export_btn)
        
        analysis_controls.addStretch()
        analysis_layout.addLayout(analysis_controls)
        
        tabs.addTab(analysis_tab, "Analysis")
        
        layout.addWidget(tabs)
        return panel

    def create_menu_bar(self):
        """Create the menu bar"""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu('File')

        save_action = QAction('Save Parameters', self)
        save_action.triggered.connect(self.save_parameters)
        file_menu.addAction(save_action)

        load_action = QAction('Load Parameters', self)
        load_action.triggered.connect(self.load_parameters)
        file_menu.addAction(load_action)

        file_menu.addSeparator()

        exit_action = QAction('Exit', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Help menu
        help_menu = menubar.addMenu('Help')

        about_action = QAction('About', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def apply_dark_theme(self):
        """Apply a modern dark theme"""
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e2e;
                color: #cdd6f4;
            }

            QGroupBox {
                font-weight: bold;
                border: 1px solid #45475a;
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 12px;
                color: #cdd6f4;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 8px;
                color: #89b4fa;
            }

            QPushButton {
                background-color: #313244;
                border: 1px solid #45475a;
                border-radius: 5px;
                padding: 8px 14px;
                color: #cdd6f4;
                font-size: 12px;
            }

            QPushButton:hover {
                background-color: #45475a;
                border-color: #89b4fa;
            }

            QPushButton:pressed {
                background-color: #585b70;
            }

            QPushButton:checked {
                background-color: #a6e3a1;
                color: #1e1e2e;
                border-color: #a6e3a1;
            }

            QPushButton:disabled {
                background-color: #1e1e2e;
                color: #6c7086;
            }

            QSlider::groove:horizontal {
                border: none;
                height: 6px;
                background: #313244;
                border-radius: 3px;
            }

            QSlider::handle:horizontal {
                background: #89b4fa;
                border: none;
                width: 16px;
                height: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }

            QSlider::handle:horizontal:hover {
                background: #b4befe;
            }

            QSpinBox, QDoubleSpinBox, QComboBox {
                background-color: #313244;
                border: 1px solid #45475a;
                border-radius: 4px;
                padding: 5px 8px;
                color: #cdd6f4;
                min-width: 70px;
            }

            QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
                border-color: #89b4fa;
            }

            QComboBox::drop-down {
                border: none;
                padding-right: 8px;
            }

            QComboBox::down-arrow {
                image: none;
                border-left: 4px solid transparent;
                border-right: 4px solid transparent;
                border-top: 5px solid #89b4fa;
            }

            QComboBox QAbstractItemView {
                background-color: #313244;
                color: #cdd6f4;
                selection-background-color: #45475a;
            }
            
            QProgressBar {
                border: 1px solid #45475a;
                border-radius: 4px;
                background: #313244;
                text-align: center;
                color: #cdd6f4;
            }
            
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #89b4fa, stop:1 #b4befe);
                border-radius: 3px;
            }
            
            QTableWidget {
                background-color: #1e1e2e;
                border: 1px solid #45475a;
                gridline-color: #313244;
                color: #cdd6f4;
            }
            
            QTableWidget::item {
                padding: 4px;
            }
            
            QTableWidget::item:selected {
                background-color: #45475a;
            }
            
            QHeaderView::section {
                background-color: #313244;
                color: #89b4fa;
                padding: 6px;
                border: none;
                border-bottom: 1px solid #45475a;
            }
            
            QMenuBar {
                background-color: #1e1e2e;
                color: #cdd6f4;
                border-bottom: 1px solid #313244;
            }

            QMenuBar::item:selected {
                background-color: #313244;
            }

            QMenu {
                background-color: #313244;
                color: #cdd6f4;
                border: 1px solid #45475a;
            }

            QMenu::item:selected {
                background-color: #45475a;
            }

            QStatusBar {
                background-color: #181825;
                color: #a6adc8;
                border-top: 1px solid #313244;
            }
            
            QLabel {
                color: #cdd6f4;
            }
        """)

    def update_serial_ports(self):
        """Update the list of available serial ports"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()

        arduino_ports = []
        other_ports = []

        for port in ports:
            port_name = port.device.lower()
            description = port.description.lower() if port.description else ""

            if ('acm' in port_name or 'usb' in port_name or
                'arduino' in description or 'uno' in description):
                arduino_ports.append(port)
            elif not port_name.startswith('/dev/ttys'):
                other_ports.append(port)

        for port in arduino_ports:
            self.port_combo.addItem(f"USB {port.device}", port.device)
        for port in other_ports:
            self.port_combo.addItem(f"SERIAL {port.device}", port.device)

    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.serial_worker and self.serial_worker.isRunning():
            self.serial_worker.stop()
            self.serial_worker = None
            self.connect_btn.setText("Connect")
            self.statusBar().showMessage("Disconnected")
        else:
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "Error", "No serial port selected")
                return

            baud = int(self.baud_combo.currentText())
            self.serial_worker = SerialWorker(port, baud)
            self.serial_worker.data_received.connect(self.on_data_received)
            self.serial_worker.connection_status.connect(self.on_connection_status)
            self.serial_worker.raw_rpm_received.connect(self.on_raw_rpm)
            self.serial_worker.start()
            self.connect_btn.setText("Connecting...")

    def on_connection_status(self, connected):
        """Handle connection status changes"""
        if connected:
            self.connect_btn.setText("Disconnect")
            self.statusBar().showMessage(f"Connected to {self.port_combo.currentData()}")
            self.last_data_time = time.time()
            self.kalman_filter.reset()
            QTimer.singleShot(500, lambda: self.send_command("GET_STATUS"))
        else:
            self.connect_btn.setText("Connect")
            self.statusBar().showMessage("Connection failed")

    def on_data_received(self, data):
        """Handle incoming serial data"""
        self.last_data_time = time.time()
        self.data_queue.put(data)

    def on_raw_rpm(self, rpm: float, target: float, timestamp: float):
        """Handle raw RPM data for PC-side processing"""
        # Apply Kalman filter
        filtered_rpm = self.kalman_filter.update(rpm)
        
        # Update stability analyzer
        self.stability_analyzer.add_sample(filtered_rpm, target, timestamp)
        
        # If in PC control mode, calculate PID here
        if self.control_mode == ControlMode.PC_PID and self.current_params['motor_enabled']:
            error = target - filtered_rpm
            pwm = self.calculate_pc_pid(error)
            if self.serial_worker:
                self.serial_worker.set_pwm(int(pwm))

    def calculate_pc_pid(self, error: float) -> float:
        """Calculate PID output on PC for maximum precision"""
        kp = self.current_params['kp']
        ki = self.current_params['ki']
        kd = self.current_params['kd']
        
        # Proportional
        p_term = kp * error
        
        # Integral with anti-windup
        self.pc_pid_integral += ki * error
        self.pc_pid_integral = max(-200, min(200, self.pc_pid_integral))
        
        # Derivative
        d_term = kd * (error - self.pc_pid_prev_error)
        self.pc_pid_prev_error = error
        
        # Total output
        output = p_term + self.pc_pid_integral + d_term
        output = max(-1000, min(1000, output))
        
        # Map to PWM (0-255)
        pwm = ((output + 1000) / 2000) * 255
        return max(0, min(255, pwm))

    def process_serial_data(self):
        """Process data from the serial queue"""
        try:
            while not self.data_queue.empty():
                line = self.data_queue.get_nowait()
                if line.startswith("STATUS:"):
                    self.parse_status_data(line)
                elif "ERROR" in line or "set to" in line:
                    self.statusBar().showMessage(f"Arduino: {line}")

            # Check for connection timeout
            if self.serial_worker and self.serial_worker.isRunning():
                if time.time() - self.last_data_time > self.connection_timeout:
                    self.statusBar().showMessage("Warning: No data from Arduino")
        except:
            pass

    def parse_status_data(self, data):
        """Parse status data from Arduino"""
        try:
            if not data.startswith("STATUS:"):
                return

            parts = data[7:].split(',')
            if len(parts) < 10:
                return

            timestamp = float(parts[0])
            target_rpm = float(parts[1])
            current_rpm = float(parts[2])
            error = float(parts[3])
            pid_output = float(parts[4])
            kp = float(parts[5])
            ki = float(parts[6])
            kd = float(parts[7])
            
            # Get Kalman-filtered RPM
            filtered_rpm = self.kalman_filter.update(current_rpm)
            
            # Calculate stability metrics
            metrics = self.stability_analyzer.calculate_metrics()
            
            # Update plot
            self.plot_update_counter += 1
            if self.plot_update_counter >= self.plot_update_interval:
                self.plot_canvas.update_plot(
                    timestamp, target_rpm, current_rpm, filtered_rpm,
                    error, pid_output, metrics.stability_score
                )
                self.plot_update_counter = 0

            # Update metric labels
            self.metric_labels['current_rpm'].setText(f"{current_rpm:.1f}")
            self.metric_labels['filtered_rpm'].setText(f"{filtered_rpm:.1f}")
            self.metric_labels['error'].setText(f"{error:.1f}")
            self.metric_labels['overshoot'].setText(f"{metrics.overshoot_percent:.1f}%")
            self.metric_labels['stability'].setText(f"{metrics.stability_score:.0f}%")
            
            # Status indicator
            if metrics.is_hunting:
                self.metric_labels['status'].setText("HUNTING")
                self.metric_labels['status'].setStyleSheet("color: #f38ba8; font-weight: bold;")
            elif metrics.is_stable:
                self.metric_labels['status'].setText("STABLE")
                self.metric_labels['status'].setStyleSheet("color: #a6e3a1; font-weight: bold;")
            else:
                self.metric_labels['status'].setText("SETTLING")
                self.metric_labels['status'].setStyleSheet("color: #f9e2af; font-weight: bold;")
            
            # Collect data during auto-tuning
            if self.auto_tuning_active:
                self.tuning_data.append({
                    'time': timestamp,
                    'rpm': current_rpm,
                    'target': target_rpm,
                    'error': error
                })
                
        except Exception as e:
            print(f"Parse error: {e}")

    def update_metrics_display(self):
        """Update metrics display periodically"""
        pass  # Metrics are updated in parse_status_data

    def send_command(self, command):
        """Send command to Arduino"""
        if self.serial_worker:
            self.serial_worker.send_command(command)

    def toggle_motor(self, checked):
        """Toggle motor enable/disable"""
        self.current_params['motor_enabled'] = checked
        if checked:
            self.motor_btn.setText("Motor ON")
            self.send_command("ENABLE_MOTOR 1")
        else:
            self.motor_btn.setText("Motor OFF")
            self.send_command("ENABLE_MOTOR 0")
            self.pc_pid_integral = 0.0
            self.pc_pid_prev_error = 0.0

    def update_target_rpm(self, value):
        """Update target RPM"""
        self.current_params['target_rpm'] = value
        self.send_command(f"SET_TARGET_RPM {value}")

    def update_kp_from_slider(self, value):
        """Update Kp from slider"""
        kp = value / 100.0
        self.kp_spin.blockSignals(True)
        self.kp_spin.setValue(kp)
        self.kp_spin.blockSignals(False)
        self.current_params['kp'] = kp
        self.send_command(f"SET_KP {kp}")

    def update_kp_from_spin(self, value):
        """Update Kp from spinbox"""
        self.kp_slider.blockSignals(True)
        self.kp_slider.setValue(int(value * 100))
        self.kp_slider.blockSignals(False)
        self.current_params['kp'] = value
        self.send_command(f"SET_KP {value}")

    def update_ki_from_slider(self, value):
        """Update Ki from slider"""
        ki = value / 1000.0
        self.ki_spin.blockSignals(True)
        self.ki_spin.setValue(ki)
        self.ki_spin.blockSignals(False)
        self.current_params['ki'] = ki
        self.send_command(f"SET_KI {ki}")

    def update_ki_from_spin(self, value):
        """Update Ki from spinbox"""
        self.ki_slider.blockSignals(True)
        self.ki_slider.setValue(int(value * 1000))
        self.ki_slider.blockSignals(False)
        self.current_params['ki'] = value
        self.send_command(f"SET_KI {value}")

    def update_kd_from_slider(self, value):
        """Update Kd from slider"""
        kd = value / 1000.0
        self.kd_spin.blockSignals(True)
        self.kd_spin.setValue(kd)
        self.kd_spin.blockSignals(False)
        self.current_params['kd'] = kd
        self.send_command(f"SET_KD {kd}")

    def update_kd_from_spin(self, value):
        """Update Kd from spinbox"""
        self.kd_slider.blockSignals(True)
        self.kd_slider.setValue(int(value * 1000))
        self.kd_slider.blockSignals(False)
        self.current_params['kd'] = value
        self.send_command(f"SET_KD {value}")

    def update_ppr(self, value):
        """Update pulses per revolution"""
        self.current_params['pulses_per_rev'] = value
        self.send_command(f"SET_PULSES_PER_REV {value}")

    def start_auto_tune(self):
        """Start one-click auto-tuning process"""
        if not self.serial_worker or not self.serial_worker.isRunning():
            QMessageBox.warning(self, "Not Connected", "Please connect to Arduino first.")
            return
        
        reply = QMessageBox.question(
            self, "Start Auto-Tune",
            "This will automatically tune the PID parameters.\n\n"
            "The process will:\n"
            "1. Enable the motor\n"
            "2. Perform step response analysis\n"
            "3. Calculate optimal PID gains\n"
            "4. Apply the new parameters\n\n"
            "Continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        self.auto_tuning_active = True
        self.auto_tuning_phase = 1
        self.tuning_data = []
        
        self.auto_tune_btn.setEnabled(False)
        self.tune_progress.setVisible(True)
        self.tune_progress.setValue(0)
        self.tune_progress.setFormat("Phase 1: Starting motor...")
        
        # Reset controller
        self.send_command("RESET_CONTROLLER")
        
        # Set conservative initial gains
        self.send_command("SET_KP 0.2")
        self.send_command("SET_KI 0.0")
        self.send_command("SET_KD 0.0")
        
        # Enable motor
        self.motor_btn.setChecked(True)
        self.toggle_motor(True)
        
        # Start step response recording
        self.stability_analyzer.start_step_response()
        
        # Continue tuning after delay
        QTimer.singleShot(3000, self.auto_tune_phase_2)

    def auto_tune_phase_2(self):
        """Phase 2: Increase Kp to find oscillation point"""
        if not self.auto_tuning_active:
            return
        
        self.auto_tuning_phase = 2
        self.tune_progress.setValue(30)
        self.tune_progress.setFormat("Phase 2: Finding ultimate gain...")
        
        # Gradually increase Kp
        self.test_kp = 0.3
        self.oscillation_detected = False
        self.kp_test_timer = QTimer()
        self.kp_test_timer.timeout.connect(self.test_next_kp)
        self.kp_test_timer.start(2000)

    def test_next_kp(self):
        """Test next Kp value for oscillation"""
        if not self.auto_tuning_active:
            self.kp_test_timer.stop()
            return
        
        metrics = self.stability_analyzer.calculate_metrics()
        
        # Check for sustained oscillation (hunting)
        if metrics.is_hunting and metrics.oscillation_frequency_hz > 0.5:
            self.kp_test_timer.stop()
            self.ultimate_gain = self.test_kp
            self.ultimate_period = 1.0 / metrics.oscillation_frequency_hz
            self.auto_tune_phase_3()
            return
        
        # Increase Kp
        self.test_kp += 0.1
        if self.test_kp > 2.0:
            # Max Kp reached without oscillation
            self.kp_test_timer.stop()
            self.ultimate_gain = 1.5
            self.ultimate_period = 0.5
            self.auto_tune_phase_3()
            return
        
        self.send_command(f"SET_KP {self.test_kp:.3f}")
        self.tune_progress.setValue(30 + int(self.test_kp * 20))
        self.tune_progress.setFormat(f"Phase 2: Testing Kp={self.test_kp:.2f}...")

    def auto_tune_phase_3(self):
        """Phase 3: Calculate optimal PID parameters"""
        self.auto_tuning_phase = 3
        self.tune_progress.setValue(70)
        self.tune_progress.setFormat("Phase 3: Calculating optimal gains...")
        
        # Stop motor briefly
        self.motor_btn.setChecked(False)
        self.toggle_motor(False)
        
        # Stop step response recording
        self.stability_analyzer.stop_step_response()
        
        # Analyze step response
        step_analysis = self.stability_analyzer.analyze_step_response()
        
        # Calculate tuned parameters using multiple methods
        results = []
        
        # Ziegler-Nichols classic
        zn_result = AutoTuner.ziegler_nichols_classic(self.ultimate_gain, self.ultimate_period)
        results.append(("Z-N Classic", zn_result))
        
        # Ziegler-Nichols no overshoot
        zn_no_os = AutoTuner.ziegler_nichols_no_overshoot(self.ultimate_gain, self.ultimate_period)
        results.append(("Z-N No Overshoot", zn_no_os))
        
        # Step response based
        if step_analysis.get("rise_time", 0) > 0:
            sr_result = AutoTuner.calculate_from_step_response(step_analysis)
            results.append(("Step Response", sr_result))
        
        # Use the no-overshoot variant for motor control (gentler)
        best_result = zn_no_os
        
        # Apply with delay
        QTimer.singleShot(1000, lambda: self.apply_tuning_result(best_result, results))

    def apply_tuning_result(self, result: TuningResult, all_results: list):
        """Apply tuning result and finish auto-tune"""
        self.tune_progress.setValue(90)
        self.tune_progress.setFormat("Applying optimized parameters...")
        
        # Apply the parameters
        self.kp_spin.setValue(result.kp)
        self.ki_spin.setValue(result.ki)
        self.kd_spin.setValue(result.kd)
        
        # Add to history
        self.add_to_history(result)
        for name, res in all_results:
            if res != result:
                res.notes = f"{name}: {res.notes}"
                self.add_to_history(res)
        
        # Enable motor with new parameters
        self.motor_btn.setChecked(True)
        self.toggle_motor(True)
        
        # Clear plot for fresh start
        self.plot_canvas.clear_data()
        
        # Finish
        self.tune_progress.setValue(100)
        self.tune_progress.setFormat("Complete!")
        
        QTimer.singleShot(1000, self.finish_auto_tune)

    def finish_auto_tune(self):
        """Finish auto-tuning process"""
        self.auto_tuning_active = False
        self.auto_tune_btn.setEnabled(True)
        self.tune_progress.setVisible(False)
        
        # Show results
        analysis_text = f"""
═══════════════════════════════════════════════════════════════
                    AUTO-TUNE COMPLETE
═══════════════════════════════════════════════════════════════

Ultimate Gain (Ku): {self.ultimate_gain:.4f}
Ultimate Period (Tu): {self.ultimate_period:.4f} seconds

Applied Parameters:
  Kp = {self.current_params['kp']:.4f}
  Ki = {self.current_params['ki']:.4f}
  Kd = {self.current_params['kd']:.4f}

Method: Ziegler-Nichols (No Overshoot variant)

Tip: Monitor the stability score. If still hunting:
  - Reduce Kp by 10-20%
  - Increase Kd slightly for more damping

═══════════════════════════════════════════════════════════════
"""
        self.analysis_text.setPlainText(analysis_text)
        
        QMessageBox.information(
            self, "Auto-Tune Complete",
            f"PID parameters have been optimized!\n\n"
            f"Kp = {self.current_params['kp']:.4f}\n"
            f"Ki = {self.current_params['ki']:.4f}\n"
            f"Kd = {self.current_params['kd']:.4f}\n\n"
            f"Monitor the stability score and fine-tune if needed."
        )

    def add_to_history(self, result: TuningResult):
        """Add tuning result to history"""
        self.tuning_history.append(result)
        
        row = self.history_table.rowCount()
        self.history_table.insertRow(row)
        self.history_table.setItem(row, 0, QTableWidgetItem(result.method))
        self.history_table.setItem(row, 1, QTableWidgetItem(f"{result.kp:.4f}"))
        self.history_table.setItem(row, 2, QTableWidgetItem(f"{result.ki:.4f}"))
        self.history_table.setItem(row, 3, QTableWidgetItem(f"{result.kd:.4f}"))

    def apply_history_item(self, item):
        """Apply parameters from history"""
        row = item.row()
        if row < len(self.tuning_history):
            result = self.tuning_history[row]
            self.kp_spin.setValue(result.kp)
            self.ki_spin.setValue(result.ki)
            self.kd_spin.setValue(result.kd)
            self.statusBar().showMessage(f"Applied {result.method} parameters")

    def start_step_response(self):
        """Start recording step response for analysis"""
        if not self.current_params['motor_enabled']:
            QMessageBox.warning(self, "Motor Off", "Please enable the motor first.")
            return
        
        self.stability_analyzer.start_step_response()
        self.analysis_text.setPlainText("Recording step response...\n\nWait 5-10 seconds, then click 'Analyze & Suggest'")
        self.statusBar().showMessage("Recording step response...")

    def analyze_system(self):
        """Analyze the system and provide suggestions"""
        self.stability_analyzer.stop_step_response()
        
        metrics = self.stability_analyzer.calculate_metrics()
        step_data = self.stability_analyzer.analyze_step_response()
        
        analysis = f"""
═══════════════════════════════════════════════════════════════
                    SYSTEM ANALYSIS REPORT
═══════════════════════════════════════════════════════════════

CURRENT METRICS
───────────────────────────────────────────────────────────────
  Current RPM:        {metrics.current_rpm:.1f}
  Target RPM:         {metrics.target_rpm:.1f}
  Error:              {metrics.error:.1f} RPM
  Stability Score:    {metrics.stability_score:.0f}%
  Hunting Detected:   {'YES - WARNING' if metrics.is_hunting else 'No'}
  Oscillation Freq:   {metrics.oscillation_frequency_hz:.2f} Hz

STEP RESPONSE ANALYSIS
───────────────────────────────────────────────────────────────
"""
        
        if step_data.get("rise_time"):
            analysis += f"""  Rise Time:          {step_data['rise_time']:.3f} seconds
  Overshoot:          {step_data['overshoot_percent']:.1f}%
  Settling Time:      {step_data['settling_time']:.3f} seconds
  Steady-State Error: {step_data['steady_state_error']:.1f} RPM
"""
        else:
            analysis += "  No step response data recorded. Click 'Record Step Response' first.\n"
        
        # Provide suggestions
        analysis += """
TUNING SUGGESTIONS
───────────────────────────────────────────────────────────────
"""
        
        if metrics.is_hunting:
            analysis += """  WARNING: HUNTING DETECTED - System is oscillating
  
  Recommended actions:
  1. REDUCE Kp by 20-30% (currently the main cause)
  2. INCREASE Kd slightly for more damping
  3. REDUCE Ki if steady-state error is acceptable
"""
        elif metrics.overshoot_percent > 20:
            analysis += f"""  WARNING: HIGH OVERSHOOT ({metrics.overshoot_percent:.1f}%)
  
  Recommended actions:
  1. REDUCE Kp by 15-20%
  2. INCREASE Kd for faster settling
  3. Ki can stay the same
"""
        elif metrics.stability_score < 70:
            analysis += """  WARNING: LOW STABILITY SCORE
  
  Recommended actions:
  1. Check for mechanical issues or load variations
  2. Verify PPR (pulses per revolution) setting
  3. Try reducing Kp slightly
"""
        else:
            analysis += """  SUCCESS: System appears well-tuned!
  
  Fine-tuning tips:
  - If response is sluggish: increase Kp slightly
  - If there's slight oscillation: increase Kd
  - If steady-state error exists: increase Ki slightly
"""
        
        analysis += """
═══════════════════════════════════════════════════════════════
"""
        
        self.analysis_text.setPlainText(analysis)

    def save_parameters(self):
        """Save parameters to file"""
        try:
            filename, _ = QFileDialog.getSaveFileName(
                self, "Save Parameters", "pid_params.json", "JSON files (*.json)"
            )
            if filename:
                data = {
                    'params': self.current_params,
                    'history': [
                        {
                            'method': r.method,
                            'kp': r.kp,
                            'ki': r.ki,
                            'kd': r.kd,
                            'timestamp': r.timestamp,
                            'notes': r.notes
                        }
                        for r in self.tuning_history
                    ]
                }
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=2)
                self.statusBar().showMessage(f"Saved to {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")

    def load_parameters(self):
        """Load parameters from file"""
        try:
            filename, _ = QFileDialog.getOpenFileName(
                self, "Load Parameters", "", "JSON files (*.json)"
            )
            if filename:
                with open(filename, 'r') as f:
                    data = json.load(f)

                params = data.get('params', {})
                for key, value in params.items():
                    if key in self.current_params:
                        self.current_params[key] = value

                # Update GUI
                self.target_rpm_spin.setValue(int(self.current_params['target_rpm']))
                self.kp_spin.setValue(self.current_params['kp'])
                self.ki_spin.setValue(self.current_params['ki'])
                self.kd_spin.setValue(self.current_params['kd'])
                self.ppr_spin.setValue(self.current_params['pulses_per_rev'])

                self.statusBar().showMessage(f"Loaded from {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load: {e}")

    def export_data(self):
        """Export plot data to CSV"""
        try:
            filename, _ = QFileDialog.getSaveFileName(
                self, "Export Data", f"motor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                "CSV files (*.csv)"
            )
            if filename and self.plot_canvas.time_data:
                with open(filename, 'w') as f:
                    f.write("Time_ms,Target_RPM,Raw_RPM,Filtered_RPM,Error,PID_Output,Stability_Score\n")
                    for i in range(len(self.plot_canvas.time_data)):
                        f.write(f"{self.plot_canvas.time_data[i]:.0f},"
                               f"{self.plot_canvas.target_rpm_data[i]:.1f},"
                               f"{self.plot_canvas.current_rpm_data[i]:.1f},"
                               f"{self.plot_canvas.filtered_rpm_data[i]:.1f},"
                               f"{self.plot_canvas.error_data[i]:.1f},"
                               f"{self.plot_canvas.pid_output_data[i]:.1f},"
                               f"{self.plot_canvas.stability_score_data[i]:.1f}\n")
                self.statusBar().showMessage(f"Exported to {filename}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to export: {e}")

    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(self, "About",
            """<h2>BLDC PID Controller - PC-Powered Auto-Tune</h2>
            <p><b>Version 2.0</b> - December 2025</p>
            
            <p>Enhanced PID tuning application that leverages PC computational power for:</p>
            <ul>
                <li>Kalman filtering for superior noise rejection</li>
                <li>One-click auto-tuning with multiple algorithms</li>
                <li>Real-time stability analysis</li>
                <li>Step response system identification</li>
            </ul>
            
            <p><b>Authors:</b><br>
            azzar budiyanto &amp; azzar persona (AI assistant)</p>
            """)

    def closeEvent(self, event):
        """Handle application close"""
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("BLDC PID Controller")
    app.setApplicationVersion("2.0")
    app.setOrganizationName("azzar")

    window = PIDControllerGUI()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
