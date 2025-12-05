#!/usr/bin/env python3
"""
BLDC Motor PID Controller - PyQt6 Modern GUI

This modern PyQt6 application provides a professional interface for configuring
and monitoring Arduino-based BLDC motor PID controllers.

Features:
- Modern, responsive PyQt6 interface
- Real-time parameter adjustment with sliders and spinboxes
- Live matplotlib plotting with automatic updates
- Auto PID tuning using Ziegler-Nichols method
- Serial communication with Arduino
- Parameter save/load functionality
- Professional styling and layout

Requirements:
- Python 3.7+
- PyQt6
- matplotlib
- pyserial
- numpy

Install dependencies:
pip install PyQt6 matplotlib pyserial numpy

Author: azzar persona (AI assistant)
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

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFormLayout, QLabel, QLineEdit, QComboBox,
    QPushButton, QSpinBox, QDoubleSpinBox, QSlider, QGroupBox,
    QFrame, QSplitter, QStatusBar, QMessageBox, QFileDialog,
    QProgressBar, QTextEdit, QCheckBox
)
from PyQt6.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QRectF, QPointF
)
from PyQt6.QtGui import QAction, QFont, QPalette, QColor

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

class SerialWorker(QThread):
    """Worker thread for serial communication"""
    data_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.serial_port = None
        self.running = True

    def run(self):
        try:
            # Try to open the serial port with exclusive access
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1,
                write_timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                rtscts=False,
                dsrdtr=False,
                exclusive=True  # Request exclusive access
            )
            print(f"Connected to {self.port} at {self.baud} baud")
            # Flush any pending data
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            self.connection_status.emit(True)

            buffer = ""
            while self.running:
                if self.serial_port.in_waiting:
                    try:
                        # Read available data
                        raw_data = self.serial_port.read(self.serial_port.in_waiting)
                        if len(raw_data) > 0:
                            # Decode and add to buffer
                            decoded = raw_data.decode('utf-8', errors='replace')
                            buffer += decoded

                            # Process complete lines
                            while '\n' in buffer:
                                line_end = buffer.find('\n')
                                line = buffer[:line_end].strip()
                                buffer = buffer[line_end + 1:]

                                # Skip empty lines or corrupted data
                                if not line or len(line) < 5:
                                    continue

                                # Filter out lines that are mostly replacement characters
                                if '\ufffd' not in line or line.count('\ufffd') / len(line) < 0.3:
                                    # Emit STATUS lines and important messages
                                    if line.startswith("STATUS:") or "ERROR" in line or "Kp set" in line or "Ki set" in line or "Kd set" in line:
                                        self.data_received.emit(line)
                                    # Try to recover STATUS data from comma-separated lines
                                    elif ',' in line:
                                        parts = line.split(',')
                                        # Check if this looks like STATUS data (10+ parts, first part numeric)
                                        if len(parts) >= 10:
                                            try:
                                                # First part should be timestamp (numeric)
                                                float(parts[0].strip())
                                                # Check if other parts look numeric
                                                float(parts[1].strip())  # target_rpm
                                                float(parts[2].strip())  # current_rpm
                                                status_line = f"STATUS:{line}"
                                                self.data_received.emit(status_line)
                                                continue
                                            except (ValueError, IndexError):
                                                pass
                                        # Check for partial STATUS data (starts with comma, looks like data continuation)
                                        elif line.startswith(',') and len(parts) >= 9:
                                            try:
                                                float(parts[1].strip())  # target_rpm position in partial line
                                                float(parts[2].strip())  # current_rpm
                                                # Reconstruct with current timestamp since original was lost
                                                current_time = int(time.time() * 1000)  # milliseconds
                                                status_line = f"STATUS:{current_time}{line}"
                                                self.data_received.emit(status_line)
                                                continue
                                            except (ValueError, IndexError):
                                                pass
                                        # Only skip obviously non-STATUS lines
                                        if not any(info_word in line.lower() for info_word in ["heartbeat", "bldc", "ready", "target", "motor"]):
                                            print(f"Skipping non-STATUS line: {line[:50]}...")
                                    else:
                                        # Emit other important Arduino messages to GUI
                                        if len(line) > 3 and not line.lower().startswith("heartbeat"):
                                            self.data_received.emit(line)
                                else:
                                    print(f"Skipping heavily corrupted line: {line[:50]}...")

                    except Exception as decode_error:
                        print(f"Serial decode error: {decode_error}")
                        buffer = ""  # Clear buffer on error
                        continue
                time.sleep(0.02)  # Slightly longer sleep to reduce CPU usage
        except Exception as e:
            error_msg = str(e)
            if "busy" in error_msg.lower() or "resource busy" in error_msg.lower():
                print(f"Serial port {self.port} is busy. Please:")
                print("1. Close Arduino IDE")
                print("2. Kill any serial monitor programs")
                print("3. Stop any other Python GUI instances")
                print("4. Run: sudo fuser -k /dev/ttyACM0")
                print("5. Or: sudo pkill -f arduino")
                self.connection_status.emit(False)
            else:
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

class PlotCanvas(FigureCanvas):
    """Matplotlib canvas for real-time plotting"""
    def __init__(self, parent=None, width=8, height=6, dpi=100):
        self.figure = Figure(figsize=(width, height), dpi=dpi)
        self.figure.patch.set_facecolor('#f0f0f0')

        # Create subplots
        self.axes = []
        for i in range(4):
            ax = self.figure.add_subplot(2, 2, i+1)
            ax.set_facecolor('#ffffff')
            ax.grid(True, alpha=0.3)
            self.axes.append(ax)

        super().__init__(self.figure)
        self.setParent(parent)

        # Setup plots
        self.setup_plots()

        # Data storage (reduced for better performance)
        self.max_points = 25  # Show only latest 25 data points
        self.time_data = []
        self.target_rpm_data = []
        self.current_rpm_data = []
        self.error_data = []
        self.pid_output_data = []
        self.kp_data = []
        self.ki_data = []
        self.kd_data = []

    def setup_plots(self):
        """Setup plot titles and labels"""
        self.axes[0].set_title('Motor Speed', fontsize=10, fontweight='bold')
        self.axes[0].set_ylabel('RPM', fontsize=8)

        self.axes[1].set_title('Control Error', fontsize=10, fontweight='bold')
        self.axes[1].set_ylabel('RPM', fontsize=8)

        self.axes[2].set_title('PID Output', fontsize=10, fontweight='bold')
        self.axes[2].set_ylabel('Output Value', fontsize=8)

        self.axes[3].set_title('PID Gains', fontsize=10, fontweight='bold')
        self.axes[3].set_ylabel('Gain Value', fontsize=8)

        for ax in self.axes:
            ax.tick_params(labelsize=8)
            ax.xaxis.set_tick_params(labelsize=7)
            ax.yaxis.set_tick_params(labelsize=7)

        self.figure.tight_layout(pad=2.0)

    def update_plot(self, time_val, target_rpm, current_rpm, error, pid_output, kp, ki, kd):
        """Update plot data"""
        # Add new data
        self.time_data.append(time_val)
        self.target_rpm_data.append(target_rpm)
        self.current_rpm_data.append(current_rpm)
        self.error_data.append(error)
        self.pid_output_data.append(pid_output)
        self.kp_data.append(kp)
        self.ki_data.append(ki)
        self.kd_data.append(kd)

        # Limit data points
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.target_rpm_data.pop(0)
            self.current_rpm_data.pop(0)
            self.error_data.pop(0)
            self.pid_output_data.pop(0)
            self.kp_data.pop(0)
            self.ki_data.pop(0)
            self.kd_data.pop(0)

        # Ensure all data arrays have the same length
        min_length = min(len(self.time_data), len(self.target_rpm_data), len(self.current_rpm_data),
                        len(self.error_data), len(self.pid_output_data), len(self.kp_data))

        if min_length == 0:
            return  # No data to plot

        # Trim all arrays to the same length
        time_data = self.time_data[-min_length:]
        target_rpm_data = self.target_rpm_data[-min_length:]
        current_rpm_data = self.current_rpm_data[-min_length:]
        error_data = self.error_data[-min_length:]
        pid_output_data = self.pid_output_data[-min_length:]
        kp_data = self.kp_data[-min_length:]
        ki_data = self.ki_data[-min_length:]
        kd_data = self.kd_data[-min_length:]

        # Convert to relative time
        time_rel = [(t - time_data[0]) / 1000 for t in time_data]

        # Clear and replot
        self.axes[0].cla()
        self.axes[1].cla()
        self.axes[2].cla()
        self.axes[3].cla()

        # RPM plot
        self.axes[0].plot(time_rel, target_rpm_data, 'r-', label='Target', linewidth=2)
        self.axes[0].plot(time_rel, current_rpm_data, 'b-', label='Current', linewidth=1.5)
        self.axes[0].set_title('Motor Speed', fontsize=10, fontweight='bold')
        self.axes[0].set_ylabel('RPM', fontsize=8)
        self.axes[0].legend(fontsize=8)
        self.axes[0].grid(True, alpha=0.2)  # Reduced alpha for performance

        # Error plot
        self.axes[1].plot(time_rel, error_data, 'g-', linewidth=1.5)
        self.axes[1].set_title('Control Error', fontsize=10, fontweight='bold')
        self.axes[1].set_ylabel('RPM', fontsize=8)
        self.axes[1].grid(True, alpha=0.2)

        # PID output plot
        self.axes[2].plot(time_rel, pid_output_data, 'm-', linewidth=1.5)
        self.axes[2].set_title('PID Output', fontsize=10, fontweight='bold')
        self.axes[2].set_ylabel('Output Value', fontsize=8)
        self.axes[2].grid(True, alpha=0.2)

        # PID gains plot
        self.axes[3].plot(time_rel, kp_data, 'r-', label='Kp', linewidth=1)
        self.axes[3].plot(time_rel, ki_data, 'g-', label='Ki', linewidth=1)
        self.axes[3].plot(time_rel, kd_data, 'b-', label='Kd', linewidth=1)
        self.axes[3].set_title('PID Gains', fontsize=10, fontweight='bold')
        self.axes[3].set_ylabel('Gain Value', fontsize=8)
        self.axes[3].legend(fontsize=8)
        self.axes[3].grid(True, alpha=0.2)

        # Set consistent styling
        for ax in self.axes:
            ax.tick_params(labelsize=8)
            if time_rel and len(time_rel) > 1:
                time_min, time_max = min(time_rel), max(time_rel)
                if time_min != time_max:  # Avoid identical min/max which causes matplotlib issues
                    ax.set_xlim(time_min, time_max)
                else:
                    # If all times are the same, set a small range around the value
                    ax.set_xlim(time_min - 0.1, time_min + 0.1)
            ax.xaxis.set_tick_params(labelsize=7)
            ax.yaxis.set_tick_params(labelsize=7)

        try:
            # Use subplots_adjust instead of tight_layout for better control
            self.figure.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.95, hspace=0.3)
            self.draw()
        except Exception as plot_error:
            print(f"Plotting error: {plot_error}")
            try:
                self.draw()  # Try drawing without layout adjustment
            except:
                pass  # Continue without crashing

class PIDControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_worker = None
        self.data_queue = queue.Queue()

        # Current parameters
        self.current_params = {
            'target_rpm': 1440.0,
            'kp': 0.25,
            'ki': 0.015,
            'kd': 0.003,
            'pulses_per_rev': 18,
            'motor_enabled': True
        }

        # Connection monitoring
        self.last_data_time = time.time()
        self.connection_timeout = 10  # seconds

        # Plot update optimization
        self.plot_update_counter = 0
        self.plot_update_interval = 5  # Update plot every 5 data points (reduces from 20Hz to ~4Hz)

        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left panel - Controls
        self.create_control_panel()
        splitter.addWidget(self.control_widget)

        # Right panel - Plots
        self.create_plot_panel()
        splitter.addWidget(self.plot_widget)

        main_layout.addWidget(splitter)

        # Set splitter proportions (30% controls, 70% plots)
        splitter.setSizes([300, 700])

        # Initialize serial ports
        self.update_serial_ports()

        # Start data processing timer (10Hz to process incoming serial data)
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.process_serial_data)
        self.data_timer.start(100)  # 10Hz

        # Apply modern styling
        self.apply_modern_style()

    def create_control_panel(self):
        """Create the control panel"""
        self.control_widget = QWidget()
        self.control_widget.setMinimumWidth(400)
        layout = QVBoxLayout(self.control_widget)

        # Serial Connection group
        serial_group = QGroupBox("Serial Connection")
        serial_layout = QFormLayout()

        # Port selection
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        serial_layout.addRow("Port:", self.port_combo)

        # Baud rate
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        serial_layout.addRow("Baud:", self.baud_combo)

        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        serial_layout.addRow("", self.connect_btn)

        # Refresh button
        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.update_serial_ports)
        serial_layout.addRow("", refresh_btn)

        serial_group.setLayout(serial_layout)
        layout.addWidget(serial_group)

        # PID Tuning group
        pid_group = QGroupBox("PID Tuning")
        pid_layout = QFormLayout()

        # Target RPM
        self.target_rpm_spin = QSpinBox()
        self.target_rpm_spin.setRange(0, 5000)
        self.target_rpm_spin.setValue(int(self.current_params['target_rpm']))
        self.target_rpm_spin.setSingleStep(10)
        self.target_rpm_spin.valueChanged.connect(self.update_target_rpm)
        pid_layout.addRow("Target RPM:", self.target_rpm_spin)

        # Kp
        kp_layout = QHBoxLayout()
        self.kp_slider = QSlider(Qt.Orientation.Horizontal)
        self.kp_slider.setRange(0, 500)  # 0-5.0 with 0.01 resolution
        self.kp_slider.setValue(int(self.current_params['kp'] * 100))
        self.kp_slider.valueChanged.connect(self.update_kp)

        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0, 5.0)
        self.kp_spin.setValue(self.current_params['kp'])
        self.kp_spin.setSingleStep(0.01)
        self.kp_spin.setDecimals(3)
        self.kp_spin.valueChanged.connect(self.update_kp_spin)

        kp_layout.addWidget(self.kp_slider)
        kp_layout.addWidget(self.kp_spin)
        pid_layout.addRow("Kp:", kp_layout)

        # Ki
        ki_layout = QHBoxLayout()
        self.ki_slider = QSlider(Qt.Orientation.Horizontal)
        self.ki_slider.setRange(0, 100)  # 0-1.0 with 0.001 resolution
        self.ki_slider.setValue(int(self.current_params['ki'] * 1000))
        self.ki_slider.valueChanged.connect(self.update_ki)

        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0, 1.0)
        self.ki_spin.setValue(self.current_params['ki'])
        self.ki_spin.setSingleStep(0.001)
        self.ki_spin.setDecimals(4)
        self.ki_spin.valueChanged.connect(self.update_ki_spin)

        ki_layout.addWidget(self.ki_slider)
        ki_layout.addWidget(self.ki_spin)
        pid_layout.addRow("Ki:", ki_layout)

        # Kd
        kd_layout = QHBoxLayout()
        self.kd_slider = QSlider(Qt.Orientation.Horizontal)
        self.kd_slider.setRange(0, 10)  # 0-0.1 with 0.0001 resolution
        self.kd_slider.setValue(int(self.current_params['kd'] * 10000))
        self.kd_slider.valueChanged.connect(self.update_kd)

        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0, 0.1)
        self.kd_spin.setValue(self.current_params['kd'])
        self.kd_spin.setSingleStep(0.0001)
        self.kd_spin.setDecimals(5)
        self.kd_spin.valueChanged.connect(self.update_kd_spin)

        kd_layout.addWidget(self.kd_slider)
        kd_layout.addWidget(self.kd_spin)
        pid_layout.addRow("Kd:", kd_layout)

        # Pulses per revolution
        self.ppr_spin = QSpinBox()
        self.ppr_spin.setRange(1, 100)
        self.ppr_spin.setValue(self.current_params['pulses_per_rev'])
        self.ppr_spin.valueChanged.connect(self.update_pulses_per_rev)
        pid_layout.addRow("Pulses/Rev:", self.ppr_spin)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        # Motor Control group
        motor_group = QGroupBox("Motor Control")
        motor_layout = QHBoxLayout()

        self.enable_motor_btn = QPushButton("Enable Motor")
        self.enable_motor_btn.setCheckable(True)
        self.enable_motor_btn.clicked.connect(self.toggle_motor)
        motor_layout.addWidget(self.enable_motor_btn)

        self.reset_btn = QPushButton("Reset Controller")
        self.reset_btn.clicked.connect(self.reset_controller)
        motor_layout.addWidget(self.reset_btn)

        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)

        # Control group
        tune_group = QGroupBox("Control")
        tune_layout = QVBoxLayout()

        tune_buttons = QHBoxLayout()
        # Add diagnostics button

        tune_layout.addLayout(tune_buttons)

        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("font-weight: bold; color: green;")
        tune_layout.addWidget(self.status_label)

        tune_group.setLayout(tune_layout)
        layout.addWidget(tune_group)

        # Add stretch to push everything to top
        layout.addStretch()

    def create_plot_panel(self):
        """Create the plotting panel"""
        self.plot_widget = QWidget()
        layout = QVBoxLayout(self.plot_widget)

        # Create plot canvas
        self.plot_canvas = PlotCanvas(self.plot_widget, width=8, height=6)
        layout.addWidget(self.plot_canvas)

        # Add navigation toolbar
        toolbar = NavigationToolbar(self.plot_canvas, self.plot_widget)
        layout.addWidget(toolbar)

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

        export_action = QAction('Export Data', self)
        export_action.triggered.connect(self.export_data)
        file_menu.addAction(export_action)

        file_menu.addSeparator()

        exit_action = QAction('Exit', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Help menu
        help_menu = menubar.addMenu('Help')

        about_action = QAction('About', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def apply_modern_style(self):
        """Apply modern styling to the application"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
                color: #333333;
            }

            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 10px;
                color: #333333;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 10px 0 10px;
                color: #333333;
                font-weight: bold;
            }

            QPushButton {
                background-color: #ffffff;
                border: 1px solid #cccccc;
                border-radius: 4px;
                padding: 8px 16px;
                font-size: 12px;
                color: #333333;
            }

            QPushButton:hover {
                background-color: #e6f3ff;
                border: 1px solid #0078d4;
                color: #333333;
            }

            QPushButton:pressed {
                background-color: #c7e4f7;
                color: #333333;
            }

            QPushButton:checked {
                background-color: #0078d4;
                color: white;
                border: 1px solid #005a9e;
            }

            QPushButton:disabled {
                background-color: #f3f3f3;
                color: #999999;
                border: 1px solid #cccccc;
            }

            QSlider::groove:horizontal {
                border: 1px solid #cccccc;
                height: 8px;
                background: #ffffff;
                margin: 2px 0;
                border-radius: 4px;
            }

            QSlider::handle:horizontal {
                background: #0078d4;
                border: 1px solid #005a9e;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }

            QSlider::handle:horizontal:hover {
                background: #106ebe;
            }

            QSpinBox, QDoubleSpinBox, QComboBox {
                border: 1px solid #cccccc;
                border-radius: 4px;
                padding: 4px 8px;
                background-color: white;
                min-width: 80px;
                color: #333333;
                selection-background-color: #0078d4;
                selection-color: white;
            }

            QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
                border: 1px solid #0078d4;
            }

            QSpinBox::up-button, QDoubleSpinBox::up-button, QComboBox::drop-down {
                border: none;
                background: transparent;
            }

            QLabel {
                font-size: 12px;
                color: #333333;
            }

            QComboBox::drop-down {
                border: none;
            }

            QComboBox::down-arrow {
                image: none;
                border-left: 4px solid transparent;
                border-right: 4px solid transparent;
                border-top: 4px solid #666666;
                margin-right: 8px;
            }

            QComboBox QAbstractItemView {
                background-color: white;
                color: #333333;
                selection-background-color: #0078d4;
                selection-color: white;
            }

            QMenuBar {
                background-color: #ffffff;
                border-bottom: 1px solid #cccccc;
                color: #333333;
            }

            QMenuBar::item {
                background-color: transparent;
                padding: 4px 8px;
                color: #333333;
            }

            QMenuBar::item:selected {
                background-color: #e6f3ff;
                color: #333333;
            }

            QMenu {
                background-color: white;
                border: 1px solid #cccccc;
                color: #333333;
            }

            QMenu::item:selected {
                background-color: #e6f3ff;
                color: #333333;
            }

            QStatusBar {
                background-color: #ffffff;
                border-top: 1px solid #cccccc;
                color: #333333;
            }
        """)

    def setup_connections(self):
        """Setup signal connections"""
        pass  # Connections are set up in create_control_panel

    def update_serial_ports(self):
        """Update the list of available serial ports"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()

        # Prioritize Arduino-like ports (ACM, USB) over generic serial ports
        arduino_ports = []
        other_ports = []

        for port in ports:
            port_name = port.device.lower()
            description = port.description.lower() if port.description else ""

            # Check if it's likely an Arduino or microcontroller
            if ('acm' in port_name or 'usb' in port_name or
                'arduino' in description or 'uno' in description or
                'mega' in description or 'nano' in description):
                arduino_ports.append(port)
            elif not port_name.startswith('/dev/ttyS'):  # Skip generic serial ports
                other_ports.append(port)

        # Add Arduino ports first
        for port in arduino_ports:
            self.port_combo.addItem(f"🔌 {port.device}: {port.description}", port.device)

        # Add other USB/serial ports
        for port in other_ports:
            self.port_combo.addItem(f"📡 {port.device}: {port.description}", port.device)

        # Add generic serial ports last (less likely to be Arduino)
        for port in ports:
            if port.device.lower().startswith('/dev/ttyS') and port not in arduino_ports and port not in other_ports:
                self.port_combo.addItem(f"{port.device}: {port.description}", port.device)

        if arduino_ports:
            # Select first Arduino port if available
            self.port_combo.setCurrentIndex(0)
        elif ports:
            # Otherwise select first available port
            self.port_combo.setCurrentIndex(0)

    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.serial_worker and self.serial_worker.isRunning():
            # Disconnect
            self.serial_worker.stop()
            self.serial_worker = None
            self.connect_btn.setText("Connect")
            self.statusBar().showMessage("Disconnected")
        else:
            # Connect
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "Connection Error", "No serial port selected")
                return

            baud = int(self.baud_combo.currentText())

            self.serial_worker = SerialWorker(port, baud)
            self.serial_worker.data_received.connect(self.on_data_received)
            self.serial_worker.connection_status.connect(self.on_connection_status)
            self.serial_worker.start()

            self.connect_btn.setText("Connecting...")

    def on_connection_status(self, connected):
        """Handle connection status changes"""
        if connected:
            self.connect_btn.setText("Disconnect")
            self.statusBar().showMessage(f"Connected to {self.port_combo.currentData()}")
            self.last_data_time = time.time()  # Reset timeout counter
            # Request initial status
            print("Requesting initial status from Arduino...")
            QTimer.singleShot(1000, lambda: self.send_command("GET_STATUS"))
        else:
            self.connect_btn.setText("Connect")
            self.statusBar().showMessage("Connection failed")

    def on_data_received(self, data):
        """Handle incoming serial data"""
        self.last_data_time = time.time()
        self.data_queue.put(data)

    def process_serial_data(self):
        """Process data from the serial queue"""
        try:
            while not self.data_queue.empty():
                line = self.data_queue.get_nowait()
                if line.startswith("STATUS:"):
                    self.parse_status_data(line)
                elif line.startswith("HEARTBEAT"):
                    # Skip heartbeat messages - too noisy
                    pass
                elif "ERROR" in line or "set to" in line:
                    # Show important status messages
                    self.statusBar().showMessage(f"Arduino: {line}")
                elif len(line.strip()) > 0 and not line.lower().startswith(("bldc", "ready", "target", "kp:", "ki:", "kd:", "motor")):
                    # Show other important Arduino messages (but filter out common startup messages)
                    self.statusBar().showMessage(f"Arduino: {line}")

            # Check for connection timeout
            if self.serial_worker and self.serial_worker.isRunning():
                if time.time() - self.last_data_time > self.connection_timeout:
                    self.statusBar().showMessage("WARNING: No data from Arduino - check connection and port")
                    print("Warning: No data received from Arduino for 10 seconds")
                else:
                    # Show data is being received
                    self.statusBar().showMessage(f"✓ Connected - receiving data from {self.port_combo.currentData()}")
        except:
            pass

    def parse_status_data(self, data):
        """Parse status data from Arduino"""
        try:
            # Skip if data doesn't start with STATUS:
            if not data.startswith("STATUS:"):
                return

            parts = data[7:].split(',')
            # Require exactly 10 parts (timestamp + 9 data values) after removing auto_tune
            if len(parts) != 10:
                print(f"Invalid status data format: expected 10 parts, got {len(parts)}: {data}")
                return

            # Parse each field with error checking
            timestamp = float(parts[0])
            target_rpm = float(parts[1])
            current_rpm = float(parts[2])
            error = float(parts[3])
            pid_output = float(parts[4])
            kp = float(parts[5])
            ki = float(parts[6])
            kd = float(parts[7])
            ppr = int(float(parts[8]))  # Convert to float first to handle decimal strings
            motor_enabled = int(float(parts[9]))

            # Update plot less frequently to improve performance
            self.plot_update_counter += 1
            if self.plot_update_counter >= self.plot_update_interval:
                self.plot_canvas.update_plot(timestamp, target_rpm, current_rpm,
                                           error, pid_output, kp, ki, kd)
                self.plot_update_counter = 0

            # Update current values if not being adjusted by user
            if not self.kp_slider.isSliderDown():
                self.kp_spin.blockSignals(True)
                self.kp_spin.setValue(kp)
                self.kp_slider.setValue(int(kp * 100))
                self.kp_spin.blockSignals(False)

            if not self.ki_slider.isSliderDown():
                self.ki_spin.blockSignals(True)
                self.ki_spin.setValue(ki)
                self.ki_slider.setValue(int(ki * 1000))
                self.ki_spin.blockSignals(False)

            if not self.kd_slider.isSliderDown():
                self.kd_spin.blockSignals(True)
                self.kd_spin.setValue(kd)
                self.kd_slider.setValue(int(kd * 10000))
                self.kd_spin.blockSignals(False)

            # Update target RPM and PPR if changed by Arduino
            self.target_rpm_spin.setValue(int(target_rpm))
            self.ppr_spin.setValue(ppr)

                
        except Exception as e:
            print(f"Error parsing status data: {e}")

    def send_command(self, command):
        """Send command to Arduino"""
        if self.serial_worker:
            self.serial_worker.send_command(command)

    def update_target_rpm(self, value):
        """Update target RPM"""
        self.current_params['target_rpm'] = value
        self.send_command(f"SET_TARGET_RPM {value}")

    def update_kp(self, value):
        """Update Kp gain from slider"""
        kp = value / 100.0
        self.current_params['kp'] = kp
        self.kp_spin.blockSignals(True)
        self.kp_spin.setValue(kp)
        self.kp_spin.blockSignals(False)
        self.send_command(f"SET_KP {kp}")

    def update_kp_spin(self, value):
        """Update Kp gain from spinbox"""
        self.current_params['kp'] = value
        self.kp_slider.blockSignals(True)
        self.kp_slider.setValue(int(value * 100))
        self.kp_slider.blockSignals(False)
        self.send_command(f"SET_KP {value}")

    def update_ki(self, value):
        """Update Ki gain from slider"""
        ki = value / 1000.0
        self.current_params['ki'] = ki
        self.ki_spin.blockSignals(True)
        self.ki_spin.setValue(ki)
        self.ki_spin.blockSignals(False)
        self.send_command(f"SET_KI {ki}")

    def update_ki_spin(self, value):
        """Update Ki gain from spinbox"""
        self.current_params['ki'] = value
        self.ki_slider.blockSignals(True)
        self.ki_slider.setValue(int(value * 1000))
        self.ki_slider.blockSignals(False)
        self.send_command(f"SET_KI {value}")

    def update_kd(self, value):
        """Update Kd gain from slider"""
        kd = value / 10000.0
        self.current_params['kd'] = kd
        self.kd_spin.blockSignals(True)
        self.kd_spin.setValue(kd)
        self.kd_spin.blockSignals(False)
        self.send_command(f"SET_KD {kd}")

    def update_kd_spin(self, value):
        """Update Kd gain from spinbox"""
        self.current_params['kd'] = value
        self.kd_slider.blockSignals(True)
        self.kd_slider.setValue(int(value * 10000))
        self.kd_slider.blockSignals(False)
        self.send_command(f"SET_KD {value}")

    def update_pulses_per_rev(self, value):
        """Update pulses per revolution"""
        self.current_params['pulses_per_rev'] = value
        self.send_command(f"SET_PULSES_PER_REV {value}")

    def toggle_motor(self, checked):
        """Toggle motor enable/disable"""
        self.current_params['motor_enabled'] = checked
        if checked:
            self.enable_motor_btn.setText("Disable Motor")
            self.send_command("ENABLE_MOTOR 1")
        else:
            self.enable_motor_btn.setText("Enable Motor")
            self.send_command("ENABLE_MOTOR 0")

    def reset_controller(self):
        """Reset PID controller"""
        self.send_command("RESET_CONTROLLER")
        self.statusBar().showMessage("Controller reset")

    def save_parameters(self):
        """Save current parameters to file"""
        try:
            filename, _ = QFileDialog.getSaveFileName(
                self, "Save Parameters", "", "JSON files (*.json);;All files (*)"
            )
            if filename:
                with open(filename, 'w') as f:
                    json.dump(self.current_params, f, indent=4)
                QMessageBox.information(self, "Success", "Parameters saved successfully")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save parameters: {str(e)}")

    def load_parameters(self):
        """Load parameters from file"""
        try:
            filename, _ = QFileDialog.getOpenFileName(
                self, "Load Parameters", "", "JSON files (*.json);;All files (*)"
            )
            if filename:
                with open(filename, 'r') as f:
                    params = json.load(f)

                # Update parameters
                for key, value in params.items():
                    if key in self.current_params:
                        self.current_params[key] = value

                # Update GUI
                self.target_rpm_spin.setValue(int(self.current_params['target_rpm']))
                self.kp_spin.setValue(self.current_params['kp'])
                self.ki_spin.setValue(self.current_params['ki'])
                self.kd_spin.setValue(self.current_params['kd'])
                self.ppr_spin.setValue(self.current_params['pulses_per_rev'])

                # Send to Arduino
                self.send_command(f"SET_TARGET_RPM {self.current_params['target_rpm']}")
                self.send_command(f"SET_KP {self.current_params['kp']}")
                self.send_command(f"SET_KI {self.current_params['ki']}")
                self.send_command(f"SET_KD {self.current_params['kd']}")
                self.send_command(f"SET_PULSES_PER_REV {self.current_params['pulses_per_rev']}")

                QMessageBox.information(self, "Success", "Parameters loaded successfully")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load parameters: {str(e)}")

    def export_data(self):
        """Export current plot data to CSV"""
        try:
            filename, _ = QFileDialog.getSaveFileName(
                self, "Export Data", "", "CSV files (*.csv);;All files (*)"
            )
            if filename and self.plot_canvas.time_data:
                with open(filename, 'w') as f:
                    f.write("Time(ms),Target_RPM,Current_RPM,Error,PID_Output,Kp,Ki,Kd\n")
                    for i in range(len(self.plot_canvas.time_data)):
                        f.write(f"{self.plot_canvas.time_data[i]:.2f},{self.plot_canvas.target_rpm_data[i]:.2f},{self.plot_canvas.current_rpm_data[i]:.2f},{self.plot_canvas.error_data[i]:.2f},{self.plot_canvas.pid_output_data[i]:.2f},{self.plot_canvas.kp_data[i]:.4f},{self.plot_canvas.ki_data[i]:.4f},{self.plot_canvas.kd_data[i]:.4f}\n")
                QMessageBox.information(self, "Success", "Data exported successfully")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to export data: {str(e)}")

    def show_about(self):
        """Show about dialog"""
        about_text = """BLDC Motor PID Controller - PyQt6 GUI

Version 1.0
December 2025

This modern PyQt6 application provides a professional interface
for configuring and monitoring Arduino-based BLDC motor PID controllers.

Features:
- Modern, responsive PyQt6 interface with professional styling
- Real-time parameter adjustment with sliders and spinboxes
- Live matplotlib plotting with automatic updates
- Auto PID tuning using Ziegler-Nichols method
- Serial communication with Arduino
- Parameter save/load functionality
- Data export capabilities

Author: azzar persona (AI assistant)"""

        QMessageBox.about(self, "About", about_text)

    def closeEvent(self, event):
        """Handle application close"""
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()









def main():
    app = QApplication(sys.argv)

    # Set application properties
    app.setApplicationName("BLDC PID Controller")
    app.setApplicationVersion("1.0")
    app.setOrganizationName("azzar")

    window = PIDControllerGUI()
    window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
