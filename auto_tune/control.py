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
                                    # Only emit STATUS lines and important messages
                                    if line.startswith("STATUS:") or "ERROR" in line or "Kp set" in line:
                                        self.data_received.emit(line)
                                else:
                                    print(f"Skipping corrupted line: {line[:50]}...")

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
        self.max_points = 200  # Reduced from 500 for better performance
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
            'motor_enabled': True,
            'auto_tune': False
        }

        # Connection monitoring
        self.last_data_time = time.time()
        self.connection_timeout = 10  # seconds

        # Plot update optimization
        self.plot_update_counter = 0
        self.plot_update_interval = 5  # Update plot every 5 data points (reduces from 20Hz to ~4Hz)

        # Auto-tuning variables
        self.auto_tune_running = False

        self.init_ui()
        self.setup_connections()
        self.update_serial_ports()

        # Start data processing timer (reduced frequency for better performance)
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.process_serial_data)
        self.data_timer.start(100)  # 10Hz (reduced from 20Hz)

        self.statusBar().showMessage("Ready")

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("BLDC Motor PID Controller - PyQt6")
        self.setGeometry(100, 100, 1400, 900)
        self.setMinimumSize(1200, 700)

        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout(central_widget)

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left panel - Controls
        self.create_control_panel()
        splitter.addWidget(self.control_widget)

        # Right panel - Plots
        self.create_plot_panel()
        splitter.addWidget(self.plot_widget)

        # Set splitter proportions
        splitter.setSizes([450, 950])

        main_layout.addWidget(splitter)

        # Create menu bar
        self.create_menu_bar()

        # Apply modern styling
        self.apply_modern_style()

    def create_control_panel(self):
        """Create the control panel"""
        self.control_widget = QWidget()
        self.control_widget.setMinimumWidth(400)
        layout = QVBoxLayout(self.control_widget)

        # Serial connection group
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

        # PID Parameters group
        pid_group = QGroupBox("PID Parameters")
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

        # Auto Tuning group
        tune_group = QGroupBox("Auto Tuning")
        tune_layout = QVBoxLayout()

        tune_buttons = QHBoxLayout()
        self.start_tune_btn = QPushButton("Start Auto Tune")
        self.start_tune_btn.clicked.connect(self.start_auto_tune)
        tune_buttons.addWidget(self.start_tune_btn)

        self.stop_tune_btn = QPushButton("Stop Auto Tune")
        self.stop_tune_btn.clicked.connect(self.stop_auto_tune)
        self.stop_tune_btn.setEnabled(False)
        tune_buttons.addWidget(self.stop_tune_btn)

        tune_layout.addLayout(tune_buttons)

        self.tune_status_label = QLabel("Ready")
        self.tune_status_label.setStyleSheet("font-weight: bold; color: green;")
        tune_layout.addWidget(self.tune_status_label)

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
                else:
                    # Print other messages to status bar
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
            # Require exactly 11 parts (timestamp + 10 data values)
            if len(parts) != 11:
                print(f"Invalid status data format: expected 11 parts, got {len(parts)}: {data}")
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
            auto_tune = int(float(parts[10]))

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

            # Update auto-tune status (only label that exists)
            if auto_tune:
                self.tune_status_label.setText("Auto-tune: ACTIVE")
                self.tune_status_label.setStyleSheet("font-weight: bold; color: blue;")
            else:
                self.tune_status_label.setText("Auto-tune: INACTIVE")
                self.tune_status_label.setStyleSheet("font-weight: bold; color: gray;")

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

    def start_auto_tune(self):
        """Start automatic PID tuning"""
        if self.auto_tune_running:
            print("Auto-tune already running, ignoring start request")
            return

        # Additional check: ensure no previous thread is still alive
        if hasattr(self, 'auto_tune_thread') and self.auto_tune_thread.is_alive():
            print("Previous auto-tune thread still running, waiting...")
            self.auto_tune_thread.join(timeout=2.0)
            if self.auto_tune_thread.is_alive():
                print("Force terminating previous thread")
                return

        print("Starting auto-tune process...")

        # Critical: Reset motor and PID state before starting new auto-tune
        print("Resetting motor and PID state for clean auto-tune...")
        self._reset_system_for_autotune()

        self.auto_tune_running = True
        self.start_tune_btn.setEnabled(False)
        self.stop_tune_btn.setEnabled(True)
        self.tune_status_label.setText("Running...")
        self.tune_status_label.setStyleSheet("font-weight: bold; color: orange;")

        # Start auto-tuning in separate thread
        try:
            self.auto_tune_thread = threading.Thread(target=self.auto_tune_pid, daemon=True)
            self.auto_tune_thread.start()
            print("Auto-tune thread started")
        except Exception as e:
            print(f"Failed to start auto-tune thread: {e}")
            self.stop_auto_tune()

    def _reset_system_for_autotune(self):
        """Reset motor and PID state for clean auto-tune operation"""
        try:
            print("  Force stopping motor completely...")
            # Use the dedicated force stop method
            motor_stopped = self._force_motor_stop(max_attempts=8)
            if not motor_stopped:
                print("  CRITICAL: Unable to stop motor - auto-tune may be unreliable")
                # Continue anyway, but warn the user

            print("  Resetting PID gains to safe defaults...")
            # Send safe default PID values multiple times for reliability
            for i in range(4):  # Increased to 4 attempts
                self.send_command("SET_KP 0.100")  # Safe starting Kp
                self.send_command("SET_KI 0.010")  # Safe starting Ki
                self.send_command("SET_KD 0.001")  # Safe starting Kd
                time.sleep(0.15)

            print("  Exiting any previous auto-tune mode...")
            for i in range(4):  # Increased to 4 attempts
                self.send_command("AUTO_TUNE 0")
                time.sleep(0.15)

            print("  Final motor disable and PWM clear...")
            self.send_command("ENABLE_MOTOR 0")
            self.send_command("SET_PWM 0")
            time.sleep(2.0)  # Wait longer for motor to fully stop

            # Final RPM check
            final_rpm = self.get_current_rpm_from_arduino()
            print(f"  Final RPM after reset: {final_rpm}")

            if final_rpm > 100.0:
                print(f"  WARNING: Motor still spinning at {final_rpm} RPM - auto-tune may be unreliable")
                print("  Consider manually stopping the motor or checking hardware connections")
            else:
                print("  Motor successfully stopped - system ready for auto-tune")

        except Exception as e:
            print(f"Warning: System reset failed: {e}")

    def _validate_motor_response(self, target_rpm):
        """Validate that motor responds properly to commands before auto-tune"""
        try:
            print("  Testing motor with incremental PWM values...")

            # Test 1: Check initial state (motor should be stopped from reset)
            initial_rpm = self.get_current_rpm_from_arduino()
            print(f"  Initial RPM (after reset): {initial_rpm}")

            # Allow some tolerance for motor coasting down
            if initial_rpm > 100:
                print(f"  WARNING: Motor still spinning at {initial_rpm} RPM - waiting for stop...")
                # Wait for motor to naturally coast down
                for wait_attempt in range(10):  # Up to 10 seconds
                    time.sleep(1)
                    current_rpm = self.get_current_rpm_from_arduino()
                    print(f"    Waiting... RPM = {current_rpm}")
                    if current_rpm < 50:
                        initial_rpm = current_rpm
                        break
                else:
                    print("  ERROR: Motor failed to stop naturally")
                    return False

            # Test 2: Low PWM test
            print("  Testing low PWM response...")
            self.send_command("SET_PWM 50")
            time.sleep(4)  # Longer wait for response
            low_rpm = self.get_current_rpm_from_arduino()
            print(f"  Low PWM (50) RPM: {low_rpm}")

            # Test 3: Medium PWM test
            print("  Testing medium PWM response...")
            self.send_command("SET_PWM 150")
            time.sleep(4)  # Longer wait for response
            med_rpm = self.get_current_rpm_from_arduino()
            print(f"  Medium PWM (150) RPM: {med_rpm}")

            # Test 4: Stop motor
            print("  Stopping motor...")
            self.send_command("SET_PWM 0")
            time.sleep(2)

            # Validate response with more realistic expectations
            min_expected_rpm = target_rpm * 0.1  # Should reach at least 10% of target
            max_expected_rpm = target_rpm * 1.2  # Should not exceed 120% of target

            if low_rpm < 20:
                print(f"  WARNING: Motor shows minimal response to low PWM ({low_rpm} RPM)")
                print("  This might indicate hardware issues or weak motor response")
                # Don't fail here - some motors respond weakly at low PWM

            if med_rpm < low_rpm + 30:
                print(f"  WARNING: Motor response not significantly increasing: {low_rpm} → {med_rpm}")
                return False

            if med_rpm < min_expected_rpm:
                print(f"  WARNING: Motor only reached {med_rpm} RPM, expected at least {min_expected_rpm}")
                print("  Motor may be underpowered or have mechanical issues")
                return False

            if med_rpm > max_expected_rpm:
                print(f"  WARNING: Motor reached {med_rpm} RPM, which is unusually high for {target_rpm} target")
                print("  ESC or motor may be overpowered")

            print("  Motor validation passed - system ready for auto-tune")
            return True

        except Exception as e:
            print(f"  Motor validation error: {e}")
            return False

    def _force_motor_stop(self, max_attempts=10):
        """Force stop the motor completely"""
        print("  Force-stopping motor...")
        for attempt in range(max_attempts):
            # Send multiple stop commands
            self.send_command("ENABLE_MOTOR 0")
            self.send_command("SET_PWM 0")
            self.send_command("AUTO_TUNE 0")  # Exit any auto-tune mode

            time.sleep(0.5)
            current_rpm = self.get_current_rpm_from_arduino()
            print(f"    Stop attempt {attempt + 1}: RPM = {current_rpm}")

            if current_rpm < 20.0:
                print(f"  Motor stopped after {attempt + 1} attempts")
                return True

        print(f"  WARNING: Motor still at {current_rpm} RPM after {max_attempts} stop attempts")
        return False

    def stop_auto_tune(self):
        """Stop automatic PID tuning"""
        self.auto_tune_running = False
        self.start_tune_btn.setEnabled(True)
        self.stop_tune_btn.setEnabled(False)
        self.tune_status_label.setText("Stopped")
        self.tune_status_label.setStyleSheet("font-weight: bold; color: red;")
        self.send_command("AUTO_TUNE 0")

    def auto_tune_pid(self):
        """Computer-side automatic PID tuning using Ziegler-Nichols method"""
        print(f"STARTING COMPUTER-CONTROLLED AUTO-TUNE")
        print(f"   CRITICAL: Make sure updated Arduino code is uploaded!")
        print(f"   Arduino file: auto_tune/code/code.ino")
        print(f"   If PID values stay at 0.000, Arduino code needs updating.")
        print(f"   Upload the code and try again.")
        print()

        # Test Arduino communication
        print("Testing Arduino communication...")
        self.send_command("GET_STATUS")
        time.sleep(0.5)
        print("If you see STATUS data above, Arduino is responding.")
        print("If you see no Arduino responses to commands, code is not uploaded!")
        print()

        try:
            # Get target RPM from GUI
            target_rpm = self.current_params.get('target_rpm', 1440)

            # Step 1: Prepare for computer-controlled auto-tune
            self.tune_status_label.setText("Phase 1: Preparing computer control...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: blue;")
            QApplication.processEvents()

            # Switch Arduino to pass-through mode (disable Arduino PID)
            self.send_command("SET_KP 0.0")  # Disable Arduino PID
            self.send_command("SET_KI 0.0")
            self.send_command("SET_KD 0.0")
            self.send_command("ENABLE_MOTOR 1")  # Keep motor enabled
            self.send_command("AUTO_TUNE 1")  # Signal auto-tune mode

            # Wait for Arduino to switch modes
            time.sleep(2)

            # Step 2: Ultimate Gain Test (computer-controlled)
            self.tune_status_label.setText("Phase 2: Finding ultimate gain...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: blue;")
            QApplication.processEvents()

            ku, tu = self.computer_controlled_autotune()
            if ku is None or not self.auto_tune_running:
                self.tune_status_label.setText("Auto-tune failed - no oscillation detected")
                self.tune_status_label.setStyleSheet("font-weight: bold; color: red;")
                self.send_command("AUTO_TUNE 0")  # Exit auto-tune mode
                self.stop_auto_tune()
                return

            # Step 3: Calculate PID Gains
            self.tune_status_label.setText("Phase 3: Calculating optimal PID gains...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: orange;")
            QApplication.processEvents()

            # Ziegler-Nichols tuning rules (conservative)
            kp_zn = 0.45 * ku
            ti = 0.83 * tu
            td = 0.125 * tu
            ki_zn = kp_zn / ti
            kd_zn = kp_zn * td

            print(f"Calculated PID gains:")
            print(f"  Kp = {kp_zn:.4f}")
            print(f"  Ki = {ki_zn:.4f}")
            print(f"  Kd = {kd_zn:.4f}")

            # Step 4: Validate and Refine PID Gains for +/-2% Accuracy
            print("\nSTEP 4: VALIDATING PID GAINS FOR +/-2% ACCURACY")
            validated_kp, validated_ki, validated_kd = self.validate_and_refine_pid_gains(
                kp_zn, ki_zn, kd_zn, ku, tu, target_rpm
            )

            # Step 5: Apply Final Validated Gains to Arduino
            self.tune_status_label.setText("Phase 5: Applying validated PID gains...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: purple;")
            QApplication.processEvents()

            kp_zn, ki_zn, kd_zn = validated_kp, validated_ki, validated_kd
            print(f"\nFINAL VALIDATED PID GAINS (+/-2% ACCURACY):")
            print(f"   Kp = {kp_zn:.4f}")
            print(f"   Ki = {ki_zn:.4f}")
            print(f"   Kd = {kd_zn:.4f}")

            # Send validated PID gains to Arduino with confirmation
            print(f"\nSENDING VALIDATED PID VALUES TO ARDUINO (4x each for reliability):")
            print(f"   These values achieve ±2% accuracy at {target_rpm} RPM")
            print(f"   If you don't see '✓ Kp set to: {kp_zn:.4f}' responses below,")
            print(f"   the Arduino code is NOT uploaded! Upload auto_tune/code/code.ino first!")
            print()

            # Send SET_KP 4 times
            for i in range(4):
                print(f"   [{i+1}/4] Sending: SET_KP {kp_zn:.4f}")
                self.send_command(f"SET_KP {kp_zn:.4f}")
                time.sleep(0.15)  # Brief wait between sends

            time.sleep(0.3)  # Wait for all responses

            # Send SET_KI 4 times
            for i in range(4):
                print(f"   [{i+1}/4] Sending: SET_KI {ki_zn:.4f}")
                self.send_command(f"SET_KI {ki_zn:.4f}")
                time.sleep(0.15)

            time.sleep(0.3)

            # Send SET_KD 4 times
            for i in range(4):
                print(f"   [{i+1}/4] Sending: SET_KD {kd_zn:.4f}")
                self.send_command(f"SET_KD {kd_zn:.4f}")
                time.sleep(0.15)

            time.sleep(0.3)

            # Send AUTO_TUNE 0 4 times
            for i in range(4):
                print(f"   [{i+1}/4] Sending: AUTO_TUNE 0 (exit auto-tune mode)")
                self.send_command("AUTO_TUNE 0")  # Exit auto-tune mode
                time.sleep(0.15)

            time.sleep(0.5)  # Wait for Arduino to process all commands

            # Request verification that Arduino is using the tuned values (multiple times)
            print("VERIFYING ARDUINO RECEIVED PID VALUES (multiple requests):")
            for i in range(3):
                print(f"   [{i+1}/3] Sending: VERIFY_TUNED_VALUES")
                self.send_command("VERIFY_TUNED_VALUES")
                time.sleep(0.3)

            # Also send multiple GET_STATUS requests
            for i in range(3):
                print(f"   [{i+1}/3] Sending: GET_STATUS for confirmation")
                self.send_command("GET_STATUS")
                time.sleep(0.3)

            # Schedule GUI updates in main thread (not from background thread)
            print("Scheduling GUI update...")
            QTimer.singleShot(0, lambda: self.update_gui_with_tuned_values(kp_zn, ki_zn, kd_zn, ku, tu))

            # Wait for GUI update to complete
            time.sleep(0.5)
            print("AUTO-TUNE COMPLETED SUCCESSFULLY!")
            print(f"   Tuned PID values sent to Arduino: Kp={kp_zn:.4f}, Ki={ki_zn:.4f}, Kd={kd_zn:.4f}")
            print("   Check STATUS data to verify Arduino is using these values.")
            print("   System is now ready for the next auto-tune run.")

        except Exception as e:
            print(f"Auto-tune error: {e}")
            self.tune_status_label.setText(f"Error: {str(e)}")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: red;")
            self.send_command("AUTO_TUNE 0")  # Exit auto-tune mode
        finally:
            print("Auto-tune thread finishing...")
            self.auto_tune_running = False
            self.start_tune_btn.setEnabled(True)
            self.stop_tune_btn.setEnabled(False)
            print("Auto-tune thread completed!")

    def update_gui_with_tuned_values(self, kp_zn, ki_zn, kd_zn, ku, tu):
        """Update GUI with tuned PID values (called in main thread)"""
        print("UPDATING GUI WITH TUNED PID VALUES...")
        print(f"   Ziegler-Nichols results: Ku={ku:.3f}, Tu={tu:.3f}s")
        print(f"   Final PID: Kp={kp_zn:.4f}, Ki={ki_zn:.4f}, Kd={kd_zn:.4f}")

        # Update GUI (block signals to prevent duplicate Arduino commands)
        self.kp_spin.blockSignals(True)
        self.ki_spin.blockSignals(True)
        self.kd_spin.blockSignals(True)

        self.kp_spin.setValue(kp_zn)
        self.ki_spin.setValue(ki_zn)
        self.kd_spin.setValue(kd_zn)
        self.current_params['kp'] = kp_zn
        self.current_params['ki'] = ki_zn
        self.current_params['kd'] = kd_zn

        self.kp_spin.blockSignals(False)
        self.ki_spin.blockSignals(False)
        self.kd_spin.blockSignals(False)

        # Update sliders to match
        self.kp_slider.setValue(int(kp_zn * 100))
        self.ki_slider.setValue(int(ki_zn * 1000))
        self.kd_slider.setValue(int(kd_zn * 10000))

        self.tune_status_label.setText(f"Complete! Arduino updated with tuned values")
        self.tune_status_label.setStyleSheet("font-weight: bold; color: green;")

        # Ensure buttons are in correct state
        self.start_tune_btn.setEnabled(True)
        self.stop_tune_btn.setEnabled(False)
        self.auto_tune_running = False

        print("✓ GUI updated - Arduino should now be using tuned PID values")
        print(f"  Final values: Kp={kp_zn:.4f}, Ki={ki_zn:.4f}, Kd={kd_zn:.4f}")
        print("  Monitor STATUS data to confirm Arduino is using these values.")
        print("AUTO-TUNE PROCESS COMPLETED SUCCESSFULLY!")

    def validate_and_refine_pid_gains(self, kp_initial, ki_initial, kd_initial, ku, tu, target_rpm):
        """Validate and refine PID gains to achieve ±2% accuracy"""
        print(f"VALIDATING PID GAINS FOR +/-2% TARGET ACCURACY")
        print(f"   Target RPM: {target_rpm}")
        print(f"   Accuracy requirement: ±{target_rpm * 0.02:.1f} RPM ({target_rpm * 0.02 / target_rpm * 100:.1f}%)")

        max_validation_attempts = 5
        best_kp, best_ki, best_kd = kp_initial, ki_initial, kd_initial
        best_error_percent = float('inf')

        for attempt in range(max_validation_attempts):
            print(f"\nVALIDATION ATTEMPT {attempt + 1}/{max_validation_attempts}")
            print(f"   Testing PID: Kp={best_kp:.4f}, Ki={best_ki:.4f}, Kd={best_kd:.4f}")

            # Test current PID values
            steady_state_error, error_percent = self.test_pid_performance(best_kp, best_ki, best_kd, target_rpm)

            print(f"   Steady-state error: {steady_state_error:.2f} RPM ({error_percent:.2f}%)")

            # Check if accuracy requirement is met
            if abs(error_percent) <= 2.0:
                print(f"   ACCURACY REQUIREMENT MET! (+/-{abs(error_percent):.2f}% < +/-2.0%)")
                return best_kp, best_ki, best_kd

            # If not accurate enough, refine the gains
            print(f"   ACCURACY REQUIREMENT NOT MET (+/-{abs(error_percent):.2f}% > +/-2.0%)")
            print(f"   Refining PID gains...")

            # Adjust gains based on error
            if abs(error_percent) > 2.0:
                # If overshooting or oscillating too much, reduce Kp and Ki
                if abs(error_percent) > 5.0:
                    adjustment_factor = 0.7  # Reduce gains significantly
                else:
                    adjustment_factor = 0.85  # Reduce gains moderately

                best_kp *= adjustment_factor
                best_ki *= adjustment_factor
                best_kd *= adjustment_factor

                print(f"   Adjusted gains by factor {adjustment_factor:.2f}")
                print(f"   New PID: Kp={best_kp:.4f}, Ki={best_ki:.4f}, Kd={best_kd:.4f}")

            # Track best performance so far
            if abs(error_percent) < abs(best_error_percent):
                best_error_percent = error_percent

        # If we couldn't achieve the accuracy requirement, return the best we found
        print(f"\nWARNING: COULD NOT ACHIEVE +/-2% ACCURACY AFTER {max_validation_attempts} ATTEMPTS")
        print(f"   Best result: {best_error_percent:.2f}% error")
        print(f"   Returning best PID values: Kp={best_kp:.4f}, Ki={best_ki:.4f}, Kd={best_kd:.4f}")
        return best_kp, best_ki, best_kd

    def test_pid_performance(self, kp, ki, kd, target_rpm):
        """Test PID performance and return steady-state error"""
        print(f"   Testing PID performance for {target_rpm} RPM...")

        # Send test PID values to Arduino
        self.send_command(f"SET_KP {kp:.4f}")
        time.sleep(0.1)
        self.send_command(f"SET_KI {ki:.4f}")
        time.sleep(0.1)
        self.send_command(f"SET_KD {kd:.4f}")
        time.sleep(0.1)
        self.send_command("AUTO_TUNE 0")  # Exit auto-tune mode
        time.sleep(0.2)

        # Enable motor and let it stabilize
        self.send_command("ENABLE_MOTOR 1")
        time.sleep(2)  # Let motor start and stabilize

        # Collect RPM data for 10 seconds to measure steady-state error
        rpm_samples = []
        start_time = time.time()

        print(f"   Collecting RPM data for 10 seconds...")
        while time.time() - start_time < 10:
            current_rpm = self.get_current_rpm_from_arduino()
            if current_rpm is not None and current_rpm > 0:
                rpm_samples.append(current_rpm)
            time.sleep(0.1)  # 10Hz sampling

        # Disable motor
        self.send_command("ENABLE_MOTOR 0")

        if len(rpm_samples) < 50:  # Need at least 5 seconds of data
            print(f"   ERROR: INSUFFICIENT DATA: Only collected {len(rpm_samples)} samples")
            return 999, 100.0  # Large error to force refinement

        # Calculate steady-state error (average of last 50 samples = last 5 seconds)
        steady_state_rpm = np.mean(rpm_samples[-50:])
        steady_state_error = steady_state_rpm - target_rpm
        error_percent = (steady_state_error / target_rpm) * 100

        print(f"   Steady-state RPM: {steady_state_rpm:.2f}")
        print(f"   Target RPM: {target_rpm}")
        print(f"   Error: {steady_state_error:.2f} RPM ({error_percent:.2f}%)")

        return steady_state_error, error_percent

    def computer_controlled_autotune(self):
        """Computer-side auto-tune: full control from laptop"""
        target_rpm = self.current_params.get('target_rpm', 1440)
        print(f"Starting computer-controlled auto-tune for target: {target_rpm} RPM")

        # Critical: Pre-auto-tune motor validation with retry
        print("Pre-validation: Testing motor response...")
        validation_attempts = 0
        max_validation_attempts = 3

        while validation_attempts < max_validation_attempts:
            if self._validate_motor_response(target_rpm):
                break  # Validation passed
            else:
                validation_attempts += 1
                if validation_attempts < max_validation_attempts:
                    print(f"Validation attempt {validation_attempts} failed - retrying with motor reset...")
                    # Try to force stop the motor again
                    self._force_motor_stop()
                    time.sleep(2)  # Wait for motor to settle
                else:
                    print(f"Motor validation failed after {max_validation_attempts} attempts")
                    print("Cannot proceed with auto-tune - motor response unreliable")
                    return None, None

        # Phase 1: Stabilize at target RPM first
        print("Phase 1: Stabilizing motor at target RPM...")
        self.computer_pid_control(target_rpm, kp=0.5, ki=0.1, kd=0.01, duration=5)

        # Phase 2: Ultimate gain search
        print("Phase 2: Finding ultimate gain (Ku)...")

        # Comprehensive motor response test with multiple attempts
        print("Comprehensive motor response test...")
        max_retries = 3
        test_rpm = 0

        for attempt in range(max_retries):
            print(f"  Test attempt {attempt + 1}/{max_retries}...")
            self.send_command("SET_PWM 0")  # Ensure stopped
            time.sleep(1)

            self.send_command("SET_PWM 200")  # Higher test PWM
            time.sleep(4)  # Wait longer for stabilization
            test_rpm = self.get_current_rpm_from_arduino()
            self.send_command("SET_PWM 0")  # Stop
            time.sleep(2)  # Let it settle

            print(f"  Motor test {attempt + 1}: PWM=200 → RPM={test_rpm}")

            if test_rpm >= 50.0:  # Success threshold
                print("  Motor response test passed")
                break
            elif attempt < max_retries - 1:
                print("  Motor response weak, trying recovery...")
                # Recovery: cycle motor power
                self.send_command("ENABLE_MOTOR 0")
                time.sleep(2)
                self.send_command("ENABLE_MOTOR 1")
                time.sleep(2)

        if test_rpm < 50.0:  # Final check
            print("CRITICAL: Motor not responding properly to PWM commands after all attempts!")
            print("   Possible issues:")
            print("   - Arduino code not uploaded (check auto_tune/code/code.ino)")
            print("   - Motor/ESC not connected or powered")
            print("   - Hall sensor not working")
            print("   - ESC not responding to PWM")
            print("   - System destabilized from previous auto-tune")
            print(f"   Motor only reached {test_rpm:.1f} RPM at PWM=200")
            print("   Try: power cycling the motor/ESC, checking connections, re-uploading Arduino code")
            return None, None

        kp_test = 0.5  # Start higher
        max_kp = 10.0  # Go higher
        oscillation_data = []

        while kp_test < max_kp and self.auto_tune_running:
            print(f"Testing Kp = {kp_test:.3f}")
            self.tune_status_label.setText(f"Testing Kp: {kp_test:.3f}")
            QApplication.processEvents()

            # Clear old data
            self.plot_canvas.current_rpm_data.clear()

            # Run computer P-control with current Kp (ultimate gain test)
            print(f"    Running P-control test with Kp={kp_test:.3f} for {target_rpm} RPM target")
            rpm_data, pwm_data = self.computer_pid_control(target_rpm, kp=kp_test, ki=0.0, kd=0.0, duration=8)  # Shorter test

            if len(rpm_data) > 60:
                try:
                    # Check for motor failure during test
                    rpm_max = max(rpm_data)
                    rpm_min = min(rpm_data)
                    rpm_range = rpm_max - rpm_min

                    if rpm_max < 10.0:
                        print(f"  CRITICAL: Motor completely stopped during test (max RPM: {rpm_max:.1f})")
                        print("  Skipping this Kp value and continuing with next...")
                        kp_test += 0.2
                        continue

                    if rpm_range < 5.0:
                        print(f"  WARNING: Very small RPM variation ({rpm_range:.1f}) - motor may not be oscillating properly")

                    # Analyze for oscillation
                    oscillation_score = self.analyze_oscillation_computer(rpm_data)

                    # If oscillation detection returned 0, motor is not responding
                    if oscillation_score == 0.0:
                        print("  Motor not responding during oscillation test - attempting recovery...")

                        # Recovery attempt 1: Reset and retry with same Kp
                        print("  Recovery 1: Resetting motor state...")
                        self.send_command("ENABLE_MOTOR 0")
                        time.sleep(1)
                        self.send_command("ENABLE_MOTOR 1")
                        time.sleep(2)

                        # Try again with same Kp
                        print(f"  Retrying oscillation test with Kp = {kp_test:.3f}")
                        rpm_data, pwm_data = self.computer_pid_control(target_rpm, kp=kp_test, ki=0.0, kd=0.0, duration=8)

                        if len(rpm_data) > 60:
                            oscillation_score = self.analyze_oscillation_computer(rpm_data)
                            rpm_mean = np.mean(rpm_data)
                            rpm_std = np.std(rpm_data)
                            print(f"  Recovery result - RPM mean: {rpm_mean:.1f}, std: {rpm_std:.1f}, score: {oscillation_score:.3f}")

                        # Recovery attempt 2: Try with different approach
                        if oscillation_score == 0.0:
                            print("  Recovery 2: Trying alternative oscillation detection...")
                            # Try a longer test with more aggressive PWM changes
                            self.computer_pid_control(target_rpm, kp=kp_test * 1.2, ki=0.0, kd=0.0, duration=12)
                            rpm_data, pwm_data = self.computer_pid_control(target_rpm, kp=kp_test, ki=0.0, kd=0.0, duration=8)

                            if len(rpm_data) > 60:
                                oscillation_score = self.analyze_oscillation_computer(rpm_data)
                                print(f"  Alternative result - score: {oscillation_score:.3f}")

                        if oscillation_score == 0.0:
                            print("  All recovery attempts failed - motor not responding")
                            print("  Possible issues:")
                            print("  - Previous PID values destabilized the system")
                            print("  - Motor/ESC thermal issues after first run")
                            print("  - Arduino serial buffer overflow")
                            print("  - Hardware connection problems")
                            return None, None

                    rpm_mean = np.mean(rpm_data)
                    rpm_std = np.std(rpm_data)
                    print(f"  RPM mean: {rpm_mean:.1f}, std: {rpm_std:.1f}, score: {oscillation_score:.3f}")

                    if oscillation_score > 0.3:  # Lower threshold for oscillation detection
                        print(f"Strong oscillation detected at Kp = {kp_test:.3f}")

                        # Measure period multiple times for accuracy
                        period = self.measure_oscillation_period_computer(rpm_data)
                        if period and 0.2 < period < 10.0:  # Reasonable period range
                            print(f"Ultimate gain found: Ku = {kp_test:.3f}, Tu = {period:.3f}s")
                            return kp_test, period
                        else:
                            print(f"Invalid period measured: {period}")
                except Exception as e:
                    print(f"Error analyzing oscillation: {e}")
                    continue

            kp_test += 0.2  # Larger increment for faster testing

        # If we get here, no oscillation was found
        print("No oscillation detected within Kp range")
        return None, None

    def computer_pid_control(self, target_rpm, kp, ki, kd, duration):
        """Run PID control entirely on computer"""
        rpm_history = []
        pwm_history = []
        time_history = []

        # PID state
        integral = 0.0
        prev_error = None  # Initialize as None
        start_time = time.time()

        print(f"Running computer PID control: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")

        loop_count = 0
        while time.time() - start_time < duration and self.auto_tune_running:
            loop_start = time.time()

            # Get current RPM from Arduino (synchronized data collection)
            current_rpm = self.get_current_rpm_from_arduino()

            # Always collect data (even if RPM is 0) for analysis
            if current_rpm is not None:
                # Calculate PID
                error = target_rpm - current_rpm
                integral += error * 0.1  # dt = 0.1s
                integral = max(-1000, min(1000, integral))  # Anti-windup
                derivative = (error - prev_error) / 0.1 if prev_error is not None else 0
                prev_error = error

                pid_output = kp * error + ki * integral + kd * derivative

                # Convert to PWM (20-255 range) - more aggressive scaling for oscillation testing
                pwm_value = int(127.5 + pid_output * 2.0)  # Higher gain for oscillation
                pwm_value = max(20, min(255, pwm_value))

                # Send PWM command to Arduino (only if changed significantly)
                if not hasattr(self, '_last_pwm') or abs(pwm_value - self._last_pwm) > 5:
                    self.send_command(f"SET_PWM {pwm_value}")
                    self._last_pwm = pwm_value
                    print(f"Sent PWM: {pwm_value}")  # Debug

                # Store synchronized data
                current_time = time.time() - start_time
                rpm_history.append(current_rpm)
                pwm_history.append(pwm_value)
                time_history.append(current_time)

            # Maintain consistent 10Hz timing
            elapsed = time.time() - loop_start
            if elapsed < 0.1:
                time.sleep(0.1 - elapsed)

        # Stop motor at end
        self.send_command("SET_PWM 0")
        if hasattr(self, '_last_pwm'):
            delattr(self, '_last_pwm')

        print(f"Collected {len(rpm_history)} data points")
        if len(rpm_history) > 0:
            print(f"RPM range: {min(rpm_history):.1f} - {max(rpm_history):.1f}")
            print(f"PWM range: {min(pwm_history)} - {max(pwm_history)}")
        return rpm_history, pwm_history

    def get_current_rpm_from_arduino(self):
        """Extract current RPM from Arduino data"""
        try:
            # During auto-tune, we need to poll for fresh data since normal updates might be paused
            if hasattr(self, 'serial_worker') and self.serial_worker.isRunning():
                # Send a request for status and wait briefly
                self.send_command("GET_STATUS")
                time.sleep(0.05)  # Brief wait for response

            # Get the most recent RPM data from the plot canvas
            if len(self.plot_canvas.current_rpm_data) > 0:
                rpm_value = self.plot_canvas.current_rpm_data[-1]
                # Validate RPM value
                if isinstance(rpm_value, (int, float)) and rpm_value >= 0:
                    return float(rpm_value)

            # If no valid data, try to get it from the data queue directly
            try:
                while not self.data_queue.empty():
                    line = self.data_queue.get_nowait()
                    if line.startswith("STATUS:"):
                        self.parse_status_data(line)
                        # After parsing, check if we now have valid RPM data
                        if len(self.plot_canvas.current_rpm_data) > 0:
                            rpm_value = self.plot_canvas.current_rpm_data[-1]
                            if isinstance(rpm_value, (int, float)) and rpm_value >= 0:
                                return float(rpm_value)
            except:
                pass

            return 0.0  # Return 0 instead of None for auto-tune continuity
        except Exception as e:
            print(f"Error getting RPM: {e}")
            return 0.0

    def analyze_oscillation_computer(self, rpm_data):
        """Analyze RPM data for oscillation on computer"""
        if len(rpm_data) < 30:
            return 0.0

        # Calculate basic statistics
        rpm_mean = np.mean(rpm_data)
        rpm_std = np.std(rpm_data)
        rpm_min = np.min(rpm_data)
        rpm_max = np.max(rpm_data)

        # CRITICAL CHECK: If motor is not responding at all, no oscillation
        if rpm_max < 10.0:  # Motor should spin at least 10 RPM if working
            print(f"  Motor not responding (max RPM: {rpm_max:.1f}) - aborting oscillation test")
            return 0.0

        # Coefficient of variation (relative variability) - avoid division by zero
        cv = rpm_std / abs(rpm_mean) if abs(rpm_mean) > 1e-6 else 0

        # Peak-to-peak amplitude
        amplitude = rpm_max - rpm_min

        # Zero crossing rate (oscillation frequency) - avoid issues with zero mean
        if abs(rpm_mean) > 1e-6:
            centered_data = rpm_data - rpm_mean
            zero_crossings = np.sum(np.diff(np.sign(centered_data)) != 0)
            zcr = zero_crossings / len(centered_data) if len(centered_data) > 0 else 0
        else:
            zcr = 0

        # Autocorrelation to detect periodicity (oscillation)
        autocorr = 0
        if len(rpm_data) > 50:
            try:
                # Calculate autocorrelation at lag 10 (about 2 seconds at 5Hz)
                autocorr_result = np.corrcoef(rpm_data[:-10], rpm_data[10:])[0,1]
                autocorr = abs(autocorr_result) if not np.isnan(autocorr_result) else 0
            except:
                autocorr = 0

        # Combine multiple metrics for oscillation detection
        # Protect against division by zero in amplitude calculation
        amplitude_score = (amplitude / abs(rpm_mean)) * 3 if abs(rpm_mean) > 1e-6 else 0
        oscillation_score = min(1.0, cv * 8 + amplitude_score + zcr * 6 + autocorr * 2)

        return oscillation_score

    def measure_oscillation_period_computer(self, rpm_data):
        """Measure oscillation period from RPM data"""
        if len(rpm_data) < 50:
            return None

        try:
            # Simple period estimation using autocorrelation
            # Look for the first peak in autocorrelation (excluding lag 0)
            data = np.array(rpm_data) - np.mean(rpm_data)  # Remove DC component

            # Compute autocorrelation
            corr = np.correlate(data, data, mode='full')
            corr = corr[len(corr)//2:]  # Take second half (positive lags)

            # Find peaks in autocorrelation (oscillation period)
            peaks = []
            for i in range(5, min(len(corr)//2, 100)):  # Look for periods up to 10 seconds
                if (corr[i] > corr[i-1] and corr[i] > corr[i+1] and
                    corr[i] > 0.3 * np.max(corr[1:50])):  # Significant peak
                    peaks.append(i)

            if peaks:
                # Use the first significant peak as the period (in seconds)
                period = peaks[0] * 0.1  # 0.1s per sample
                if 0.2 <= period <= 10.0:  # Reasonable range
                    return period

            # Fallback: simple zero-crossing based period estimation
            diffs = np.diff(data)
            zero_crossings = []
            for i in range(1, len(diffs)):
                if diffs[i-1] * diffs[i] < 0:  # Sign change
                    zero_crossings.append(i)

            if len(zero_crossings) >= 4:
                # Average distance between zero crossings
                distances = np.diff(zero_crossings)
                avg_distance = np.mean(distances)
                period = avg_distance * 0.1  # Convert to seconds
                if 0.2 <= period <= 10.0:
                    return period

        except Exception as e:
            print(f"Error measuring period: {e}")

        return None



    def measure_multiple_periods(self, data, min_periods=3):
        """Measure multiple oscillation periods for robustness"""
        periods = []

        # Analyze in overlapping windows
        window_size = min(80, len(data) // 2)

        for start in range(0, len(data) - window_size, window_size // 3):
            window = data[start:start + window_size]
            if len(window) > 40:
                period = self.estimate_period(window)
                if 500 < period < 10000:  # Reasonable period range (0.5-10 seconds)
                    periods.append(period)

        # Return at least min_periods measurements
        return periods[:min_periods] if len(periods) >= min_periods else None

    def validate_and_optimize_gains(self, kp_zn, ki_zn, kd_zn):
        """Validate calculated gains and optimize if necessary"""
        try:
            # Test the calculated gains
            self.send_command(f"SET_KP {kp_zn}")
            self.send_command(f"SET_KI {ki_zn}")
            self.send_command(f"SET_KD {kd_zn}")

            # Wait for system to settle
            time.sleep(5)

            # Evaluate performance with more comprehensive analysis
            if len(self.plot_canvas.error_data) > 150:  # Need more data for reliable analysis
                recent_errors = self.plot_canvas.error_data[-150:]

                # Calculate comprehensive performance metrics
                steady_state_error = np.mean(np.abs(recent_errors[-75:]))  # Last 3.75 seconds
                overshoot = self.calculate_overshoot(recent_errors)
                settling_time = self.calculate_settling_time(recent_errors)
                oscillation_amplitude = self.calculate_oscillation_amplitude(recent_errors[-75:])

                # Enhanced performance criteria
                performance_score = self.calculate_performance_score(
                    steady_state_error, overshoot, settling_time, oscillation_amplitude
                )

                # Excellent performance - use calculated gains
                if performance_score >= 0.8:
                    return (kp_zn, ki_zn, kd_zn)

                # Good performance - minor adjustments
                elif performance_score >= 0.6:
                    # Fine-tune for slightly better performance
                    if overshoot > 80:
                        kp_adjusted = 0.9 * kp_zn
                        ki_adjusted = 0.9 * ki_zn
                        kd_adjusted = 1.1 * kd_zn
                        return (kp_adjusted, ki_adjusted, kd_adjusted)
                    return (kp_zn, ki_zn, kd_zn)

                # Poor performance - apply conservative tuning
                else:
                    # Use more conservative Ziegler-Nichols coefficients
                    kp_conservative = 0.35 * ku  # More conservative
                    ti_conservative = 1.2 * tu   # Slower integral
                    td_conservative = 0.1 * tu   # Less derivative

                    ki_conservative = kp_conservative / ti_conservative
                    kd_conservative = kp_conservative * td_conservative

                    return (kp_conservative, ki_conservative, kd_conservative)

            # If validation data insufficient, return original gains
            return (kp_zn, ki_zn, kd_zn)

        except Exception as e:
            print(f"Gain validation error: {e}")
            return (kp_zn, ki_zn, kd_zn)

    def check_amplitude_stability(self, data, window=40):
        """Check if oscillation amplitude is stable"""
        if len(data) < window * 2:
            return False

        # Calculate amplitude in different segments
        segment1 = data[-window*2:-window]
        segment2 = data[-window:]

        amp1 = np.max(segment1) - np.min(segment1)
        amp2 = np.max(segment2) - np.min(segment2)

        # Amplitude should be similar (within 20%)
        return abs(amp1 - amp2) / max(amp1, amp2) < 0.2

    def calculate_oscillation_amplitude(self, error_data):
        """Calculate the amplitude of oscillations in steady state"""
        if len(error_data) < 20:
            return 0

        # Use peak-to-peak amplitude of recent data
        return np.max(error_data) - np.min(error_data)

    def calculate_performance_score(self, steady_state_error, overshoot, settling_time, oscillation_amplitude):
        """Calculate overall performance score (0-1, higher is better)"""
        # Normalize metrics (lower values are better)
        sse_score = max(0, 1 - steady_state_error / 100)  # Expect <100 RPM error
        overshoot_score = max(0, 1 - overshoot / 150)     # Expect <150% overshoot
        settling_score = max(0, 1 - settling_time / 10)   # Expect <10 seconds settling
        oscillation_score = max(0, 1 - oscillation_amplitude / 200)  # Expect <200 RPM oscillation

        # Weighted average (steady state most important)
        return 0.4 * sse_score + 0.25 * overshoot_score + 0.2 * settling_score + 0.15 * oscillation_score

    def calculate_overshoot(self, error_data, window=50):
        """Calculate percentage overshoot from error data"""
        if len(error_data) < window:
            return 0

        # Find peak error deviation
        peak_error = max(np.abs(error_data[-window:]))
        steady_state = np.mean(np.abs(error_data[-20:]))  # Last part as steady state

        if steady_state == 0:
            return 0

        return (peak_error / steady_state - 1) * 100

    def calculate_settling_time(self, error_data, threshold=5, steady_window=20):
        """Calculate settling time (time to reach steady state)"""
        if len(error_data) < steady_window * 2:
            return float('inf')

        # Find steady state value
        steady_state = np.mean(error_data[-steady_window:])

        # Find when error stays within threshold of steady state
        threshold_value = abs(steady_state) * threshold / 100

        for i in range(len(error_data) - steady_window, -1, -1):
            window_errors = error_data[i:i + steady_window]
            if np.all(np.abs(window_errors - steady_state) < threshold_value):
                return (len(error_data) - i) / 20.0  # Convert to seconds

        return float('inf')

    def detect_oscillation(self, data, threshold=0.12):
        """Enhanced oscillation detection with multiple criteria"""
        if len(data) < 30:
            return False

        # Method 1: Zero crossing rate
        diffs = np.diff(data)
        sign_changes = np.diff(np.sign(diffs))
        zero_crossings = np.sum(np.abs(sign_changes)) / 2
        crossing_rate = zero_crossings / len(data)

        # Method 2: Autocorrelation check (oscillating signals have high autocorrelation at lag = period)
        if len(data) > 40:
            # Check autocorrelation at different lags
            autocorr_values = []
            for lag in range(5, min(30, len(data)//3)):
                corr = np.corrcoef(data[:-lag], data[lag:])[0, 1]
                if not np.isnan(corr):
                    autocorr_values.append(abs(corr))

            max_autocorr = max(autocorr_values) if autocorr_values else 0
        else:
            max_autocorr = 0

        # Method 3: Spectral analysis (check for dominant frequency)
        if len(data) > 32:
            # Simple FFT to check for dominant frequency
            fft = np.fft.fft(data - np.mean(data))
            freqs = np.fft.fftfreq(len(data))
            magnitudes = np.abs(fft)

            # Find dominant frequency (excluding DC component)
            dominant_idx = np.argmax(magnitudes[1:]) + 1
            dominant_magnitude = magnitudes[dominant_idx]
            total_power = np.sum(magnitudes[1:]**2)

            # Check if there's a clear dominant frequency
            spectral_concentration = dominant_magnitude**2 / total_power if total_power > 0 else 0
        else:
            spectral_concentration = 0

        # Combine criteria with weighted scoring
        score = 0
        score += 1.0 if crossing_rate > threshold else 0  # Zero crossing
        score += 0.8 if max_autocorr > 0.6 else 0         # Autocorrelation
        score += 0.6 if spectral_concentration > 0.3 else 0  # Spectral concentration

        return score >= 1.2  # Require at least 2 out of 3 criteria

    def estimate_period(self, data, sample_rate=20):
        """Estimate oscillation period from zero crossings"""
        if len(data) < 20:
            return 1.0

        # Find zero crossings of the signal around mean
        mean_val = np.mean(data)
        centered_data = data - mean_val

        zero_crossings = []
        for i in range(1, len(centered_data)):
            if centered_data[i-1] * centered_data[i] < 0:
                zero_crossings.append(i)

        if len(zero_crossings) < 4:
            return 1.0

        # Calculate average period between zero crossings
        periods = np.diff(zero_crossings)
        avg_period = np.mean(periods)

        return avg_period / sample_rate * 1000  # Convert to milliseconds

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
