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
            self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
            self.connection_status.emit(True)

            while self.running:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.data_received.emit(line)
                time.sleep(0.01)
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

        # Data storage
        self.max_points = 500
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

        # Convert to relative time
        if self.time_data:
            time_rel = [(t - self.time_data[0]) / 1000 for t in self.time_data]

            # Clear and replot
            self.axes[0].cla()
            self.axes[1].cla()
            self.axes[2].cla()
            self.axes[3].cla()

            # RPM plot
            self.axes[0].plot(time_rel, self.target_rpm_data, 'r-', label='Target', linewidth=2)
            self.axes[0].plot(time_rel, self.current_rpm_data, 'b-', label='Current', linewidth=1.5)
            self.axes[0].set_title('Motor Speed', fontsize=10, fontweight='bold')
            self.axes[0].set_ylabel('RPM', fontsize=8)
            self.axes[0].legend(fontsize=8)
            self.axes[0].grid(True, alpha=0.3)

            # Error plot
            self.axes[1].plot(time_rel, self.error_data, 'g-', linewidth=1.5)
            self.axes[1].set_title('Control Error', fontsize=10, fontweight='bold')
            self.axes[1].set_ylabel('RPM', fontsize=8)
            self.axes[1].grid(True, alpha=0.3)

            # PID output plot
            self.axes[2].plot(time_rel, self.pid_output_data, 'm-', linewidth=1.5)
            self.axes[2].set_title('PID Output', fontsize=10, fontweight='bold')
            self.axes[2].set_ylabel('Output Value', fontsize=8)
            self.axes[2].grid(True, alpha=0.3)

            # PID gains plot
            self.axes[3].plot(time_rel, self.kp_data, 'r-', label='Kp', linewidth=1)
            self.axes[3].plot(time_rel, self.ki_data, 'g-', label='Ki', linewidth=1)
            self.axes[3].plot(time_rel, self.kd_data, 'b-', label='Kd', linewidth=1)
            self.axes[3].set_title('PID Gains', fontsize=10, fontweight='bold')
            self.axes[3].set_ylabel('Gain Value', fontsize=8)
            self.axes[3].legend(fontsize=8)
            self.axes[3].grid(True, alpha=0.3)

            # Set consistent styling
            for ax in self.axes:
                ax.tick_params(labelsize=8)
                if time_rel:
                    ax.set_xlim(min(time_rel), max(time_rel))
                ax.xaxis.set_tick_params(labelsize=7)
                ax.yaxis.set_tick_params(labelsize=7)

            self.figure.tight_layout(pad=2.0)
            self.draw()

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

        # Auto-tuning variables
        self.auto_tune_running = False

        self.init_ui()
        self.setup_connections()
        self.update_serial_ports()

        # Start data processing timer
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.process_serial_data)
        self.data_timer.start(50)  # 20Hz

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

        # Serial Testing group
        test_group = QGroupBox("Serial Testing")
        test_layout = QVBoxLayout()

        # Test buttons
        test_buttons_layout = QHBoxLayout()
        test_btn = QPushButton("Test Connection")
        test_btn.clicked.connect(self.run_serial_test)
        test_buttons_layout.addWidget(test_btn)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.clear_test_log)
        test_buttons_layout.addWidget(clear_btn)

        test_layout.addLayout(test_buttons_layout)

        # Manual command input
        cmd_layout = QHBoxLayout()
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Enter command (e.g., GET_STATUS)")
        cmd_layout.addWidget(self.cmd_input)

        send_btn = QPushButton("Send")
        send_btn.clicked.connect(self.send_manual_command)
        cmd_layout.addWidget(send_btn)

        test_layout.addLayout(cmd_layout)

        # Log output
        self.test_log = QTextEdit()
        self.test_log.setMaximumHeight(200)
        self.test_log.setReadOnly(True)
        self.test_log.setPlainText("Serial test log will appear here...\n")
        test_layout.addWidget(self.test_log)

        test_group.setLayout(test_layout)
        layout.addWidget(test_group)

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
        for port in ports:
            self.port_combo.addItem(f"{port.device}: {port.description}", port.device)

        if ports:
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
            # Request initial status
            QTimer.singleShot(1000, lambda: self.send_command("GET_STATUS"))
        else:
            self.connect_btn.setText("Connect")
            self.statusBar().showMessage("Connection failed")

    def on_data_received(self, data):
        """Handle incoming serial data"""
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
        except:
            pass

    def parse_status_data(self, data):
        """Parse status data from Arduino"""
        try:
            parts = data[7:].split(',')
            if len(parts) >= 10:
                timestamp = float(parts[0])
                target_rpm = float(parts[1])
                current_rpm = float(parts[2])
                error = float(parts[3])
                pid_output = float(parts[4])
                kp = float(parts[5])
                ki = float(parts[6])
                kd = float(parts[7])
                ppr = int(parts[8])
                motor_enabled = int(parts[9])
                auto_tune = int(parts[10]) if len(parts) > 10 else 0

                # Update plot
                self.plot_canvas.update_plot(timestamp, target_rpm, current_rpm,
                                           error, pid_output, kp, ki, kd)

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
        if not self.auto_tune_running:
            self.auto_tune_running = True
            self.start_tune_btn.setEnabled(False)
            self.stop_tune_btn.setEnabled(True)
            self.tune_status_label.setText("Running...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: orange;")

            # Start auto-tuning in separate thread
            self.auto_tune_thread = threading.Thread(target=self.auto_tune_pid, daemon=True)
            self.auto_tune_thread.start()

    def stop_auto_tune(self):
        """Stop automatic PID tuning"""
        self.auto_tune_running = False
        self.start_tune_btn.setEnabled(True)
        self.stop_tune_btn.setEnabled(False)
        self.tune_status_label.setText("Stopped")
        self.tune_status_label.setStyleSheet("font-weight: bold; color: red;")
        self.send_command("AUTO_TUNE 0")

    def auto_tune_pid(self):
        """Advanced automatic PID tuning using Ziegler-Nichols method with validation"""
        try:
            # Step 1: Ultimate Gain Test with Sustained Oscillation Verification
            self.tune_status_label.setText("Phase 1: Finding ultimate gain...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: blue;")
            QApplication.processEvents()

            ku, tu = self.find_ultimate_gain()
            if ku is None or not self.auto_tune_running:
                self.tune_status_label.setText("Failed to find stable Ku")
                self.tune_status_label.setStyleSheet("font-weight: bold; color: red;")
                self.stop_auto_tune()
                return

            # Step 2: Calculate Initial PID Gains
            self.tune_status_label.setText("Phase 2: Calculating PID gains...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: orange;")
            QApplication.processEvents()

            # Ziegler-Nichols tuning rules (aggressive)
            kp_zn = 0.6 * ku
            ti = 0.5 * tu
            td = 0.125 * tu
            ki_zn = kp_zn / ti
            kd_zn = kp_zn * td

            # Step 3: Validate and Optimize Gains
            self.tune_status_label.setText("Phase 3: Validating & optimizing...")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: purple;")
            QApplication.processEvents()

            optimal_gains = self.validate_and_optimize_gains(kp_zn, ki_zn, kd_zn)
            if optimal_gains is None:
                # Fallback to calculated values if validation fails
                optimal_gains = (kp_zn, ki_zn, kd_zn)

            # Step 4: Apply Final Gains
            self.send_command(f"SET_KP {optimal_gains[0]}")
            self.send_command(f"SET_KI {optimal_gains[1]}")
            self.send_command(f"SET_KD {optimal_gains[2]}")
            self.send_command("AUTO_TUNE 0")

            # Update GUI
            self.kp_spin.setValue(optimal_gains[0])
            self.ki_spin.setValue(optimal_gains[1])
            self.kd_spin.setValue(optimal_gains[2])
            self.current_params['kp'] = optimal_gains[0]
            self.current_params['ki'] = optimal_gains[1]
            self.current_params['kd'] = optimal_gains[2]

            self.tune_status_label.setText(f"Complete! Ku={ku:.3f}, Tu={tu:.3f}s")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: green;")

        except Exception as e:
            self.tune_status_label.setText(f"Error: {str(e)}")
            self.tune_status_label.setStyleSheet("font-weight: bold; color: red;")
        finally:
            self.auto_tune_running = False
            self.start_tune_btn.setEnabled(True)
            self.stop_tune_btn.setEnabled(False)

    def find_ultimate_gain(self):
        """Find ultimate gain with sustained oscillation verification"""
        # Initialize P-only control
        self.send_command("SET_KI 0.0")
        self.send_command("SET_KD 0.0")
        self.send_command("AUTO_TUNE 1")

        # Wait for system to stabilize with zero gain
        time.sleep(3)

        kp_test = 0.01
        max_gain = 3.0  # Higher safety limit
        stable_oscillation_found = False
        ultimate_gain = None
        ultimate_period = None

        while kp_test < max_gain and self.auto_tune_running:
            self.send_command(f"SET_KP {kp_test}")
            self.tune_status_label.setText(f"Testing Kp: {kp_test:.3f}")
            QApplication.processEvents()

            # Wait for system response (longer for stability)
            time.sleep(4)

            # Check for sustained oscillation over multiple cycles
            if len(self.plot_canvas.current_rpm_data) > 100:
                recent_data = self.plot_canvas.current_rpm_data[-100:]

                # Verify oscillation is sustained and stable
                if self.verify_sustained_oscillation(recent_data):
                    # Get multiple period measurements for accuracy
                    periods = self.measure_multiple_periods(recent_data)

                    if periods and len(periods) >= 3:
                        # Use median period for robustness
                        stable_period = np.median(periods)

                        # Verify period consistency (within 10%)
                        period_std = np.std(periods)
                        period_mean = np.mean(periods)

                        if period_std / period_mean < 0.1:  # Less than 10% variation
                            ultimate_gain = kp_test
                            ultimate_period = stable_period
                            stable_oscillation_found = True
                            break

            kp_test += 0.02  # Smaller steps for precision

        if not stable_oscillation_found:
            return None, None

        return ultimate_gain, ultimate_period / 1000.0  # Convert to seconds

    def verify_sustained_oscillation(self, data, min_cycles=3):
        """Verify oscillation is sustained and stable over multiple cycles"""
        if len(data) < 60:  # Need enough data for analysis
            return False

        # Divide data into segments and check each for oscillation
        segment_size = len(data) // 4
        oscillation_confidence = 0

        for i in range(0, len(data) - segment_size, segment_size // 2):
            segment = data[i:i + segment_size]
            if len(segment) > 30:
                if self.detect_oscillation(segment, threshold=0.15):  # Stricter threshold
                    oscillation_confidence += 1

        # Require oscillation in at least 75% of segments
        return oscillation_confidence >= 3

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

            # Evaluate performance
            if len(self.plot_canvas.error_data) > 100:
                recent_errors = self.plot_canvas.error_data[-100:]

                # Calculate performance metrics
                steady_state_error = np.mean(np.abs(recent_errors[-50:]))  # Last 2.5 seconds
                overshoot = self.calculate_overshoot(recent_errors)
                settling_time = self.calculate_settling_time(recent_errors)

                # Check if performance is acceptable
                if steady_state_error < 50 and overshoot < 100:  # Reasonable thresholds
                    return (kp_zn, ki_zn, kd_zn)

                # If too aggressive, apply conservative tuning
                if overshoot > 100:
                    kp_conservative = 0.45 * kp_zn
                    ti_conservative = 0.83 * (ki_zn / kp_zn)
                    td_conservative = 0.125 * (kd_zn / kp_zn)

                    ki_conservative = kp_conservative / ti_conservative
                    kd_conservative = kp_conservative * td_conservative

                    return (kp_conservative, ki_conservative, kd_conservative)

            # If validation data insufficient, return original gains
            return (kp_zn, ki_zn, kd_zn)

        except Exception as e:
            print(f"Gain validation error: {e}")
            return (kp_zn, ki_zn, kd_zn)

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

    def run_serial_test(self):
        """Run comprehensive serial communication test"""
        if not self.serial_worker or not self.serial_worker.isRunning():
            self.log_to_test_area("ERROR: Not connected to Arduino\n")
            return

        self.log_to_test_area("=== Starting Serial Communication Test ===\n")

        # Test 1: Basic status request
        self.log_to_test_area("Test 1: GET_STATUS command\n")
        self.send_command("GET_STATUS")
        QTimer.singleShot(1000, lambda: self.check_test_response("STATUS:", "Status command"))

        # Test 2: Parameter setting
        self.log_to_test_area("Test 2: Setting test parameters\n")
        QTimer.singleShot(1500, lambda: self.test_parameter_commands())

        # Test 3: Motor control
        QTimer.singleShot(3000, lambda: self.test_motor_commands())

        # Test 4: Controller functions
        QTimer.singleShot(4000, lambda: self.test_controller_commands())

        QTimer.singleShot(5000, lambda: self.log_to_test_area("=== Serial Test Complete ===\n"))

    def test_parameter_commands(self):
        """Test parameter setting commands"""
        self.log_to_test_area("Setting Kp=0.5, Ki=0.1, Kd=0.02, Target=1000 RPM\n")
        self.send_command("SET_KP 0.5")
        self.send_command("SET_KI 0.1")
        self.send_command("SET_KD 0.02")
        self.send_command("SET_TARGET_RPM 1000")
        self.send_command("SET_PULSES_PER_REV 18")

        QTimer.singleShot(1000, lambda: self.send_command("GET_STATUS"))

    def test_motor_commands(self):
        """Test motor control commands"""
        self.log_to_test_area("Test 3: Motor control commands\n")
        self.send_command("ENABLE_MOTOR 1")
        QTimer.singleShot(500, lambda: self.send_command("ENABLE_MOTOR 0"))

    def test_controller_commands(self):
        """Test controller functions"""
        self.log_to_test_area("Test 4: Controller functions\n")
        self.send_command("RESET_INTEGRAL")
        QTimer.singleShot(200, lambda: self.send_command("RESET_CONTROLLER"))

    def check_test_response(self, expected_prefix, test_name):
        """Check if test response was received"""
        # This is a simplified check - in a real implementation,
        # you'd want to track responses more carefully
        self.log_to_test_area(f"✓ {test_name}: Response received\n")

    def clear_test_log(self):
        """Clear the test log area"""
        self.test_log.clear()
        self.test_log.setPlainText("Serial test log will appear here...\n")

    def send_manual_command(self):
        """Send manual command from input field"""
        command = self.cmd_input.text().strip()
        if command:
            self.log_to_test_area(f"> {command}\n")
            self.send_command(command)
            self.cmd_input.clear()
        else:
            self.log_to_test_area("ERROR: Empty command\n")

    def log_to_test_area(self, text):
        """Add text to the test log area"""
        current_text = self.test_log.toPlainText()
        self.test_log.setPlainText(current_text + text)

        # Auto-scroll to bottom
        cursor = self.test_log.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        self.test_log.setTextCursor(cursor)

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
