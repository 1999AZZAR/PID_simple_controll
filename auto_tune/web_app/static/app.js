// BLDC Motor PID Controller - Web Serial API Interface

class BLDCController {
    constructor() {
        this.port = null;
        this.reader = null;
        this.isConnected = false;
        this.charts = {};
        this.currentData = {
            connected: false,
            parameters: {
                target_rpm: 1440,
                kp: 0.25,
                ki: 0.015,
                kd: 0.003,
                pulses_per_rev: 18,
                motor_enabled: true
            },
            plot_data: {
                time: [],
                target_rpm: [],
                current_rpm: [],
                error: [],
                pid_output: [],
                kp: [],
                ki: [],
                kd: []
            }
        };

        // Connection monitoring
        this.lastDataTime = 0;
        this.connectionTimeout = 5000; // 5 seconds timeout
        this.connecting = false; // Flag to prevent multiple connection attempts

        this.init();
    }

    init() {
        this.checkWebSerialSupport();
        this.setupEventListeners();
        this.setupCharts();
        this.updateUI();

        // Start connection monitoring
        this.startConnectionMonitoring();
    }

    startConnectionMonitoring() {
        setInterval(() => {
            if (this.isConnected && this.lastDataTime > 0) {
                const timeSinceLastData = Date.now() - this.lastDataTime;
                if (timeSinceLastData > this.connectionTimeout) {
                    console.warn('No data received for', timeSinceLastData, 'ms');
                    this.updateConnectionStatus(false, 'No Data');
                    this.showAlert('No data received from Arduino. Check connection.', 'warning');
                }
            }
        }, 1000); // Check every second
    }

    showConnectionInfo() {
        const info = `
Web Serial API Connection Info:

• This interface connects directly to your Arduino
• No backend server required
• Requires Chrome, Edge, or Opera browser
• Works on localhost without HTTPS
• Requires user permission for serial access

IMPORTANT: Arduino may reset when serial connection opens!
   This is normal - wait 2-3 seconds for it to reboot.

Steps to connect:
1. Upload Arduino firmware from code/ directory
2. Connect Arduino to computer via USB
3. Click "Connect to Arduino" button
4. Grant permission when prompted
5. Select your Arduino serial port
6. Wait for Arduino to reboot (2-3 seconds)
7. Real-time control begins!

Troubleshooting:
• If "Device Lost" appears: Arduino reset - click "Reconnect"
• If "No Data": Click "Request Status" manually
• Check Arduino power and USB connection
• Verify firmware is uploaded correctly
• Try different USB port
• Check browser console for detailed logs

Console logs show:
• "Serial port connected" = USB connection established
• "STATUS data processed" = Arduino is sending data
• "Sent command to Arduino" = Commands being transmitted
        `;

        alert(info);
    }

    checkWebSerialSupport() {
        if (!('serial' in navigator)) {
            this.showAlert('Web Serial API not supported in this browser. Please use Chrome, Edge, or Opera.', 'danger');
            document.getElementById('connect-btn').disabled = true;
            document.getElementById('connect-btn').innerHTML = 'Not Supported';
        }
    }

    setupEventListeners() {
        // Serial connection
        document.getElementById('connect-btn').addEventListener('click', () => this.toggleConnection());
        document.getElementById('request-status-btn').addEventListener('click', () => this.requestStatus());
        document.getElementById('reconnect-btn').addEventListener('click', () => this.attemptReconnection());
        document.getElementById('refresh-ports-btn').addEventListener('click', () => this.showConnectionInfo());

        // PID parameters
        document.getElementById('target-rpm').addEventListener('change', (e) => this.updateTargetRPM(e.target.value));
        document.getElementById('kp-slider').addEventListener('input', (e) => this.updateKp(e.target.value));
        document.getElementById('ki-slider').addEventListener('input', (e) => this.updateKi(e.target.value));
        document.getElementById('kd-slider').addEventListener('input', (e) => this.updateKd(e.target.value));
        document.getElementById('ppr').addEventListener('change', (e) => this.updatePPR(e.target.value));

        // Motor control
        document.getElementById('motor-toggle-btn').addEventListener('click', () => this.toggleMotor());
        document.getElementById('reset-btn').addEventListener('click', () => this.resetController());

        // File operations
        document.getElementById('save-params-btn').addEventListener('click', () => this.saveParameters());
        document.getElementById('load-params-btn').addEventListener('click', () => this.loadParameters());
        document.getElementById('export-data-btn').addEventListener('click', () => this.exportData());

        // About modal
        document.getElementById('about-btn').addEventListener('click', () => this.showAbout());

        // File input
        document.getElementById('file-input').addEventListener('change', (e) => this.handleFileLoad(e));
    }

    setupCharts() {
        const chartConfig = {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
                duration: 0 // Disable animations for real-time updates
            },
            plugins: {
                legend: {
                    display: true,
                    position: 'top'
                }
            },
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: {
                        display: true,
                        text: 'Time (s)'
                    }
                }
            }
        };

        // Speed chart
        this.charts.speed = new Chart(document.getElementById('speed-chart'), {
            type: 'line',
            data: {
                datasets: [{
                    label: 'Target RPM',
                    data: [],
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.1
                }, {
                    label: 'Current RPM',
                    data: [],
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.1
                }]
            },
            options: {
                ...chartConfig,
                scales: {
                    ...chartConfig.scales,
                    y: {
                        title: {
                            display: true,
                            text: 'RPM'
                        }
                    }
                }
            }
        });

        // Error chart
        this.charts.error = new Chart(document.getElementById('error-chart'), {
            type: 'line',
            data: {
                datasets: [{
                    label: 'Error',
                    data: [],
                    borderColor: 'rgb(255, 159, 64)',
                    backgroundColor: 'rgba(255, 159, 64, 0.1)',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.1
                }]
            },
            options: {
                ...chartConfig,
                scales: {
                    ...chartConfig.scales,
                    y: {
                        title: {
                            display: true,
                            text: 'RPM'
                        }
                    }
                }
            }
        });

        // PID output chart
        this.charts.output = new Chart(document.getElementById('output-chart'), {
            type: 'line',
            data: {
                datasets: [{
                    label: 'PID Output',
                    data: [],
                    borderColor: 'rgb(153, 102, 255)',
                    backgroundColor: 'rgba(153, 102, 255, 0.1)',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.1
                }]
            },
            options: {
                ...chartConfig,
                scales: {
                    ...chartConfig.scales,
                    y: {
                        title: {
                            display: true,
                            text: 'Output Value'
                        }
                    }
                }
            }
        });

        // PID gains chart
        this.charts.gains = new Chart(document.getElementById('gains-chart'), {
            type: 'line',
            data: {
                datasets: [{
                    label: 'Kp',
                    data: [],
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    borderWidth: 1,
                    fill: false,
                    tension: 0.1
                }, {
                    label: 'Ki',
                    data: [],
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    borderWidth: 1,
                    fill: false,
                    tension: 0.1
                }, {
                    label: 'Kd',
                    data: [],
                    borderColor: 'rgb(255, 205, 86)',
                    backgroundColor: 'rgba(255, 205, 86, 0.1)',
                    borderWidth: 1,
                    fill: false,
                    tension: 0.1
                }]
            },
            options: {
                ...chartConfig,
                scales: {
                    ...chartConfig.scales,
                    y: {
                        title: {
                            display: true,
                            text: 'Gain Value'
                        }
                    }
                }
            }
        });
    }

    async connectSerial() {
        try {
            // Request a port
            this.port = await navigator.serial.requestPort();

            // Open the port
            await this.port.open({
                baudRate: 115200,
                dataBits: 8,
                stopBits: 1,
                parity: 'none'
            });

            console.log('Serial port connected');
            this.isConnected = true;
            this.lastDataTime = Date.now(); // Initialize data timer
            this.updateConnectionStatus(true);

            // Enable request status button
            document.getElementById('request-status-btn').disabled = false;

            // Start reading data
            this.startReading();

            // Arduino often resets when serial connection opens
            // Send multiple status requests with increasing delays
            console.log('Waiting for Arduino to reboot and be ready...');
            setTimeout(() => {
                console.log('Sending first status request...');
                this.sendCommand("GET_STATUS");
            }, 2000);
            setTimeout(() => {
                console.log('Sending second status request...');
                this.sendCommand("GET_STATUS");
            }, 4000);
            setTimeout(() => {
                console.log('Sending third status request...');
                this.sendCommand("GET_STATUS");
            }, 6000);

        } catch (error) {
            console.error('Failed to connect serial port:', error);
            this.updateConnectionStatus(false, 'Error');
            if (error.name === 'NotFoundError') {
                this.showAlert('No serial port selected. Please connect your Arduino and try again.', 'warning');
            } else if (error.name === 'NotAllowedError') {
                this.showAlert('Serial port access denied. Please grant permission and try again.', 'warning');
            } else {
                this.showAlert(`Serial connection failed: ${error.message}`, 'danger');
            }
        }
    }

    async disconnectSerial() {
        try {
            this.isConnected = false; // Set flag first

            if (this.reader) {
                try {
                    await this.reader.cancel();
                } catch (e) {
                    console.warn('Error cancelling reader:', e);
                }
                this.reader = null;
            }

            if (this.port) {
                try {
                    await this.port.close();
                } catch (e) {
                    console.warn('Error closing port:', e);
                }
                this.port = null;
            }

            this.updateConnectionStatus(false);

            // Disable request status button and hide reconnect
            document.getElementById('request-status-btn').disabled = true;
            document.getElementById('reconnect-btn').style.display = 'none';

            console.log('Serial port disconnected');
        } catch (error) {
            console.error('Error disconnecting:', error);
            this.updateConnectionStatus(false, 'Disconnect Error');
        }
    }

    async startReading() {
        try {
            const decoder = new TextDecoder();
            let buffer = '';

            // Create reader once for the connection
            this.reader = this.port.readable.getReader();

            try {
                while (true) {
                    const { value, done } = await this.reader.read();
                    if (done) break;

                    // Decode the received data
                    const chunk = decoder.decode(value, { stream: true });
                    buffer += chunk;

                    // Process complete lines (split by newline)
                    const lines = buffer.split('\n');
                    buffer = lines.pop(); // Keep incomplete line in buffer

                    // Process each complete line
                    for (const line of lines) {
                        const trimmedLine = line.trim();
                        if (trimmedLine) {
                            this.processSerialLine(trimmedLine);
                        }
                    }
                }
            } catch (error) {
                console.error('Reader error:', error);
                // Don't treat all reader errors as fatal - might be temporary
                if (!this.isConnected) {
                    throw error; // Re-throw if we're disconnecting
                }
            } finally {
                // Only release if we still have a reader
                if (this.reader) {
                    try {
                        this.reader.releaseLock();
                    } catch (e) {
                        console.warn('Error releasing reader:', e);
                    }
                    this.reader = null;
                }
            }
        } catch (error) {
            console.error('Error in serial reading:', error);

            // Handle device lost error (Arduino reset)
            if (error.name === 'NetworkError' && error.message.includes('device has been lost')) {
                console.log('Arduino reset detected - this is normal!');
                this.showAlert('Arduino reset detected (normal). Waiting for reboot...', 'info');
                this.updateConnectionStatus(false, 'Device Lost');

                // Wait for Arduino to reboot, then show reconnect option
                setTimeout(() => {
                    if (!this.isConnected) {
                        console.log('Arduino should be ready now - click Reconnect');
                        this.showAlert('Arduino should be rebooted. Click "Reconnect" to continue.', 'success');
                    }
                }, 3000);
            } else if (this.isConnected) {
                this.updateConnectionStatus(false, 'Read Error');
            }
        }
    }

    async attemptReconnection() {
        // Only attempt if not already connected and not already attempting
        if (this.isConnected || this.connecting) return;

        this.connecting = true;
        console.log('Attempting manual reconnection to Arduino...');

        try {
            await this.connectSerial();
        } catch (error) {
            console.error('Reconnection failed:', error);
            this.showAlert('Reconnection failed. Try connecting manually.', 'warning');
        } finally {
            this.connecting = false;
        }
    }

    parseStatusData(data) {
        try {
            if (!data.startsWith("STATUS:")) {
                return;
            }

            console.log('Processing STATUS data:', data);

            const parts = data.substring(7).split(',');
            if (parts.length !== 11) {
                console.warn('Invalid status data format, expected 11 parts, got', parts.length, ':', data);
                return;
            }

            // Parse data
            const timestamp = parseFloat(parts[0]);
            const target_rpm = parseFloat(parts[1]);
            const current_rpm = parseFloat(parts[2]);
            const error = parseFloat(parts[3]);
            const pid_output = parseFloat(parts[4]);
            const kp = parseFloat(parts[5]);
            const ki = parseFloat(parts[6]);
            const kd = parseFloat(parts[7]);
            const last_pwm_value = parseInt(parts[8]);
            const ppr = parseInt(parts[9]);
            const motor_enabled = parseInt(parts[10]);

            // Update last data time
            this.lastDataTime = Date.now();

            // Update current data
            this.currentData.parameters.target_rpm = target_rpm;
            this.currentData.parameters.kp = kp;
            this.currentData.parameters.ki = ki;
            this.currentData.parameters.kd = kd;
            this.currentData.parameters.pulses_per_rev = ppr;
            this.currentData.parameters.motor_enabled = motor_enabled;

            // Add to plot data
            this.currentData.plot_data.time.push(timestamp);
            this.currentData.plot_data.target_rpm.push(target_rpm);
            this.currentData.plot_data.current_rpm.push(current_rpm);
            this.currentData.plot_data.error.push(error);
            this.currentData.plot_data.pid_output.push(pid_output);
            this.currentData.plot_data.kp.push(kp);
            this.currentData.plot_data.ki.push(ki);
            this.currentData.plot_data.kd.push(kd);

            // Limit data points
            const maxPoints = 25;
            if (this.currentData.plot_data.time.length > maxPoints) {
                this.currentData.plot_data.time.shift();
                this.currentData.plot_data.target_rpm.shift();
                this.currentData.plot_data.current_rpm.shift();
                this.currentData.plot_data.error.shift();
                this.currentData.plot_data.pid_output.shift();
                this.currentData.plot_data.kp.shift();
                this.currentData.plot_data.ki.shift();
                this.currentData.plot_data.kd.shift();
            }

            // Update UI
            this.updateUI();
            this.updateCharts();

            console.log('STATUS data processed successfully');

        } catch (error) {
            console.error('Error parsing status data:', error);
        }
    }

    async sendCommand(command) {
        if (this.port && this.port.writable) {
            try {
                const encoder = new TextEncoder();
                const writer = this.port.writable.getWriter();
                const data = encoder.encode(command + '\n');
                await writer.write(data);
                writer.releaseLock();
                console.log('Sent command to Arduino:', command);
            } catch (error) {
                console.error('Error sending command:', error);
                this.showAlert('Failed to send command to Arduino', 'warning');
            }
        } else {
            console.warn('Serial port not connected - cannot send command:', command);
        }
    }

    processSerialLine(line) {
        try {
            // Debug: log all received lines (first few characters)
            console.log('Serial line received:', line.substring(0, 50) + (line.length > 50 ? '...' : ''));

            if (line.startsWith("STATUS:")) {
                this.parseStatusData(line);
            } else if (line.includes("ERROR") || line.includes("set to") || line.includes("ready") || line.includes("BLDC")) {
                console.log('Arduino message:', line);
                // Only show important messages as alerts
                if (line.includes("ERROR")) {
                    this.showAlert(`Arduino: ${line}`, 'warning');
                }
            } else {
                // Log other lines for debugging
                console.log('Other serial data:', line);
            }
        } catch (error) {
            console.error('Error processing serial line:', error);
        }
    }

    updateUI() {
        // Update parameter inputs
        const params = this.currentData.parameters;
        document.getElementById('target-rpm').value = params.target_rpm;
        document.getElementById('kp-slider').value = params.kp;
        document.getElementById('ki-slider').value = params.ki;
        document.getElementById('kd-slider').value = params.kd;
        document.getElementById('ppr').value = params.pulses_per_rev;

        // Update slider value displays
        document.getElementById('kp-value').textContent = params.kp.toFixed(3);
        document.getElementById('ki-value').textContent = params.ki.toFixed(4);
        document.getElementById('kd-value').textContent = params.kd.toFixed(4);

        // Update motor button and status
        const motorBtn = document.getElementById('motor-toggle-btn');
        if (params.motor_enabled) {
            motorBtn.className = 'btn btn-danger';
            motorBtn.textContent = 'Disable Motor';
        } else {
            motorBtn.className = 'btn btn-success';
            motorBtn.textContent = 'Enable Motor';
        }

        // Update motor status display
        this.updateMotorStatus();
    }

    updateCharts() {
        const plotData = this.currentData.plot_data;
        const timeData = plotData.time;

        // Convert timestamps to relative time
        const timeRel = timeData.length > 0 ?
            timeData.map(t => (t - timeData[0]) / 1000) : [];

        // Update speed chart
        this.charts.speed.data.datasets[0].data = timeRel.map((t, i) => ({x: t, y: plotData.target_rpm[i]}));
        this.charts.speed.data.datasets[1].data = timeRel.map((t, i) => ({x: t, y: plotData.current_rpm[i]}));
        this.charts.speed.update('none');

        // Update error chart
        this.charts.error.data.datasets[0].data = timeRel.map((t, i) => ({x: t, y: plotData.error[i]}));
        this.charts.error.update('none');

        // Update output chart
        this.charts.output.data.datasets[0].data = timeRel.map((t, i) => ({x: t, y: plotData.pid_output[i]}));
        this.charts.output.update('none');

        // Update gains chart
        this.charts.gains.data.datasets[0].data = timeRel.map((t, i) => ({x: t, y: plotData.kp[i]}));
        this.charts.gains.data.datasets[1].data = timeRel.map((t, i) => ({x: t, y: plotData.ki[i]}));
        this.charts.gains.data.datasets[2].data = timeRel.map((t, i) => ({x: t, y: plotData.kd[i]}));
        this.charts.gains.update('none');
    }

    updateConnectionStatus(connected, status = '') {
        const statusEl = document.getElementById('serial-status');
        const connectBtn = document.getElementById('connect-btn');

        if (connected) {
            statusEl.textContent = 'Connected to Arduino - Receiving Data';
            statusEl.style.color = 'var(--success)';
            connectBtn.className = 'btn btn-danger';
            connectBtn.textContent = 'Disconnect';
            document.getElementById('reconnect-btn').style.display = 'none';
            this.currentData.connected = true;
            console.log('Connection status: Connected');
        } else if (status === 'Error') {
            statusEl.textContent = 'Connection Error - Check USB connection';
            statusEl.style.color = 'var(--danger)';
            connectBtn.className = 'btn btn-primary';
            connectBtn.textContent = 'Connect to Arduino';
            this.currentData.connected = false;
            console.log('Connection status: Error');
        } else if (status === 'Read Error') {
            statusEl.textContent = 'Read Error - Arduino connected but not responding';
            statusEl.style.color = 'var(--warning)';
            this.currentData.connected = false;
            console.log('Connection status: Read Error');
        } else if (status === 'No Data') {
            statusEl.textContent = 'No Data - Connected but no STATUS messages';
            statusEl.style.color = 'var(--warning)';
            document.getElementById('reconnect-btn').style.display = 'block';
            console.log('Connection status: No Data');
        } else if (status === 'Device Lost') {
            statusEl.textContent = 'Device Lost - Arduino reset detected';
            statusEl.style.color = 'var(--danger)';
            document.getElementById('reconnect-btn').style.display = 'block';
            console.log('Connection status: Device Lost');
        } else {
            statusEl.textContent = 'Not Connected';
            statusEl.style.color = 'var(--gray-500)';
            connectBtn.className = 'btn btn-primary';
            connectBtn.textContent = 'Connect to Arduino';
            this.currentData.connected = false;
            console.log('Connection status: Disconnected');
        }
    }

    updateMotorStatus() {
        const motorStatusEl = document.getElementById('motor-status');
        const motorIndicator = document.getElementById('motor-indicator');
        const motorText = document.getElementById('motor-state-text');
        const rpmDisplay = document.getElementById('rpm-display');

        const isEnabled = this.currentData.parameters.motor_enabled;
        const currentRpm = this.currentData.plot_data.current_rpm[this.currentData.plot_data.current_rpm.length - 1] || 0;

        if (isEnabled) {
            motorStatusEl.textContent = 'Enabled';
            motorStatusEl.style.color = 'var(--success)';
            motorIndicator.className = 'motor-dot enabled';
            motorText.textContent = 'Motor Enabled';
        } else {
            motorStatusEl.textContent = 'Disabled';
            motorStatusEl.style.color = 'var(--gray-500)';
            motorIndicator.className = 'motor-dot disabled';
            motorText.textContent = 'Motor Disabled';
        }

        rpmDisplay.textContent = Math.round(currentRpm);
        rpmDisplay.style.color = isEnabled ? 'var(--success)' : 'var(--gray-500)';
    }

    async loadPorts() {
        // Web Serial API doesn't provide port listing
        // The user must manually select the port
        const portSelect = document.getElementById('port-select');
        portSelect.innerHTML = '<option value="">Click Connect to select port...</option>';
        this.showAlert('Web Serial: Click "Connect" to choose your Arduino port', 'info');
    }

    async toggleConnection() {
        if (this.isConnected) {
            // Disconnect
            await this.disconnectSerial();
            this.showAlert('Disconnected from Arduino', 'info');
        } else {
            // Connect - Web Serial API will show port selection dialog
            await this.connectSerial();
        }
    }

    async requestStatus() {
        console.log('Manually requesting status from Arduino');
        await this.sendCommand("GET_STATUS");
        this.showAlert('Status requested from Arduino', 'info');
    }

    async updateTargetRPM(value) {
        await this.sendCommand(`SET_TARGET_RPM ${parseFloat(value)}`);
    }

    async updateKp(value) {
        const kpValue = parseFloat(value);
        document.getElementById('kp-value').textContent = kpValue.toFixed(3);
        await this.sendCommand(`SET_KP ${kpValue}`);
    }

    async updateKi(value) {
        const kiValue = parseFloat(value);
        document.getElementById('ki-value').textContent = kiValue.toFixed(4);
        await this.sendCommand(`SET_KI ${kiValue}`);
    }

    async updateKd(value) {
        const kdValue = parseFloat(value);
        document.getElementById('kd-value').textContent = kdValue.toFixed(4);
        await this.sendCommand(`SET_KD ${kdValue}`);
    }

    async updatePPR(value) {
        await this.sendCommand(`SET_PULSES_PER_REV ${parseInt(value)}`);
    }

    async sendParameterUpdate(params) {
        try {
            await fetch('/api/parameters', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(params)
            });
        } catch (error) {
            console.error('Parameter update error:', error);
        }
    }

    async toggleMotor() {
        const enabled = !this.currentData.parameters.motor_enabled;
        await this.sendCommand(`ENABLE_MOTOR ${enabled ? 1 : 0}`);
    }

    async resetController() {
        await this.sendCommand("RESET_CONTROLLER");
        this.showAlert('Controller reset', 'info');
    }

    async saveParameters() {
        try {
            const response = await fetch('/api/save', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(this.currentData.parameters)
            });

            const result = await response.json();
            if (result.success) {
                this.showAlert(`Parameters saved as ${result.filename}`, 'success');
            } else {
                this.showAlert('Failed to save parameters', 'danger');
            }
        } catch (error) {
            console.error('Save error:', error);
            this.showAlert('Failed to save parameters', 'danger');
        }
    }

    loadParameters() {
        document.getElementById('file-input').click();
    }

    async handleFileLoad(event) {
        const file = event.target.files[0];
        if (!file) return;

        try {
            const text = await file.text();
            const params = JSON.parse(text);

            // Apply parameters
            await fetch('/api/parameters', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(params)
            });

            this.showAlert('Parameters loaded successfully', 'success');
        } catch (error) {
            console.error('Load error:', error);
            this.showAlert('Failed to load parameters', 'danger');
        }

        // Reset file input
        event.target.value = '';
    }

    async exportData() {
        try {
            const response = await fetch('/api/export');
            const data = await response.json();

            // Create and download CSV file
            const blob = new Blob([data.csv_data], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `pid_data_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.csv`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            window.URL.revokeObjectURL(url);

            this.showAlert('Data exported successfully', 'success');
        } catch (error) {
            console.error('Export error:', error);
            this.showAlert('Failed to export data', 'danger');
        }
    }

    showAbout() {
        const modal = new bootstrap.Modal(document.getElementById('about-modal'));
        modal.show();
    }

    showAlert(message, type = 'info') {
        // Create alert element
        const alertEl = document.createElement('div');
        alertEl.className = `alert alert-${type} alert-dismissible fade show position-fixed`;
        alertEl.style.cssText = 'top: 20px; right: 20px; z-index: 9999; min-width: 300px;';
        alertEl.innerHTML = `
            ${message}
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;

        document.body.appendChild(alertEl);

        // Auto remove after 5 seconds
        setTimeout(() => {
            if (alertEl.parentNode) {
                alertEl.remove();
            }
        }, 5000);
    }
}

// Initialize the application when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    new BLDCController();
});
