// BLDC Motor PID Controller - PC-Powered Auto-Tune Web Interface
// Uses PC's computational power for Kalman filtering, stability analysis, and auto-tuning

/**
 * Kalman Filter for RPM smoothing
 * Provides superior noise rejection using PC's processing power
 */
class KalmanFilter {
    constructor(processVariance = 1e-4, measurementVariance = 0.5) {
        this.Q = processVariance;  // Process noise
        this.R = measurementVariance;  // Measurement noise
        this.estimate = 0;
        this.errorEstimate = 1;
        this.initialized = false;
    }

    update(measurement) {
        if (!this.initialized) {
            this.estimate = measurement;
            this.initialized = true;
            return this.estimate;
        }

        // Prediction step
        const prediction = this.estimate;
        const predictionError = this.errorEstimate + this.Q;

        // Update step
        const kalmanGain = predictionError / (predictionError + this.R);
        this.estimate = prediction + kalmanGain * (measurement - prediction);
        this.errorEstimate = (1 - kalmanGain) * predictionError;

        return this.estimate;
    }

    reset() {
        this.estimate = 0;
        this.errorEstimate = 1;
        this.initialized = false;
    }
}

/**
 * Stability Analyzer for real-time motor control analysis
 */
class StabilityAnalyzer {
    constructor(windowSize = 100) {
        this.windowSize = windowSize;
        this.rpmHistory = [];
        this.errorHistory = [];
        this.timeHistory = [];
        this.targetRpm = 0;
    }

    addSample(rpm, target, timestamp) {
        this.rpmHistory.push(rpm);
        this.errorHistory.push(target - rpm);
        this.timeHistory.push(timestamp);
        this.targetRpm = target;

        // Limit history size
        if (this.rpmHistory.length > this.windowSize) {
            this.rpmHistory.shift();
            this.errorHistory.shift();
            this.timeHistory.shift();
        }
    }

    calculateMetrics() {
        const metrics = {
            currentRpm: 0,
            targetRpm: this.targetRpm,
            error: 0,
            rpmFiltered: 0,
            overshootPercent: 0,
            oscillationFrequencyHz: 0,
            stabilityScore: 100,
            isHunting: false,
            isStable: true
        };

        if (this.rpmHistory.length < 10) {
            return metrics;
        }

        const rpmArray = this.rpmHistory;
        const errorArray = this.errorHistory;

        metrics.currentRpm = rpmArray[rpmArray.length - 1];
        metrics.error = errorArray[errorArray.length - 1];

        // Calculate filtered RPM (last 5 samples average)
        const last5 = rpmArray.slice(-5);
        metrics.rpmFiltered = last5.reduce((a, b) => a + b, 0) / last5.length;

        // Calculate overshoot
        if (this.targetRpm > 0) {
            const maxRpm = Math.max(...rpmArray);
            if (maxRpm > this.targetRpm) {
                metrics.overshootPercent = ((maxRpm - this.targetRpm) / this.targetRpm) * 100;
            }
        }

        // Detect oscillation using zero-crossing analysis
        if (errorArray.length >= 20) {
            let zeroCrossings = 0;
            for (let i = 1; i < errorArray.length; i++) {
                if ((errorArray[i] > 0 && errorArray[i - 1] < 0) ||
                    (errorArray[i] < 0 && errorArray[i - 1] > 0)) {
                    zeroCrossings++;
                }
            }

            if (zeroCrossings >= 4 && this.timeHistory.length >= 2) {
                const duration = (this.timeHistory[this.timeHistory.length - 1] - this.timeHistory[0]) / 1000;
                if (duration > 0) {
                    metrics.oscillationFrequencyHz = zeroCrossings / (2 * duration);
                    metrics.isHunting = metrics.oscillationFrequencyHz > 0.5 && metrics.oscillationFrequencyHz < 10;
                }
            }
        }

        // Calculate stability score
        const recentErrors = errorArray.slice(-10);
        const meanError = Math.abs(recentErrors.reduce((a, b) => a + b, 0) / recentErrors.length);
        const recentRpm = rpmArray.slice(-20);
        const variance = this.calculateVariance(recentRpm);
        const stdDev = Math.sqrt(variance);

        const errorScore = Math.max(0, 100 - (meanError / (this.targetRpm + 1)) * 500);
        const varianceScore = Math.max(0, 100 - (stdDev / (this.targetRpm + 1)) * 200);
        const oscillationPenalty = metrics.isHunting ? 30 : 0;

        metrics.stabilityScore = Math.max(0, Math.min(100, (errorScore + varianceScore) / 2 - oscillationPenalty));
        metrics.isStable = metrics.stabilityScore > 70 && !metrics.isHunting;

        return metrics;
    }

    calculateVariance(arr) {
        const mean = arr.reduce((a, b) => a + b, 0) / arr.length;
        return arr.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / arr.length;
    }

    getSuggestions(metrics) {
        const suggestions = [];

        if (metrics.isHunting) {
            suggestions.push({
                type: 'danger',
                title: '⚠️ Hunting Detected',
                text: 'System is oscillating. Reduce Kp by 20-30% and/or increase Kd for more damping.'
            });
        }

        if (metrics.overshootPercent > 20) {
            suggestions.push({
                type: 'warning',
                title: '⚠️ High Overshoot',
                text: `Overshoot is ${metrics.overshootPercent.toFixed(1)}%. Reduce Kp by 15-20% or increase Kd.`
            });
        }

        if (metrics.stabilityScore < 70 && !metrics.isHunting) {
            suggestions.push({
                type: 'warning',
                title: '⚠️ Low Stability',
                text: 'Check mechanical issues, verify PPR setting, or try reducing Kp slightly.'
            });
        }

        if (Math.abs(metrics.error) > 50 && metrics.isStable) {
            suggestions.push({
                type: 'info',
                title: 'ℹ️ Steady-State Error',
                text: `Error of ${metrics.error.toFixed(1)} RPM. Consider increasing Ki slightly.`
            });
        }

        if (metrics.isStable && metrics.stabilityScore > 80 && suggestions.length === 0) {
            suggestions.push({
                type: 'success',
                title: '✓ System Well-Tuned',
                text: `Stability score: ${metrics.stabilityScore.toFixed(0)}%. Fine-tuning tips: increase Kp for faster response, increase Kd for less oscillation.`
            });
        }

        return suggestions;
    }

    clear() {
        this.rpmHistory = [];
        this.errorHistory = [];
        this.timeHistory = [];
    }
}

/**
 * Auto-Tuner using Ziegler-Nichols method
 */
class AutoTuner {
    static zieglerNicholsNoOvershoot(ku, tu) {
        return {
            kp: 0.2 * ku,
            ki: (2 * 0.2 * ku) / tu,
            kd: (0.2 * ku * tu) / 3,
            method: 'Ziegler-Nichols (No Overshoot)',
            notes: `Ku=${ku.toFixed(4)}, Tu=${tu.toFixed(4)}s`
        };
    }

    static zieglerNicholsClassic(ku, tu) {
        return {
            kp: 0.6 * ku,
            ki: (2 * 0.6 * ku) / tu,
            kd: (0.6 * ku * tu) / 8,
            method: 'Ziegler-Nichols (Classic)',
            notes: `Ku=${ku.toFixed(4)}, Tu=${tu.toFixed(4)}s`
        };
    }
}

/**
 * Main BLDC Controller Class
 */
class BLDCController {
    constructor() {
        this.port = null;
        this.reader = null;
        this.isConnected = false;
        this.charts = {};
        
        // PC-powered components
        this.kalmanFilter = new KalmanFilter();
        this.stabilityAnalyzer = new StabilityAnalyzer(100);
        
        // Current data and parameters
        this.currentData = {
            connected: false,
            parameters: {
                target_rpm: 1440,
                kp: 0.30,
                ki: 0.02,
                kd: 0.02,
                pulses_per_rev: 4,
                motor_enabled: false
            },
            plot_data: {
                time: [],
                target_rpm: [],
                current_rpm: [],
                filtered_rpm: [],
                error: [],
                pid_output: [],
                stability_score: []
            }
        };

        // Auto-tuning state
        this.autoTuning = {
            active: false,
            phase: 0,
            testKp: 0,
            ultimateGain: 0,
            ultimatePeriod: 0,
            timer: null
        };

        // Tuning history
        this.tuningHistory = [];

        // Connection monitoring
        this.lastDataTime = 0;
        this.connectionTimeout = 5000;

        this.init();
    }

    init() {
        this.checkWebSerialSupport();
        this.setupEventListeners();
        this.setupCharts();
        this.updateUI();
        this.startConnectionMonitoring();
    }

    checkWebSerialSupport() {
        if (!('serial' in navigator)) {
            this.showAlert('Web Serial API not supported. Please use Chrome, Edge, or Opera.', 'danger');
            document.getElementById('connect-btn').disabled = true;
            document.getElementById('connect-btn').innerHTML = '<i class="fas fa-times me-2"></i>Not Supported';
        }
    }

    setupEventListeners() {
        // Connection
        document.getElementById('connect-btn').addEventListener('click', () => this.toggleConnection());
        document.getElementById('request-status-btn').addEventListener('click', () => this.requestStatus());
        document.getElementById('reconnect-btn').addEventListener('click', () => this.attemptReconnection());

        // PID parameters
        document.getElementById('target-rpm').addEventListener('change', (e) => this.updateTargetRPM(e.target.value));
        document.getElementById('kp-slider').addEventListener('input', (e) => this.updateKp(e.target.value));
        document.getElementById('ki-slider').addEventListener('input', (e) => this.updateKi(e.target.value));
        document.getElementById('kd-slider').addEventListener('input', (e) => this.updateKd(e.target.value));
        document.getElementById('ppr').addEventListener('change', (e) => this.updatePPR(e.target.value));

        // Motor control
        document.getElementById('motor-toggle-btn').addEventListener('click', () => this.toggleMotor());
        document.getElementById('reset-btn').addEventListener('click', () => this.resetController());
        
        // Auto-tune
        document.getElementById('auto-tune-btn').addEventListener('click', () => this.startAutoTune());
        
        // Analysis
        document.getElementById('analyze-btn').addEventListener('click', () => this.analyzeSystem());

        // File operations
        document.getElementById('save-params-btn').addEventListener('click', () => this.saveParameters());
        document.getElementById('load-params-btn').addEventListener('click', () => this.loadParameters());
        document.getElementById('export-data-btn').addEventListener('click', () => this.exportData());
        document.getElementById('file-input').addEventListener('change', (e) => this.handleFileLoad(e));

        // About
        document.getElementById('about-btn').addEventListener('click', () => this.showAbout());
    }

    setupCharts() {
        const darkTheme = {
            backgroundColor: '#1e1e2e',
            gridColor: 'rgba(205, 214, 244, 0.1)',
            textColor: '#a6adc8',
            titleColor: '#cdd6f4'
        };

        const chartConfig = {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 0 },
            plugins: {
                legend: {
                    display: false
                }
            },
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: { display: true, text: 'Time (s)', color: darkTheme.textColor },
                    grid: { color: darkTheme.gridColor },
                    ticks: { color: darkTheme.textColor }
                },
                y: {
                    grid: { color: darkTheme.gridColor },
                    ticks: { color: darkTheme.textColor }
                }
            }
        };

        // Speed chart with three lines
        this.charts.speed = new Chart(document.getElementById('speed-chart'), {
            type: 'line',
            data: {
                datasets: [
                    {
                        label: 'Target RPM',
                        data: [],
                        borderColor: '#f5c2e7',
                        borderWidth: 2,
                        borderDash: [5, 5],
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0
                    },
                    {
                        label: 'Raw RPM',
                        data: [],
                        borderColor: '#74c7ec',
                        borderWidth: 1,
                        fill: false,
                        tension: 0.1,
                        pointRadius: 0,
                        backgroundColor: 'rgba(116, 199, 236, 0.2)'
                    },
                    {
                        label: 'Filtered RPM',
                        data: [],
                        borderColor: '#89b4fa',
                        borderWidth: 2,
                        fill: false,
                        tension: 0.3,
                        pointRadius: 0
                    }
                ]
            },
            options: {
                ...chartConfig,
                scales: {
                    ...chartConfig.scales,
                    y: { ...chartConfig.scales.y, title: { display: true, text: 'RPM', color: darkTheme.textColor } }
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
                    borderColor: '#f38ba8',
                    backgroundColor: 'rgba(243, 139, 168, 0.2)',
                    borderWidth: 1.5,
                    fill: true,
                    tension: 0.1,
                    pointRadius: 0
                }]
            },
            options: {
                ...chartConfig,
                scales: {
                    ...chartConfig.scales,
                    y: { ...chartConfig.scales.y, title: { display: true, text: 'Error (RPM)', color: darkTheme.textColor } }
                }
            }
        });

        // Stability chart
        this.charts.stability = new Chart(document.getElementById('stability-chart'), {
            type: 'line',
            data: {
                datasets: [{
                    label: 'Stability Score',
                    data: [],
                    borderColor: '#a6e3a1',
                    backgroundColor: 'rgba(166, 227, 161, 0.2)',
                    borderWidth: 2,
                    fill: true,
                    tension: 0.3,
                    pointRadius: 0
                }]
            },
            options: {
                ...chartConfig,
                scales: {
                    ...chartConfig.scales,
                    y: { 
                        ...chartConfig.scales.y, 
                        title: { display: true, text: 'Score (%)', color: darkTheme.textColor },
                        min: 0,
                        max: 100
                    }
                }
            }
        });
    }

    startConnectionMonitoring() {
        setInterval(() => {
            if (this.isConnected && this.lastDataTime > 0) {
                const timeSinceLastData = Date.now() - this.lastDataTime;
                if (timeSinceLastData > this.connectionTimeout) {
                    this.updateConnectionStatus(false, 'No Data');
                }
            }
        }, 1000);
    }

    async connectSerial() {
        try {
            this.port = await navigator.serial.requestPort();
            await this.port.open({
                baudRate: 115200,
                dataBits: 8,
                stopBits: 1,
                parity: 'none'
            });

            this.isConnected = true;
            this.lastDataTime = Date.now();
            this.kalmanFilter.reset();
            this.stabilityAnalyzer.clear();
            this.updateConnectionStatus(true);

            document.getElementById('request-status-btn').disabled = false;
            document.getElementById('auto-tune-btn').disabled = false;

            this.startReading();

            // Request status after Arduino reboots
            setTimeout(() => this.sendCommand("GET_STATUS"), 2000);
            setTimeout(() => this.sendCommand("GET_STATUS"), 4000);

        } catch (error) {
            console.error('Connection failed:', error);
            this.updateConnectionStatus(false, 'Error');
            this.showAlert(`Connection failed: ${error.message}`, 'danger');
        }
    }

    async disconnectSerial() {
        this.isConnected = false;
        
        if (this.reader) {
            try { await this.reader.cancel(); } catch (e) {}
            this.reader = null;
        }
        
        if (this.port) {
            try { await this.port.close(); } catch (e) {}
            this.port = null;
        }
        
        this.updateConnectionStatus(false);
        document.getElementById('request-status-btn').disabled = true;
        document.getElementById('auto-tune-btn').disabled = true;
    }

    async startReading() {
        try {
            const decoder = new TextDecoder();
            let buffer = '';
            this.reader = this.port.readable.getReader();

            while (true) {
                const { value, done } = await this.reader.read();
                if (done) break;

                buffer += decoder.decode(value, { stream: true });

                const lines = buffer.split('\n');
                buffer = lines.pop();

                for (const line of lines) {
                    const trimmed = line.trim();
                    if (trimmed) this.processSerialLine(trimmed);
                }
            }
        } catch (error) {
            console.error('Read error:', error);
            if (this.isConnected) {
                this.updateConnectionStatus(false, 'Read Error');
                document.getElementById('reconnect-btn').style.display = 'block';
            }
        } finally {
            if (this.reader) {
                try { this.reader.releaseLock(); } catch (e) {}
                this.reader = null;
            }
        }
    }

    processSerialLine(line) {
        if (line.startsWith("STATUS:")) {
            this.parseStatusData(line);
        } else if (line.includes("ERROR")) {
            this.showAlert(`Arduino: ${line}`, 'warning');
        }
    }

    parseStatusData(data) {
        try {
            const parts = data.substring(7).split(',');
            if (parts.length !== 11) return;

            const timestamp = parseFloat(parts[0]);
            const target_rpm = parseFloat(parts[1]);
            const current_rpm = parseFloat(parts[2]);
            const error = parseFloat(parts[3]);
            const pid_output = parseFloat(parts[4]);
            const kp = parseFloat(parts[5]);
            const ki = parseFloat(parts[6]);
            const kd = parseFloat(parts[7]);
            const ppr = parseInt(parts[9]);
            const motor_enabled = parseInt(parts[10]);

            this.lastDataTime = Date.now();

            // Apply Kalman filter
            const filtered_rpm = this.kalmanFilter.update(current_rpm);

            // Update stability analyzer
            this.stabilityAnalyzer.addSample(filtered_rpm, target_rpm, timestamp);
            const metrics = this.stabilityAnalyzer.calculateMetrics();

            // Update current data
            this.currentData.parameters = {
                target_rpm, kp, ki, kd,
                pulses_per_rev: ppr,
                motor_enabled: motor_enabled === 1
            };

            // Add to plot data
            const plotData = this.currentData.plot_data;
            plotData.time.push(timestamp);
            plotData.target_rpm.push(target_rpm);
            plotData.current_rpm.push(current_rpm);
            plotData.filtered_rpm.push(filtered_rpm);
            plotData.error.push(error);
            plotData.pid_output.push(pid_output);
            plotData.stability_score.push(metrics.stabilityScore);

            // Limit data points
            const maxPoints = 200;
            if (plotData.time.length > maxPoints) {
                Object.keys(plotData).forEach(key => plotData[key].shift());
            }

            // Update displays
            this.updateMetricsDisplay(metrics, current_rpm, filtered_rpm, error);
            this.updateUI();
            this.updateCharts();

            // Auto-tuning data collection
            if (this.autoTuning.active) {
                this.processAutoTuneData(metrics);
            }

        } catch (error) {
            console.error('Parse error:', error);
        }
    }

    updateMetricsDisplay(metrics, rawRpm, filteredRpm, error) {
        document.getElementById('rpm-display').textContent = rawRpm.toFixed(0);
        document.getElementById('filtered-rpm-display').textContent = filteredRpm.toFixed(0);
        document.getElementById('error-display').textContent = error.toFixed(1);
        document.getElementById('stability-display').textContent = metrics.stabilityScore.toFixed(0) + '%';

        const huntingEl = document.getElementById('hunting-status');
        if (metrics.isHunting) {
            huntingEl.textContent = 'HUNTING';
            huntingEl.className = 'status-value status-badge hunting';
        } else if (metrics.isStable) {
            huntingEl.textContent = 'STABLE';
            huntingEl.className = 'status-value status-badge stable';
        } else {
            huntingEl.textContent = 'SETTLING';
            huntingEl.className = 'status-value status-badge settling';
        }

        // Color code error
        const errorEl = document.getElementById('error-display');
        if (Math.abs(error) < 20) {
            errorEl.style.color = 'var(--success)';
        } else if (Math.abs(error) < 100) {
            errorEl.style.color = 'var(--warning)';
        } else {
            errorEl.style.color = 'var(--danger)';
        }
    }

    updateCharts() {
        const plotData = this.currentData.plot_data;
        const timeData = plotData.time;

        if (timeData.length < 2) return;

        const timeRel = timeData.map(t => (t - timeData[0]) / 1000);

        // Speed chart
        this.charts.speed.data.datasets[0].data = timeRel.map((t, i) => ({ x: t, y: plotData.target_rpm[i] }));
        this.charts.speed.data.datasets[1].data = timeRel.map((t, i) => ({ x: t, y: plotData.current_rpm[i] }));
        this.charts.speed.data.datasets[2].data = timeRel.map((t, i) => ({ x: t, y: plotData.filtered_rpm[i] }));
        this.charts.speed.update('none');

        // Error chart
        this.charts.error.data.datasets[0].data = timeRel.map((t, i) => ({ x: t, y: plotData.error[i] }));
        this.charts.error.update('none');

        // Stability chart
        this.charts.stability.data.datasets[0].data = timeRel.map((t, i) => ({ x: t, y: plotData.stability_score[i] }));
        this.charts.stability.update('none');
    }

    updateUI() {
        const params = this.currentData.parameters;
        
        document.getElementById('target-rpm').value = params.target_rpm;
        document.getElementById('kp-slider').value = params.kp;
        document.getElementById('ki-slider').value = params.ki;
        document.getElementById('kd-slider').value = params.kd;
        document.getElementById('ppr').value = params.pulses_per_rev;

        document.getElementById('kp-value').textContent = params.kp.toFixed(3);
        document.getElementById('ki-value').textContent = params.ki.toFixed(4);
        document.getElementById('kd-value').textContent = params.kd.toFixed(4);

        // Motor button
        const motorBtn = document.getElementById('motor-toggle-btn');
        if (params.motor_enabled) {
            motorBtn.className = 'btn btn-danger btn-lg w-100 mb-2';
            motorBtn.innerHTML = '<i class="fas fa-stop me-2"></i>Disable Motor';
            document.getElementById('motor-status').textContent = 'Enabled';
            document.getElementById('motor-status').style.color = 'var(--success)';
        } else {
            motorBtn.className = 'btn btn-success btn-lg w-100 mb-2';
            motorBtn.innerHTML = '<i class="fas fa-play me-2"></i>Enable Motor';
            document.getElementById('motor-status').textContent = 'Disabled';
            document.getElementById('motor-status').style.color = 'var(--text-muted)';
        }
    }

    updateConnectionStatus(connected, status = '') {
        const statusEl = document.getElementById('serial-status');
        const connectBtn = document.getElementById('connect-btn');

        if (connected) {
            statusEl.textContent = 'Connected';
            statusEl.style.color = 'var(--success)';
            connectBtn.className = 'btn btn-danger btn-lg w-100 mb-2';
            connectBtn.innerHTML = '<i class="fas fa-unlink me-2"></i>Disconnect';
            document.getElementById('reconnect-btn').style.display = 'none';
        } else {
            statusEl.textContent = status || 'Not Connected';
            statusEl.style.color = status === 'Error' ? 'var(--danger)' : 'var(--text-muted)';
            connectBtn.className = 'btn btn-primary btn-lg w-100 mb-2';
            connectBtn.innerHTML = '<i class="fas fa-link me-2"></i>Connect to Arduino';
        }
    }

    async sendCommand(command) {
        if (this.port && this.port.writable) {
            try {
                const writer = this.port.writable.getWriter();
                await writer.write(new TextEncoder().encode(command + '\n'));
                writer.releaseLock();
                console.log('Sent:', command);
            } catch (error) {
                console.error('Send error:', error);
            }
        }
    }

    // ========== Auto-Tune Functions ==========
    
    startAutoTune() {
        if (!this.isConnected) {
            this.showAlert('Please connect to Arduino first.', 'warning');
            return;
        }

        if (!confirm('This will automatically tune PID parameters.\n\nThe process will:\n1. Enable the motor\n2. Test increasing Kp values\n3. Detect oscillation point\n4. Calculate optimal PID\n\nContinue?')) {
            return;
        }

        this.autoTuning.active = true;
        this.autoTuning.phase = 1;
        this.autoTuning.testKp = 0.2;
        
        // Show progress
        document.getElementById('tune-progress-container').style.display = 'block';
        document.getElementById('auto-tune-btn').disabled = true;
        this.updateTuneProgress(0, 'Phase 1: Starting motor...');

        // Reset controller and set conservative gains
        this.sendCommand('RESET_CONTROLLER');
        this.sendCommand('SET_KP 0.2');
        this.sendCommand('SET_KI 0');
        this.sendCommand('SET_KD 0');

        // Enable motor
        this.sendCommand('ENABLE_MOTOR 1');
        this.currentData.parameters.motor_enabled = true;
        this.updateUI();

        // Start phase 2 after delay
        setTimeout(() => this.autoTunePhase2(), 3000);
    }

    autoTunePhase2() {
        if (!this.autoTuning.active) return;

        this.autoTuning.phase = 2;
        this.updateTuneProgress(30, 'Phase 2: Finding oscillation point...');

        // Start testing Kp values
        this.autoTuning.timer = setInterval(() => this.testNextKp(), 2000);
    }

    testNextKp() {
        if (!this.autoTuning.active) {
            clearInterval(this.autoTuning.timer);
            return;
        }

        const metrics = this.stabilityAnalyzer.calculateMetrics();

        // Check for sustained oscillation
        if (metrics.isHunting && metrics.oscillationFrequencyHz > 0.5) {
            clearInterval(this.autoTuning.timer);
            this.autoTuning.ultimateGain = this.autoTuning.testKp;
            this.autoTuning.ultimatePeriod = 1.0 / metrics.oscillationFrequencyHz;
            this.autoTunePhase3();
            return;
        }

        // Increase Kp
        this.autoTuning.testKp += 0.1;
        
        if (this.autoTuning.testKp > 2.0) {
            // Max reached without oscillation - use estimates
            clearInterval(this.autoTuning.timer);
            this.autoTuning.ultimateGain = 1.5;
            this.autoTuning.ultimatePeriod = 0.5;
            this.autoTunePhase3();
            return;
        }

        this.sendCommand(`SET_KP ${this.autoTuning.testKp.toFixed(3)}`);
        this.updateTuneProgress(30 + this.autoTuning.testKp * 20, `Testing Kp = ${this.autoTuning.testKp.toFixed(2)}...`);
    }

    autoTunePhase3() {
        this.autoTuning.phase = 3;
        this.updateTuneProgress(80, 'Phase 3: Calculating optimal parameters...');

        // Stop motor briefly
        this.sendCommand('ENABLE_MOTOR 0');

        // Calculate tuned parameters
        const result = AutoTuner.zieglerNicholsNoOvershoot(
            this.autoTuning.ultimateGain,
            this.autoTuning.ultimatePeriod
        );

        // Clamp to reasonable values
        result.kp = Math.max(0.01, Math.min(2.0, result.kp));
        result.ki = Math.max(0.001, Math.min(0.2, result.ki));
        result.kd = Math.max(0.001, Math.min(0.1, result.kd));

        // Apply parameters
        setTimeout(() => {
            this.sendCommand(`SET_KP ${result.kp.toFixed(4)}`);
            this.sendCommand(`SET_KI ${result.ki.toFixed(4)}`);
            this.sendCommand(`SET_KD ${result.kd.toFixed(4)}`);
            
            this.currentData.parameters.kp = result.kp;
            this.currentData.parameters.ki = result.ki;
            this.currentData.parameters.kd = result.kd;
            this.updateUI();

            // Re-enable motor
            this.sendCommand('ENABLE_MOTOR 1');

            // Add to history
            this.addToHistory(result);

            // Finish
            this.updateTuneProgress(100, 'Complete!');
            this.finishAutoTune(result);
        }, 1000);
    }

    finishAutoTune(result) {
        this.autoTuning.active = false;
        
        setTimeout(() => {
            document.getElementById('tune-progress-container').style.display = 'none';
            document.getElementById('auto-tune-btn').disabled = false;
            
            this.showAlert(
                `Auto-tune complete!\n\nKp = ${result.kp.toFixed(4)}\nKi = ${result.ki.toFixed(4)}\nKd = ${result.kd.toFixed(4)}`,
                'success'
            );

            // Update analysis panel
            this.analyzeSystem();
        }, 1000);
    }

    updateTuneProgress(percent, status) {
        document.getElementById('tune-progress').style.width = percent + '%';
        document.getElementById('tune-status').textContent = status;
    }

    addToHistory(result) {
        this.tuningHistory.unshift({
            ...result,
            timestamp: new Date().toLocaleTimeString()
        });

        // Update history display
        const historyEl = document.getElementById('tuning-history');
        historyEl.innerHTML = this.tuningHistory.slice(0, 5).map((item, idx) => `
            <div class="history-item" onclick="controller.applyHistoryItem(${idx})">
                <div class="history-method">${item.method}</div>
                <div class="history-params">
                    Kp: ${item.kp.toFixed(3)} | Ki: ${item.ki.toFixed(4)} | Kd: ${item.kd.toFixed(4)}
                </div>
                <div class="history-time">${item.timestamp}</div>
            </div>
        `).join('');
    }

    applyHistoryItem(index) {
        const item = this.tuningHistory[index];
        if (item) {
            this.sendCommand(`SET_KP ${item.kp.toFixed(4)}`);
            this.sendCommand(`SET_KI ${item.ki.toFixed(4)}`);
            this.sendCommand(`SET_KD ${item.kd.toFixed(4)}`);
            this.showAlert(`Applied ${item.method} parameters`, 'success');
        }
    }

    // ========== Analysis Functions ==========

    analyzeSystem() {
        const metrics = this.stabilityAnalyzer.calculateMetrics();
        const suggestions = this.stabilityAnalyzer.getSuggestions(metrics);

        let html = `
            <div class="analysis-section">
                <h5><i class="fas fa-chart-bar me-2"></i>Current Metrics</h5>
                <div class="metrics-grid">
                    <div class="metric-item">
                        <span class="metric-label">Current RPM</span>
                        <span class="metric-value">${metrics.currentRpm.toFixed(1)}</span>
                    </div>
                    <div class="metric-item">
                        <span class="metric-label">Target RPM</span>
                        <span class="metric-value">${metrics.targetRpm.toFixed(0)}</span>
                    </div>
                    <div class="metric-item">
                        <span class="metric-label">Error</span>
                        <span class="metric-value">${metrics.error.toFixed(1)}</span>
                    </div>
                    <div class="metric-item">
                        <span class="metric-label">Overshoot</span>
                        <span class="metric-value">${metrics.overshootPercent.toFixed(1)}%</span>
                    </div>
                    <div class="metric-item">
                        <span class="metric-label">Stability</span>
                        <span class="metric-value">${metrics.stabilityScore.toFixed(0)}%</span>
                    </div>
                    <div class="metric-item">
                        <span class="metric-label">Oscillation</span>
                        <span class="metric-value">${metrics.oscillationFrequencyHz.toFixed(2)} Hz</span>
                    </div>
                </div>
            </div>
            <div class="analysis-section">
                <h5><i class="fas fa-lightbulb me-2"></i>Suggestions</h5>
        `;

        if (suggestions.length === 0) {
            html += '<p class="text-muted">Enable motor and wait for data to generate suggestions.</p>';
        } else {
            suggestions.forEach(s => {
                html += `
                    <div class="suggestion-item suggestion-${s.type}">
                        <div class="suggestion-title">${s.title}</div>
                        <div class="suggestion-text">${s.text}</div>
                    </div>
                `;
            });
        }

        html += '</div>';
        document.getElementById('analysis-output').innerHTML = html;
    }

    processAutoTuneData(metrics) {
        // Called during auto-tune to monitor progress
        // Could add additional logic here
    }

    // ========== Control Functions ==========

    async toggleConnection() {
        if (this.isConnected) {
            await this.disconnectSerial();
        } else {
            await this.connectSerial();
        }
    }

    async toggleMotor() {
        const enabled = !this.currentData.parameters.motor_enabled;
        await this.sendCommand(`ENABLE_MOTOR ${enabled ? 1 : 0}`);
    }

    async resetController() {
        await this.sendCommand('RESET_CONTROLLER');
        this.kalmanFilter.reset();
        this.stabilityAnalyzer.clear();
        this.showAlert('Controller reset', 'info');
    }

    async requestStatus() {
        await this.sendCommand('GET_STATUS');
    }

    async attemptReconnection() {
        if (!this.isConnected) {
            await this.connectSerial();
        }
    }

    async updateTargetRPM(value) {
        await this.sendCommand(`SET_TARGET_RPM ${parseFloat(value)}`);
    }

    async updateKp(value) {
        document.getElementById('kp-value').textContent = parseFloat(value).toFixed(3);
        await this.sendCommand(`SET_KP ${parseFloat(value)}`);
    }

    async updateKi(value) {
        document.getElementById('ki-value').textContent = parseFloat(value).toFixed(4);
        await this.sendCommand(`SET_KI ${parseFloat(value)}`);
    }

    async updateKd(value) {
        document.getElementById('kd-value').textContent = parseFloat(value).toFixed(4);
        await this.sendCommand(`SET_KD ${parseFloat(value)}`);
    }

    async updatePPR(value) {
        await this.sendCommand(`SET_PULSES_PER_REV ${parseInt(value)}`);
    }

    // ========== File Operations ==========

    saveParameters() {
        const data = {
            params: this.currentData.parameters,
            history: this.tuningHistory,
            timestamp: new Date().toISOString()
        };

        localStorage.setItem('bldc_pid_params_v2', JSON.stringify(data));

        const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `pid_params_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.json`;
        a.click();
        URL.revokeObjectURL(url);

        this.showAlert('Parameters saved', 'success');
    }

    loadParameters() {
        const saved = localStorage.getItem('bldc_pid_params_v2');
        if (saved) {
            try {
                const data = JSON.parse(saved);
                const params = data.params;
                
                this.sendCommand(`SET_TARGET_RPM ${params.target_rpm}`);
                this.sendCommand(`SET_KP ${params.kp}`);
                this.sendCommand(`SET_KI ${params.ki}`);
                this.sendCommand(`SET_KD ${params.kd}`);
                this.sendCommand(`SET_PULSES_PER_REV ${params.pulses_per_rev}`);

                if (data.history) {
                    this.tuningHistory = data.history;
                    this.addToHistory(this.tuningHistory[0] || { method: 'Loaded', kp: params.kp, ki: params.ki, kd: params.kd });
                }

                this.showAlert('Parameters loaded', 'success');
                return;
            } catch (e) {}
        }
        document.getElementById('file-input').click();
    }

    async handleFileLoad(event) {
        const file = event.target.files[0];
        if (!file) return;

        try {
            const text = await file.text();
            const data = JSON.parse(text);
            const params = data.params || data;

            this.sendCommand(`SET_TARGET_RPM ${params.target_rpm || 1440}`);
            this.sendCommand(`SET_KP ${params.kp || 0.3}`);
            this.sendCommand(`SET_KI ${params.ki || 0.02}`);
            this.sendCommand(`SET_KD ${params.kd || 0.02}`);
            this.sendCommand(`SET_PULSES_PER_REV ${params.pulses_per_rev || 4}`);

            this.showAlert('Parameters loaded from file', 'success');
        } catch (error) {
            this.showAlert('Failed to load file', 'danger');
        }
        event.target.value = '';
    }

    exportData() {
        const plotData = this.currentData.plot_data;
        if (plotData.time.length === 0) {
            this.showAlert('No data to export', 'warning');
            return;
        }

        let csv = 'Time_ms,Target_RPM,Raw_RPM,Filtered_RPM,Error,Stability_Score\n';
        for (let i = 0; i < plotData.time.length; i++) {
            csv += `${plotData.time[i]},${plotData.target_rpm[i]},${plotData.current_rpm[i]},${plotData.filtered_rpm[i]},${plotData.error[i]},${plotData.stability_score[i]}\n`;
        }

        const blob = new Blob([csv], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `motor_data_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.csv`;
        a.click();
        URL.revokeObjectURL(url);

        this.showAlert(`Exported ${plotData.time.length} data points`, 'success');
    }

    showAbout() {
        new bootstrap.Modal(document.getElementById('about-modal')).show();
    }

    showAlert(message, type = 'info') {
        const alertEl = document.createElement('div');
        alertEl.className = `alert alert-${type} alert-dismissible fade show position-fixed`;
        alertEl.style.cssText = 'top: 80px; right: 20px; z-index: 9999; min-width: 300px; max-width: 400px;';
        alertEl.innerHTML = `
            ${message.replace(/\n/g, '<br>')}
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;
        document.body.appendChild(alertEl);
        setTimeout(() => alertEl.remove(), 5000);
    }
}

// Initialize
let controller;
document.addEventListener('DOMContentLoaded', () => {
    controller = new BLDCController();
});
