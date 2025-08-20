document.addEventListener('DOMContentLoaded', () => {
    // --- UI Element References ---
    const navItems = document.querySelectorAll('.left-nav .nav-item');
    const modulePages = document.querySelectorAll('.module-page');
    const connectBtn = document.getElementById('connect-btn');
    const connectionIndicator = document.getElementById('connection-indicator');
    const speedSlider = document.getElementById('speed-slider');
    const speedValue = document.getElementById('speed-value');
    const robotStatusText = document.getElementById('robot-status-text');
    const currentToolText = document.getElementById('current-tool-text');
    const logOutput = document.getElementById('log-output');
    const clearConsoleBtn = document.getElementById('clear-console');

    // Right Panel Coordinate Toggle Buttons
    const cartesianBtn = document.getElementById('cartesian-btn');
    const jointBtn = document.getElementById('joint-btn');
    // Right Panel Coordinate Displays
    const cartesianCoords = document.getElementById('cartesian-coords');
    const jointCoords = document.getElementById('joint-coords');

    // References for Cartesian coordinate displays (Right Panel)
    const coordX = document.getElementById('coord-x');
    const coordY = document.getElementById('coord-y');
    const coordZ = document.getElementById('coord-z');
    const coordRx = document.getElementById('coord-rx');
    const coordRy = document.getElementById('coord-ry');
    const coordRz = document.getElementById('coord-rz');

    // References for Joint coordinate displays (Right Panel)
    const coordJ1 = document.getElementById('coord-j1');
    const coordJ2 = document.getElementById('coord-j2');
    const coordJ3 = document.getElementById('coord-j3');
    const coordJ4 = document.getElementById('coord-j4');
    const coordJ5 = document.getElementById('coord-j5');
    const coordJ6 = document.getElementById('coord-j6');

    const rightPanelToolStatus = document.getElementById('right-panel-tool-status');
    const rightPanelToolState = document.getElementById('right-panel-tool-state');

    const toolSelectionButtons = document.querySelectorAll('.tool-btn');
    const toolControlPanels = document.querySelectorAll('.tool-controls');
    const softGripperOpenBtn = document.getElementById('soft-gripper-open');
    const softGripperCloseBtn = document.getElementById('soft-gripper-close');
    const softGripperStatus = document.getElementById('soft-gripper-status');
    const suctionOnBtn = document.getElementById('suction-on');
    const suctionOffBtn = document.getElementById('suction-off');
    const vacuumPressureDisplay = document.getElementById('vacuum-pressure');
    const suctionStatus = document.getElementById('suction-status');

    const pythonCodeEditor = document.getElementById('python-code');
    const scriptConsole = document.getElementById('script-console');
    const runScriptBtn = document.querySelector('#python-scripting .primary-btn');
    const goHomeBtn = document.getElementById('go-home-btn'); // Go Home button
    const teachPositionFromManualBtn = document.getElementById('teach-position-from-manual');

    // UI Element References for Backend Integration
    const usbPortSelect = document.getElementById('usbPortSelect');
    const backendStatusMessage = document.getElementById('backendStatusMessage');

    // Slider and value display references for Manual Control
    const worldModeBtn = document.getElementById('world-mode-btn');
    const toolModeBtn = document.getElementById('tool-mode-btn');
    const jointModeBtn = document.getElementById('joint-mode-btn');
    const cartesianSliders = document.getElementById('cartesian-sliders');
    const jointSliders = document.getElementById('joint-sliders');

    const xSlider = document.getElementById('x-slider');
    const xValue = document.getElementById('x-value');
    const ySlider = document.getElementById('y-slider');
    const yValue = document.getElementById('y-value');
    const zSlider = document.getElementById('z-slider');
    const zValue = document.getElementById('z-value');
    const rxSlider = document.getElementById('rx-slider');
    const rxValue = document.getElementById('rx-value');
    const rySlider = document.getElementById('ry-slider');
    const ryValue = document.getElementById('ry-value');
    const rzSlider = document.getElementById('rz-slider');
    const rzValue = document.getElementById('rz-value');
    const j1Slider = document.getElementById('j1-slider');
    const j1Value = document.getElementById('j1-value');
    const j2Slider = document.getElementById('j2-slider');
    const j2Value = document.getElementById('j2-value');
    const j3Slider = document.getElementById('j3-slider');
    const j3Value = document.getElementById('j3-value');
    const j4Slider = document.getElementById('j4-slider');
    const j4Value = document.getElementById('j4-value');
    const j5Slider = document.getElementById('j5-slider');
    const j5Value = document.getElementById('j5-value');
    const j6Slider = document.getElementById('j6-slider');
    const j6Value = document.getElementById('j6-value');

    // --- Teaching & Playback UI References ---
    const addCurrentPosBtn = document.getElementById('add-current-pos-btn');
    const waypointsUl = document.getElementById('waypoints-ul');
    const playbackPlayBtn = document.getElementById('playback-play-btn');
    const playbackPauseBtn = document.getElementById('playback-pause-btn');
    const playbackStopBtn = document.getElementById('playback-stop-btn');
    const playbackLoopBtn = document.getElementById('playback-loop-btn');

    // --- Drag Teaching & Playback (NEW) UI Elements ---
    const torqueToggleBtn = document.getElementById('torque-toggle-btn');
    const recordBtn = document.getElementById('record-btn');
    const recordingsUl = document.getElementById('recordings-ul');
    const recordingPlayBtn = document.getElementById('recording-play-btn');
    const recordingPauseBtn = document.getElementById('recording-pause-btn');
    const recordingStopBtn = document.getElementById('recording-stop-btn');
    const recordingLoopBtn = document.getElementById('recording-loop-btn');

    // Add test communication button
    const testCommBtn = document.createElement('button');
    testCommBtn.className = 'btn secondary-btn';
    testCommBtn.innerHTML = '<i class="fas fa-wifi"></i> Test Communication';
    testCommBtn.id = 'test-comm-btn';
    document.querySelector('.drag-teach-controls .drag-teach-buttons').appendChild(testCommBtn);

    // Add ping button for basic connectivity test
    const pingBtn = document.createElement('button');
    pingBtn.className = 'btn secondary-btn';
    pingBtn.innerHTML = '<i class="fas fa-signal"></i> Ping Servo';
    pingBtn.id = 'ping-btn';
    document.querySelector('.drag-teach-controls .drag-teach-buttons').appendChild(pingBtn);

    // --- State Variables ---
    let isRobotConnected = false; // Reflects actual backend connection status
    let currentTool = "None";
    let softGripperState = "Open";
    let suctionGripperState = "OFF";
    let robotCoordInterval = null; // To store interval ID for simulated movement or UI updates
    let selectedPort = null; // Stores the currently selected USB port for backend calls
    let currentControlMode = 'world'; // 'world', 'tool', or 'joint'

    // --- Teaching & Playback State Variables ---
    let waypoints = [];
    let selectedWaypointIndex = null;
    let playbackState = 'stopped'; // 'stopped', 'playing', 'paused'
    let isLoopingWaypoints = false;
    let loopTimeoutId = null;
    let currentLoopIndex = 0;

    // --- Drag Teaching State (NEW) ---
    let isTorqueOn = true; // Tracks the current torque state
    let isRecording = false; // True if continuous recording is active
    let recordingPollInterval = null; // Interval ID for polling robot coordinates during recording
    let currentRecordingData = []; // Stores the joint positions recorded during current session
    let teachRecordings = []; // Array to store saved recordings (e.g., [{name: "rec1", data: [...]}, ...])
    let selectedRecordingName = null; // Name of the currently selected recording for playback
    let recordingPlaybackState = 'stopped'; // 'stopped', 'playing', 'paused'
    let recordingPlaybackData = []; // The data for the currently playing recording
    let recordingLoopState = { active: false, timeoutId: null, currentIndex: 0 }; // For looped playback

    // --- Constants for Teaching ---
    const RECORDING_INTERVAL_MS = 100; // Interval for polling robot coords during recording


    // Initial robot coordinates (will be updated by sliders or actual robot feedback)
    let currentRobotCoords = {
        x: 0.00, y: 0.00, z: 0.00,
        rx: 0.00, ry: 0.00, rz: 0.00,
        j1: 0.00, j2: 0.00, j3: 0.00,
        j4: 0.00, j5: 0.00, j6: 0.00
    };

    // Backend server URL
    const BACKEND_URL = 'http://localhost:5000';

    // --- Helper Functions ---
    function addLog(message, type = 'info', targetConsole = logOutput) {
        const p = document.createElement('p');
        const timestamp = new Date().toLocaleTimeString();
        p.className = `log-${type}`;
        p.textContent = `[${timestamp}] ${message}`;
        targetConsole.appendChild(p);
        targetConsole.scrollTop = targetConsole.scrollHeight; // Scroll to bottom
    }

    function updateRobotCoordinatesUI() {
        // This function now updates both Cartesian and Joint displays in the right panel
        // It converts radians to degrees for display
        coordX.textContent = currentRobotCoords.x.toFixed(2);
        coordY.textContent = currentRobotCoords.y.toFixed(2);
        coordZ.textContent = currentRobotCoords.z.toFixed(2);
        coordRx.textContent = currentRobotCoords.rx.toFixed(2);
        coordRy.textContent = currentRobotCoords.ry.toFixed(2);
        coordRz.textContent = currentRobotCoords.rz.toFixed(2);

        coordJ1.textContent = (currentRobotCoords.j1 * 180 / Math.PI).toFixed(2);
        coordJ2.textContent = (currentRobotCoords.j2 * 180 / Math.PI).toFixed(2);
        coordJ3.textContent = (currentRobotCoords.j3 * 180 / Math.PI).toFixed(2);
        coordJ4.textContent = (currentRobotCoords.j4 * 180 / Math.PI).toFixed(2);
        coordJ5.textContent = (currentRobotCoords.j5 * 180 / Math.PI).toFixed(2);
        coordJ6.textContent = (currentRobotCoords.j6 * 180 / Math.PI).toFixed(2);
    }


    // This function updates both left panel sliders and right panel displays
    function updateManualControlSlidersAndUI() {
        // Update slider positions and their displayed values based on currentRobotCoords
        xSlider.value = currentRobotCoords.x;
        xValue.textContent = currentRobotCoords.x.toFixed(2);
        ySlider.value = currentRobotCoords.y;
        yValue.textContent = currentRobotCoords.y.toFixed(2);
        zSlider.value = currentRobotCoords.z;
        zValue.textContent = currentRobotCoords.z.toFixed(2);
        rxSlider.value = currentRobotCoords.rx;
        rxValue.textContent = currentRobotCoords.rx.toFixed(2);
        rySlider.value = currentRobotCoords.ry;
        ryValue.textContent = currentRobotCoords.ry.toFixed(2);
        rzSlider.value = currentRobotCoords.rz;
        rzValue.textContent = currentRobotCoords.rz.toFixed(2);

        // Joint sliders and their values
        // Sliders work with radians * 100, display shows degrees
        j1Slider.value = (currentRobotCoords.j1 * 100).toFixed(0);
        j1Value.textContent = (currentRobotCoords.j1 * 180 / Math.PI).toFixed(2);
        j2Slider.value = (currentRobotCoords.j2 * 100).toFixed(0);
        j2Value.textContent = (currentRobotCoords.j2 * 180 / Math.PI).toFixed(2);
        j3Slider.value = (currentRobotCoords.j3 * 100).toFixed(0);
        j3Value.textContent = (currentRobotCoords.j3 * 180 / Math.PI).toFixed(2);
        j4Slider.value = (currentRobotCoords.j4 * 100).toFixed(0);
        j4Value.textContent = (currentRobotCoords.j4 * 180 / Math.PI).toFixed(2);
        j5Slider.value = (currentRobotCoords.j5 * 100).toFixed(0);
        j5Value.textContent = (currentRobotCoords.j5 * 180 / Math.PI).toFixed(2);
        j6Slider.value = (currentRobotCoords.j6 * 100).toFixed(0);
        j6Value.textContent = (currentRobotCoords.j6 * 180 / Math.PI).toFixed(2);

        // Also ensure the right panel coordinates are updated
        updateRobotCoordinatesUI();

        if (!isRobotConnected) {
            clearInterval(robotCoordInterval); // Ensure interval is cleared if disconnected
        }
    }

    // Debounce function to limit how often a function is called
    function debounce(func, delay) {
        let timeout;
        const wrapper = function(...args) {
            const context = this;
            clearTimeout(timeout);
            timeout = setTimeout(() => func.apply(context, args), delay);
        };
        // Expose the original function to allow immediate (non-debounced) calls when precise timing is needed
        wrapper.callback = func;
        return wrapper;
    }

    // Function to send joint positions to backend (used by manual control, teaching, and playback)
    // This is debounced to avoid sending too many requests while a slider is being dragged
    const sendJointPositionsToBackend = debounce(async (jointData) => {
        // jointData: An object containing j1, j2, j3, j4, j5, j6 in radians.
        // This function will be called with `currentRobotCoords` from manual sliders,
        // or with specific waypoint/recorded data from playback functions.

        if (!isRobotConnected) {
            addLog("Robot not connected! Cannot send joint commands.", 'error');
            return;
        }

        const speedPercentage = parseFloat(speedSlider.value); // Get speed from the global speed slider (0 to 100)
        // If speed is 0%, block movement entirely.
        if (isNaN(speedPercentage) || speedPercentage <= 0) {
            addLog("Speed is 0%. Movement blocked.", 'warn');
            return;
        }

        // Define min and max times for movement (in seconds)
        const minTime = 2.0;
        const maxTime = 5.0;

        // Calculate time_from_start based on speed slider percentage
        const normalizedSpeed = speedPercentage / 100.0;
        const invertedNormalizedSpeed = 1.0 - normalizedSpeed;
        const timeFromStartSec = minTime + (maxTime - minTime) * invertedNormalizedSpeed;

        // Construct the payload using the provided jointData and calculated time_from_start
        const payload = {
            j1: jointData.j1,
            j2: jointData.j2,
            j3: jointData.j3,
            j4: jointData.j4,
            j5: jointData.j5,
            j6: jointData.j6,
            time_from_start: timeFromStartSec
        };

        addLog(`Sending joint command: J1:${payload.j1.toFixed(2)}, J2:${payload.j2.toFixed(2)}, J3:${payload.j3.toFixed(2)}, ... Time: ${timeFromStartSec.toFixed(2)}s`, 'info');
        return await sendCommandToBackend('/api/set_joint_positions', payload, 'set joint positions');
    }, 100); // Debounce by 100ms


    // Very basic Python syntax highlighting (client-side only for demo)
    function highlightPythonCode() {
        if (!pythonCodeEditor) {
            console.warn("Python code editor element not found.");
            return;
        }

        const code = pythonCodeEditor.textContent;
        const keywords = ['robot', 'move_to', 'set_gripper', 'wait', 'if', 'else', 'for', 'while', 'def', 'class', 'import', 'from', 'as', 'True', 'False', 'None', 'and', 'or', 'not', 'in', 'is', 'return', 'print'];
        const operators = ['+', '-', '*', '/', '%', '**', '//', '=', '==', '!=', '<', '>', '<=', '>=', 'and', 'or', 'not'];
        const strings = /(".*?")|('.*?')/g;
        const comments = /(#.*)/g;
        const numbers = /\b\d+(\.\d+)?\b/g;

        let highlightedCode = code;

        // Order matters: comments first to avoid highlighting inside them
        highlightedCode = highlightedCode.replace(comments, '<span class="token-comment">$&</span>');
        highlightedCode = highlightedCode.replace(strings, '<span class="token-string">$&</span>');
        highlightedCode = highlightedCode.replace(numbers, '<span class="token-number">$&</span>');

        keywords.forEach(keyword => {
            const regex = new RegExp(`\\b${keyword}\\b`, 'g');
            highlightedCode = highlightedCode.replace(regex, `<span class="token-keyword">${keyword}</span>`);
        });

        operators.forEach(operator => {
            // Escape operators for regex if they are special characters
            const escapedOperator = operator.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
            const regex = new RegExp(`(${escapedOperator})`, 'g');
            highlightedCode = highlightedCode.replace(regex, `<span class="token-operator">$&</span>`);
        });

        highlightedCode = highlightedCode.replace(/\n/g, '<br>'); // Preserve newlines

        pythonCodeEditor.innerHTML = highlightedCode;
    }

    // --- Backend Communication Functions ---

    // Function to populate the dropdown by fetching real data from backend
    async function populateUsbPorts() {
        // Clear existing options
        usbPortSelect.innerHTML = '';
        const defaultOption = document.createElement('option');
        defaultOption.value = '';
        defaultOption.textContent = 'Loading Ports...';
        defaultOption.disabled = true;
        defaultOption.selected = true;
        usbPortSelect.appendChild(defaultOption);

        backendStatusMessage.textContent = "Fetching available ports...";
        backendStatusMessage.className = "status-message"; // Reset to info style

        try {
            const response = await fetch(`${BACKEND_URL}/api/ports`);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const ports = await response.json();

            // Clear again before populating actual ports
            usbPortSelect.innerHTML = '';

            const initialOption = document.createElement('option');
            initialOption.value = '';
            initialOption.textContent = '-- Select a Port --';
            initialOption.disabled = true;
            initialOption.selected = true;
            usbPortSelect.appendChild(initialOption);

            if (ports.length === 0) {
                const noPortsOption = document.createElement('option');
                noPortsOption.value = '';
                noPortsOption.textContent = 'No USB ports detected';
                noPortsOption.disabled = true;
                usbPortSelect.appendChild(noPortsOption);
                backendStatusMessage.textContent = 'No USB ports found. Is the robot connected and backend running?';
                backendStatusMessage.classList.add('error');
            } else {
                ports.forEach(port => {
                    const option = document.createElement('option');
                    option.value = port;
                    option.textContent = port;
                    if (port === '/dev/ttyUSB0') { // Heuristic for robot port
                         option.textContent += ' (Robot Arm)';
                         selectedPort = port; // Auto-select the robot arm port if found
                         option.selected = true;
                    }
                    usbPortSelect.appendChild(option);
                });
                if (selectedPort) {
                    backendStatusMessage.textContent = `Robot port selected: ${selectedPort}`;
                    backendStatusMessage.classList.add('success');
                } else {
                    backendStatusMessage.textContent = "Select a port for the robot.";
                    backendStatusMessage.classList.add('info');
                }
            }
            updateConnectButtonState(); // Update button state after ports are populated
        } catch (error) {
            console.error('Failed to fetch USB ports:', error);
            backendStatusMessage.textContent = `Error fetching ports: ${error.message}. Is backend running at ${BACKEND_URL}?`;
            backendStatusMessage.classList.add('error');

            usbPortSelect.innerHTML = '';
            const errorOption = document.createElement('option');
            errorOption.value = '';
            errorOption.textContent = '-- Error fetching ports --';
            errorOption.disabled = true;
            errorOption.selected = true;
            usbPortSelect.appendChild(errorOption);
            updateConnectButtonState(); // Update button state even on error
        }
    }

    // Function to update the connect/disconnect button state based on port selection
    function updateConnectButtonState() {
        // Enable connect button only if a port is selected and not already connected
        if (usbPortSelect.value && usbPortSelect.value !== '-- Select a Port --' && usbPortSelect.value !== 'No USB ports detected' && !isRobotConnected) {
            connectBtn.disabled = false;
            connectBtn.classList.add('connect');
        } else if (isRobotConnected) {
            // If already connected, the button becomes 'Disconnect' and should be enabled
            connectBtn.disabled = false;
            connectBtn.classList.add('active'); // Indicate active state
            connectBtn.classList.remove('connect'); // Remove connect styling
            connectBtn.textContent = 'Disconnect';
        } else {
            // Otherwise, disable it
            connectBtn.disabled = true;
            connectBtn.classList.remove('connect');
            connectBtn.classList.remove('active');
            connectBtn.textContent = 'Connect';
        }
    }

    // Generic function to send command to backend
    async function sendCommandToBackend(endpoint, bodyData, commandName) {
        // Log sending command for all commands
        addLog(`Sending command to backend: ${commandName}`, 'info');

        try {
            const response = await fetch(`${BACKEND_URL}${endpoint}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(bodyData),
            });

            const result = await response.json();

            // Check if it's a command whose success/failure logs should be suppressed for network issues
            const isSuppressibleCommand = (commandName === 'set joint positions' || commandName === 'Go Home');

            if (response.ok && result.success) {
                // Only log success to GUI if it's NOT a suppressible command
                if (!isSuppressibleCommand) {
                    addLog(`Backend responded (Success for ${commandName}): ${result.output || result.message || 'No specific output.'}`, 'success');
                }
                // Always return success
                return { success: true, ...result };
            } else {
                // It's a failure (response.ok is false or result.success is false)
                const errorMessage = result.error || result.message || 'Unknown error.';
                // Determine if it's a network-related failure from the backend's explicit error message
                const backendReportsNetworkError = (errorMessage.includes('Could not reach backend') || errorMessage.includes('Network error'));

                if (isSuppressibleCommand && backendReportsNetworkError) {
                    // Suppress GUI log, but always console.error for debugging
                    console.error(`Suppressed GUI log (backend reported network error) for ${commandName}: ${errorMessage}`);
                } else {
                    // Log all other failures to GUI
                    addLog(`Backend responded (Failure for ${commandName}): ${errorMessage}`, 'error');
                }
                // Always console.error for any failure for debugging
                console.error(`Backend responded (Failure for ${commandName}): ${errorMessage}`);
                return { success: false, message: errorMessage };
            }
        } catch (error) { // This catch block handles actual network errors (e.g., fetch failed)
            const networkErrorMessage = `Network error: ${error.message}`;
            // Determine if it's the exact network unreachable error we want to suppress
            const isUnreachableFetchError = networkErrorMessage.includes('NetworkError when attempting to fetch resource.') || networkErrorMessage.includes('Could not reach backend at http://localhost:5000'); // Check for the specific log message text

            // Check if it's a command whose logs should be suppressed and if it's the specific network error
            const isSuppressibleCommand = (commandName === 'set joint positions' || commandName === 'Go Home');

            if (isSuppressibleCommand && isUnreachableFetchError) {
                // Suppress GUI log, but always console.error for debugging
                console.error(`Suppressed GUI log (fetch network error) for ${commandName}: ${networkErrorMessage}`);
            } else {
                // Log other network errors to GUI
                addLog(`Network error for ${commandName}: Could not reach backend at ${BACKEND_URL}. Is it running?`, 'error');
            }
            // Always console.error for any network error for debugging
            console.error(`Network or server error for ${commandName}:`, error);
            return { success: false, message: networkErrorMessage };
        }
    }

    /**
     * Fetches the list of saved recordings from the backend.
     */
    async function loadRecordingsList() {
        const data = await sendCommandToBackend('/api/get_recordings', {}, 'Get Recordings List');
        if (data && data.success) {
            teachRecordings = data.recordings;
            renderRecordings();
        } else {
            addLog("Failed to load recordings list from backend.", 'error');
            teachRecordings = [];
            renderRecordings();
        }
    }

    // --- Event Listeners ---

    // Navigation
    navItems.forEach(item => {
        item.addEventListener('click', () => {
            navItems.forEach(nav => nav.classList.remove('active'));
            item.classList.add('active');

            const targetModule = item.dataset.module;
            modulePages.forEach(page => {
                if (page.id === targetModule) {
                    page.classList.add('active');
                } else {
                    page.classList.remove('active');
                }
            });
            addLog(`Navigated to: ${item.textContent.trim()}`);

            if (targetModule === 'python-scripting') {
                highlightPythonCode();
            }
            // When navigating to manual control, ensure the correct sliders are shown
            if (targetModule === 'manual-control') {
                // Ensure initial state is World mode and Cartesian sliders active
                currentControlMode = 'world';
                cartesianSliders.classList.add('active');
                jointSliders.classList.remove('active');
                // Update button states
                worldModeBtn.classList.add('active');
                toolModeBtn.classList.remove('active');
                jointModeBtn.classList.remove('active');
            }
        });
    });

    // Connect/Disconnect Button
    connectBtn.addEventListener('click', () => {
        connectBtn.disabled = true;
        if (isRobotConnected) {
            // --- DISCONNECT SEQUENCE ---
            addLog("Disconnecting... Sending shutdown command...", 'warn');
            connectBtn.textContent = 'Disconnecting...';
            backendStatusMessage.textContent = 'Sending shutdown command...';
            backendStatusMessage.className = 'status-message info';
            fetch(`${BACKEND_URL}/api/disconnect_and_shutdown`, { method: 'POST' })
                .then(res => res.json())
                .then(data => {
                    addLog(`Backend shutdown: ${data.message}`, 'success');
                    backendStatusMessage.textContent = 'Disconnected successfully.';
                    backendStatusMessage.classList.add('success');
                })
                .catch(err => {
                    addLog(`Shutdown command failed: ${err}`, 'error');
                    backendStatusMessage.textContent = `Shutdown failed: ${err.message}`;
                    backendStatusMessage.classList.add('error');
                })
                .finally(() => {
                    isRobotConnected = false;
                    connectionIndicator.classList.replace('connected', 'disconnected');
                    connectBtn.textContent = 'Connect';
                    connectBtn.classList.remove('active');
                    robotStatusText.textContent = 'Disconnected';
                    addLog("Disconnected from UI.", 'warn');
                    clearInterval(robotCoordInterval); // Stop any ongoing UI update intervals
                    Object.keys(currentRobotCoords).forEach(key => currentRobotCoords[key] = 0); // Reset coords
                    updateManualControlSlidersAndUI(); // Update UI to reflect reset coords
                    updateRobotCoordinatesUI(); // Update right panel too
                    connectBtn.disabled = false;
                    usbPortSelect.disabled = false;
                    updateConnectButtonState();
                    updatePlaybackControlsState(); // Disable playback controls on disconnect
                    updateRecordingPlaybackControls();
                    updateDragTeachControlsState();
                });
        } else {
            // --- CONNECT LOGIC ---
            selectedPort = usbPortSelect.value;

            if (!selectedPort || selectedPort === '-- Select a Port --' || selectedPort === 'No USB ports detected') {
                addLog("Please select a robot port first.", 'error');
                backendStatusMessage.textContent = "Error: No port selected.";
                backendStatusMessage.classList.add('error');
                connectBtn.disabled = false;
                return;
            }

            addLog(`Attempting to connect to robot via port ${selectedPort}...`, 'info');
            connectBtn.textContent = 'Connecting...';
            connectBtn.disabled = true;
            connectionIndicator.classList.remove('disconnected');
            connectionIndicator.classList.remove('connected');
            usbPortSelect.disabled = true;
            robotStatusText.textContent = 'Connecting via backend...';

            (async () => { // Self-executing async function to allow await
                const runNodeResult = await sendCommandToBackend(
                    '/api/run_ros_node',
                    { port: selectedPort },
                    'run ROS node'
                );

                if (runNodeResult.success) {
                    addLog("ROS node started. Attempting to bring up robot via backend...", 'info');
                    robotStatusText.textContent = 'Bringing Up Systems...';
                    const bringUpResult = await sendCommandToBackend(
                        '/api/bringup_robot',
                        { port: selectedPort },
                        'bring up robot launch'
                    );

                    if (bringUpResult.success) {
                        isRobotConnected = true;
                        connectionIndicator.classList.add('connected');
                        connectBtn.textContent = 'Disconnect';
                        connectBtn.classList.add('active');
                        robotStatusText.textContent = 'Idle';
                        addLog("Rotatum Chotu connected successfully!", 'success');
                        backendStatusMessage.textContent = "Robot Connected & Ready!";
                        backendStatusMessage.classList.add('success');
                        
                        updateManualControlSlidersAndUI();
                        updatePlaybackControlsState(); // Update playback controls on connect
                        updateRecordingPlaybackControls();
                        updateDragTeachControlsState();

                        robotCoordInterval = setInterval(updateManualControlSlidersAndUI, 500);

                    } else {
                        addLog(`Connection failed: ${bringUpResult.message}.`, 'error');
                        backendStatusMessage.textContent = `Connection failed: ${bringUpResult.message}`;
                        backendStatusMessage.classList.add('error');
                        isRobotConnected = false;
                        connectionIndicator.classList.remove('connected');
                        connectionIndicator.classList.add('disconnected');
                        connectBtn.textContent = 'Connect (Failed)';
                        connectBtn.classList.remove('active');
                        robotStatusText.textContent = 'Disconnected';
                    }
                } else {
                    addLog(`Connection failed: ${runNodeResult.message}`, 'error');
                    backendStatusMessage.textContent = `Connection failed: ${runNodeResult.message}`;
                    backendStatusMessage.classList.add('error');
                    isRobotConnected = false;
                    connectionIndicator.classList.remove('connected');
                    connectionIndicator.classList.add('disconnected');
                    connectBtn.textContent = 'Connect (Failed)';
                    connectBtn.classList.remove('active');
                    robotStatusText.textContent = 'Disconnected';
                }
                connectBtn.disabled = false;
                usbPortSelect.disabled = isRobotConnected;
                updateConnectButtonState();
                updatePlaybackControlsState();
                updateRecordingPlaybackControls();
                updateDragTeachControlsState();
            })();
        }
    });

    // USB Port Dropdown Change (Update selectedPort variable and button state)
    usbPortSelect.addEventListener('change', () => {
        selectedPort = usbPortSelect.value;
        addLog(`Selected USB Port: ${selectedPort}`, 'info');
        backendStatusMessage.textContent = `Selected port: ${selectedPort}`;
        backendStatusMessage.classList.add('info');
        if (isRobotConnected) {
            addLog("Port changed while connected. Please disconnect and reconnect if desired.", 'warn');
        }
        updateConnectButtonState(); // Update button state when port changes
    });


    // Global Speed Slider (existing logic)
    speedSlider.addEventListener('input', () => {
        speedValue.textContent = `${speedSlider.value}%`;
        addLog(`Global Speed set to: ${speedSlider.value}%`);
        // Trigger a joint command send to update speed instantly, if in joint mode and connected
        if (currentControlMode === 'joint' && isRobotConnected) {
            // Re-send current joint positions with new speed
            sendJointPositionsToBackend(currentRobotCoords);
        }
    });

    // Clear Console Log (existing logic)
    clearConsoleBtn.addEventListener('click', () => {
        logOutput.innerHTML = '<p class="log-info">[Console cleared]</p>';
    });

    // Right Panel Coordinate Toggle (existing logic)
    cartesianBtn.addEventListener('click', () => {
        cartesianBtn.classList.add('active');
        jointBtn.classList.remove('active');
        cartesianCoords.classList.add('active');
        jointCoords.classList.remove('active');
    });

    jointBtn.addEventListener('click', () => {
        jointBtn.classList.add('active');
        cartesianBtn.classList.remove('active');
        jointCoords.classList.add('active');
        cartesianCoords.classList.remove('active');
    });

    // Manual Control Mode Toggles (World, Tool, Joint)
    worldModeBtn.addEventListener('click', () => {
        if (!isRobotConnected) { addLog("Robot not connected!", 'error'); return; }
        worldModeBtn.classList.add('active');
        toolModeBtn.classList.remove('active');
        jointModeBtn.classList.remove('active');

        cartesianSliders.classList.add('active');
        jointSliders.classList.remove('active');
        currentControlMode = 'world';
        addLog("Manual Control Mode: World");
    });

    toolModeBtn.addEventListener('click', () => {
        if (!isRobotConnected) { addLog("Robot not connected!", 'error'); return; }
        toolModeBtn.classList.add('active');
        worldModeBtn.classList.remove('active');
        jointModeBtn.classList.remove('active');

        cartesianSliders.classList.add('active');
        jointSliders.classList.remove('active');
        currentControlMode = 'tool';
        addLog("Manual Control Mode: Tool (Cartesian)");
    });

    jointModeBtn.addEventListener('click', () => {
        if (!isRobotConnected) { addLog("Robot not connected!", 'error'); return; }
        jointModeBtn.classList.add('active');
        worldModeBtn.classList.remove('active');
        toolModeBtn.classList.remove('active');

        jointSliders.classList.add('active');
        cartesianSliders.classList.remove('active');
        currentControlMode = 'joint';
        addLog("Manual Control Mode: Joint");
    });

    // Dashboard "Start Manual Control" button (existing logic)
    document.querySelector('.dashboard-summary + .btn').addEventListener('click', (e) => {
        const targetModule = e.target.dataset.module;
        document.querySelector(`.nav-item[data-module="${targetModule}"]`).click();
    });

    // "Go Home" button functionality
    if (goHomeBtn) {
        goHomeBtn.addEventListener('click', async () => {
            if (!isRobotConnected) {
                addLog("Robot not connected! Cannot send 'Go Home' command.", 'error');
                return;
            }
            // Respect speed = 0% as a hard stop
            const speedPercentage = parseFloat(speedSlider.value);
            if (isNaN(speedPercentage) || speedPercentage <= 0) {
                addLog("Speed is 0%. 'Go Home' is blocked.", 'warn');
                return;
            }
            addLog("Sending robot to home position (all joints to 0 radians)...", 'info');

            // For 'Go Home', use current speed setting
            const minTime = 2.0;
            const maxTime = 5.0;
            const normalizedSpeed = speedPercentage / 100.0;
            const invertedNormalizedSpeed = 1.0 - normalizedSpeed;
            const timeFromStartSec = minTime + (maxTime - minTime) * invertedNormalizedSpeed;

            const homeJoints = {
                j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 0.0, j6: 0.0,
                time_from_start: timeFromStartSec // Use current speed setting for Go Home
            };

            // Update UI sliders and coordinates immediately
            Object.assign(currentRobotCoords, homeJoints);
            updateManualControlSlidersAndUI(); // Update UI with home position

            const response = await sendCommandToBackend('/api/set_joint_positions', homeJoints, 'Go Home');
            if (response.success) {
                addLog("Robot successfully commanded to home position.", 'success');
            }
        });
    }

    // Gripper Operations - Tool Selection (existing logic)
    toolSelectionButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            toolSelectionButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            const selectedTool = btn.dataset.tool;
            toolControlPanels.forEach(panel => {
                if (panel.id === `${selectedTool}-controls`) {
                    panel.classList.add('active');
                } else {
                    panel.classList.remove('active');
                }
            });
            currentTool = btn.textContent.trim();
            rightPanelToolStatus.textContent = currentTool;

            if (selectedTool === "soft-gripper") {
                rightPanelToolState.textContent = softGripperState;
            } else if (selectedTool === "suction-gripper") {
                rightPanelToolState.textContent = suctionGripperState;
            } else if (selectedTool === "pen-mount") {
                rightPanelToolState.textContent = "N/A";
            } else {
                rightPanelToolState.textContent = "Idle";
            }
            addLog(`Selected tool: ${currentTool}`);
        });
    });

    // Soft Gripper Actions (Simulated) (existing logic)
    softGripperOpenBtn.addEventListener('click', () => {
        if (!isRobotConnected) { addLog("Robot not connected!", 'error'); return; }
        softGripperState = "Opening...";
        softGripperStatus.textContent = "Status: Opening...";
        rightPanelToolState.textContent = "Opening...";
        addLog("Soft Gripper: Opening...", 'info');
        setTimeout(() => {
            softGripperState = "Open";
            softGripperStatus.textContent = "Status: Open";
            rightPanelToolState.textContent = "Open";
            addLog("Soft Gripper: Opened", 'success');
        }, 800);
    });
    softGripperCloseBtn.addEventListener('click', () => {
        if (!isRobotConnected) { addLog("Robot not connected!", 'error'); return; }
        softGripperState = "Closing...";
        softGripperStatus.textContent = "Status: Closing...";
        rightPanelToolState.textContent = "Closing...";
        addLog("Soft Gripper: Closing...", 'info');
        setTimeout(() => {
            softGripperState = "Closed";
            softGripperStatus.textContent = "Status: Closed";
            rightPanelToolState.textContent = "Closed";
            addLog("Soft Gripper: Closed", 'success');
        }, 800);
    });

    // Suction Gripper Actions (Simulated) (existing logic)
    suctionOnBtn.addEventListener('click', () => {
        if (!isRobotConnected) { addLog("Robot not connected!", 'error'); return; }
        suctionGripperState = "Activating...";
        suctionStatus.textContent = "Status: Activating...";
        vacuumPressureDisplay.textContent = "0 kPa";
        rightPanelToolState.textContent = "Activating...";
        addLog("Suction Gripper: Activating...", 'info');
        setTimeout(() => {
            suctionGripperState = "ON";
            suctionStatus.textContent = "Status: ON";
            vacuumPressureDisplay.textContent = "80 kPa";
            rightPanelToolState.textContent = "ON";
            addLog("Suction Gripper: Activated", 'success');
        }, 1000);
    });
    suctionOffBtn.addEventListener('click', () => {
        if (!isRobotConnected) { addLog("Robot not connected!", 'error'); return; }
        suctionGripperState = "Deactivating...";
        suctionStatus.textContent = "Status: Deactivating...";
        vacuumPressureDisplay.textContent = "80 kPa";
        rightPanelToolState.textContent = "Deactivating...";
        addLog("Suction Gripper: Deactivating...", 'info');
        setTimeout(() => {
            suctionGripperState = "OFF";
            suctionStatus.textContent = "Status: OFF";
            vacuumPressureDisplay.textContent = "0 kPa";
            rightPanelToolState.textContent = "OFF";
            addLog("Suction Gripper: Deactivated", 'success');
        }, 700);
    });

    // Manual Control Sliders Event Listeners (Cartesian)
    if (xSlider) {
        xSlider.addEventListener('input', () => {
            currentRobotCoords.x = parseFloat(xSlider.value);
            updateManualControlSlidersAndUI();
        });
        ySlider.addEventListener('input', () => {
            currentRobotCoords.y = parseFloat(ySlider.value);
            updateManualControlSlidersAndUI();
        });
        zSlider.addEventListener('input', () => {
            currentRobotCoords.z = parseFloat(zSlider.value);
            updateManualControlSlidersAndUI();
        });
        rxSlider.addEventListener('input', () => {
            currentRobotCoords.rx = parseFloat(rxSlider.value);
            updateManualControlSlidersAndUI();
        });
        rySlider.addEventListener('input', () => {
            currentRobotCoords.ry = parseFloat(rySlider.value);
            updateManualControlSlidersAndUI();
        });
        rzSlider.addEventListener('input', () => {
            currentRobotCoords.rz = parseFloat(rzSlider.value);
            updateManualControlSlidersAndUI();
        });
    }

    // Manual Control Sliders Event Listeners (Joint)
    if (j1Slider) {
        j1Slider.addEventListener('input', () => {
            currentRobotCoords.j1 = parseFloat(j1Slider.value) / 100.0; // Convert slider value to radians
            updateManualControlSlidersAndUI();
            sendJointPositionsToBackend(currentRobotCoords); // Debounced call to backend
        });
        j2Slider.addEventListener('input', () => {
            currentRobotCoords.j2 = parseFloat(j2Slider.value) / 100.0;
            updateManualControlSlidersAndUI();
            sendJointPositionsToBackend(currentRobotCoords);
        });
        j3Slider.addEventListener('input', () => {
            currentRobotCoords.j3 = parseFloat(j3Slider.value) / 100.0;
            updateManualControlSlidersAndUI();
            sendJointPositionsToBackend(currentRobotCoords);
        });
        j4Slider.addEventListener('input', () => {
            currentRobotCoords.j4 = parseFloat(j4Slider.value) / 100.0;
            updateManualControlSlidersAndUI();
            sendJointPositionsToBackend(currentRobotCoords);
        });
        j5Slider.addEventListener('input', () => {
            currentRobotCoords.j5 = parseFloat(j5Slider.value) / 100.0;
            updateManualControlSlidersAndUI();
            sendJointPositionsToBackend(currentRobotCoords);
        });
        j6Slider.addEventListener('input', () => {
            currentRobotCoords.j6 = parseFloat(j6Slider.value) / 100.0;
            updateManualControlSlidersAndUI();
            sendJointPositionsToBackend(currentRobotCoords);
        });
    }

    // --- WAYPOINT TEACHING & PLAYBACK LOGIC ---

    /**
     * Updates the state of waypoint playback control buttons (play, pause, stop, loop).
     */
    function updatePlaybackControlsState() {
        const isPlaying = playbackState === 'playing';
        const isPaused = playbackState === 'paused';

        playbackPlayBtn.disabled = !isRobotConnected || isPlaying || selectedWaypointIndex === null;
        playbackPauseBtn.disabled = !isRobotConnected || (!isPlaying && !isPaused);
        playbackStopBtn.disabled = !isRobotConnected || (!isPlaying && !isPaused);
        playbackLoopBtn.disabled = !isRobotConnected || isPlaying || waypoints.length === 0;

        // Update Pause/Continue button text
        playbackPauseBtn.innerHTML = isPaused ? '<i class="fas fa-play"></i> Continue' : '<i class="fas fa-pause"></i> Pause';

        // Update Loop button text
        playbackLoopBtn.innerHTML = isLoopingWaypoints ? '<i class="fas fa-spinner fa-spin"></i> Looping...' : '<i class="fas fa-redo-alt"></i> Loop';
        if(isLoopingWaypoints) {
            playbackLoopBtn.classList.add('active');
        } else {
            playbackLoopBtn.classList.remove('active');
        }
    }

    /**
     * Renders the list of waypoints in the UI.
     */
    function renderWaypoints() {
        waypointsUl.innerHTML = ''; // Clear the list
        if (waypoints.length === 0) {
            const li = document.createElement('li');
            li.textContent = 'No waypoints recorded yet.';
            li.style.color = 'var(--text-secondary)';
            li.style.cursor = 'default';
            waypointsUl.appendChild(li);
        } else {
            waypoints.forEach((wp, index) => {
                const li = document.createElement('li');
                li.dataset.index = index; // Store index for event delegation
                if (index === selectedWaypointIndex) {
                    li.classList.add('selected');
                }

                // Display joint values for waypoint
                const textSpan = document.createElement('span');
                textSpan.className = 'waypoint-text';
                textSpan.textContent = `WP ${index + 1}: (J1: ${(wp.j1 * 180 / Math.PI).toFixed(1)}, J2: ${(wp.j2 * 180 / Math.PI).toFixed(1)}, ...)`;
                li.appendChild(textSpan);

                // Create delete button
                const deleteBtn = document.createElement('button');
                deleteBtn.className = 'btn delete-btn';
                deleteBtn.innerHTML = '<i class="fas fa-trash"></i>';
                li.appendChild(deleteBtn);

                waypointsUl.appendChild(li);
            });
        }
        updatePlaybackControlsState(); // Update buttons whenever list is re-rendered
    }

    // --- Waypoint Playback Controls (Individual Listeners) ---
    waypointsUl.addEventListener('click', (e) => {
        const li = e.target.closest('li');
        if (!li || li.textContent.includes('No waypoints')) return;

        if (e.target.closest('.delete-btn')) {
            const index = parseInt(li.dataset.index, 10);
            waypoints.splice(index, 1);
            if (selectedWaypointIndex === index) selectedWaypointIndex = null;
            else if (selectedWaypointIndex > index) selectedWaypointIndex--;
            renderWaypoints();
            addLog(`Waypoint ${index + 1} deleted.`, 'info');
        } else {
            selectedWaypointIndex = parseInt(li.dataset.index, 10);
            renderWaypoints();
            addLog(`Selected waypoint ${selectedWaypointIndex + 1}.`, 'info');
        }
    });

    addCurrentPosBtn.addEventListener('click', () => {
        if (!isRobotConnected) {
            addLog("Robot not connected. Cannot add waypoint.", 'error');
            return;
        }
        // Use the current coordinates from the UI state
        const newWaypoint = {
            j1: currentRobotCoords.j1, j2: currentRobotCoords.j2, j3: currentRobotCoords.j3,
            j4: currentRobotCoords.j4, j5: currentRobotCoords.j5, j6: currentRobotCoords.j6,
        };
        waypoints.push(newWaypoint);
        selectedWaypointIndex = waypoints.length - 1; // Auto-select the newly added waypoint
        addLog("Current position added as waypoint.", 'success');
        renderWaypoints();
    });

    teachPositionFromManualBtn.addEventListener('click', () => {
        addCurrentPosBtn.click(); // Programmatically click the main button
        const teachingNav = document.querySelector('.nav-item[data-module="teaching-playback"]');
        if (teachingNav) teachingNav.click();
    });

    async function moveToWaypoint(waypoint) {
        if (!isRobotConnected || !waypoint) {
            addLog("Cannot move to waypoint. Robot not connected or waypoint invalid.", 'error');
            return false;
        }
        const response = await sendJointPositionsToBackend.callback(waypoint); // Use .callback to bypass debounce
        if (response && response.success) {
            Object.assign(currentRobotCoords, waypoint); // Update internal coords
            updateManualControlSlidersAndUI(); // Update UI
            return true;
        }
        return false;
    }

    async function executeLoopStep() {
        if (!isLoopingWaypoints || waypoints.length === 0) {
            isLoopingWaypoints = false;
            playbackState = 'stopped';
            updatePlaybackControlsState();
            return;
        }

        const waypointToMove = waypoints[currentLoopIndex];
        addLog(`Looping: Moving to waypoint ${currentLoopIndex + 1}/${waypoints.length}.`, 'info');

        const moveSuccess = await moveToWaypoint(waypointToMove);

        if (!moveSuccess) {
            addLog("Loop stopped due to movement failure.", 'error');
            isLoopingWaypoints = false;
            playbackState = 'stopped';
            updatePlaybackControlsState();
            return;
        }

        // If loop was stopped while moving, exit
        if (!isLoopingWaypoints) return;

        currentLoopIndex = (currentLoopIndex + 1) % waypoints.length;

        // Wait for a bit before moving to the next point
        loopTimeoutId = setTimeout(executeLoopStep, 1000);
    }

    playbackPlayBtn.addEventListener('click', async () => {
        if (playbackState === 'playing' || selectedWaypointIndex === null) return;
        playbackState = 'playing';
        updatePlaybackControlsState();
        addLog(`Playing single waypoint ${selectedWaypointIndex + 1}.`, 'info');
        await moveToWaypoint(waypoints[selectedWaypointIndex]);
        playbackState = 'stopped';
        updatePlaybackControlsState();
    });

    playbackLoopBtn.addEventListener('click', () => {
        if (isLoopingWaypoints) {
            isLoopingWaypoints = false;
            playbackState = 'stopped';
            clearTimeout(loopTimeoutId);
            addLog("Waypoint loop stopped.", 'info');
        } else {
            if (waypoints.length === 0) {
                addLog("No waypoints to loop.", 'warn');
                return;
            }
            isLoopingWaypoints = true;
            playbackState = 'playing';
            currentLoopIndex = 0;
            addLog("Starting waypoint loop.", 'info');
            executeLoopStep();
        }
        updatePlaybackControlsState();
    });

    playbackPauseBtn.addEventListener('click', () => {
        // This button is complex with looping. For now, it just stops the loop.
        if (isLoopingWaypoints) {
            isLoopingWaypoints = false;
            playbackState = 'stopped';
            clearTimeout(loopTimeoutId);
            addLog("Waypoint loop stopped (paused).", 'info');
            updatePlaybackControlsState();
        }
    });

    playbackStopBtn.addEventListener('click', () => {
        isLoopingWaypoints = false;
        playbackState = 'stopped';
        clearTimeout(loopTimeoutId);
        addLog("Waypoint playback stopped.", 'info');
        updatePlaybackControlsState();
    });
    
    // --- DRAG TEACHING & PLAYBACK LOGIC (NEW) ---

    /**
     * Updates the UI of the torque button based on the current state.
     */
    function updateTorqueButton() {
        if (isTorqueOn) {
            torqueToggleBtn.innerHTML = '<i class="fas fa-bolt"></i> Torque ON';
            torqueToggleBtn.classList.remove('torque-off');
        } else {
            torqueToggleBtn.innerHTML = '<i class="fas fa-unlock"></i> Torque OFF';
            torqueToggleBtn.classList.add('torque-off');
        }
    }

    /**
     * Updates the UI of the record button based on the current state.
     */
    function updateRecordButton() {
        if (isRecording) {
            recordBtn.innerHTML = '<i class="fas fa-stop-circle"></i> Stop Recording';
            recordBtn.classList.add('active');
        } else {
            recordBtn.innerHTML = '<i class="fas fa-video"></i> Start Recording';
            recordBtn.classList.remove('active');
        }
    }

    /**
     * Updates the enabled/disabled state of the drag teaching controls.
     */
    function updateDragTeachControlsState() {
        // Teaching (torque/record) is only possible when the main ROS connection is OFF
        // to avoid serial port conflicts.
        const canTeach = !isRobotConnected && selectedPort;
        torqueToggleBtn.disabled = !canTeach;
        recordBtn.disabled = !canTeach || isTorqueOn; // Can't record if torque is on

        if (!canTeach) {
            torqueToggleBtn.title = "Disconnect from the robot to enable teaching.";
            recordBtn.title = "Disconnect from the robot and turn torque OFF to enable recording.";
        } else if (isTorqueOn) {
            recordBtn.title = "Turn torque OFF to enable recording.";
        }
        else {
            torqueToggleBtn.title = "";
            recordBtn.title = "";
        }
    }

    /**
     * Renders the list of saved recordings in the UI.
     */
    function renderRecordings() {
        recordingsUl.innerHTML = '';
        if (teachRecordings.length === 0) {
            const li = document.createElement('li');
            li.textContent = 'No recordings saved yet.';
            li.style.color = 'var(--text-secondary)';
            li.style.cursor = 'default';
            recordingsUl.appendChild(li);
        } else {
            teachRecordings.forEach((recName) => {
                const li = document.createElement('li');
                li.dataset.name = recName;
                if (recName === selectedRecordingName) li.classList.add('selected');
                li.innerHTML = `
                    <span class="recording-text">${recName.replace('.json', '')}</span>
                    <button class="btn delete-btn"><i class="fas fa-trash"></i></button>
                `;
                recordingsUl.appendChild(li);
            });
        }
        updateRecordingPlaybackControls();
    }

    /**
     * Updates the enabled/disabled state of the recording playback controls.
     */
    function updateRecordingPlaybackControls() {
        // Playback of recordings requires the ROS connection to be ON.
        const canPlay = isRobotConnected;
        const isPlaying = recordingPlaybackState === 'playing' || recordingLoopState.active;
        const isPaused = recordingPlaybackState === 'paused';
        const hasSelection = !!selectedRecordingName;

        recordingPlayBtn.disabled = !canPlay || isPlaying || !hasSelection;
        recordingPauseBtn.disabled = !canPlay || (!isPlaying && !isPaused);
        recordingStopBtn.disabled = !canPlay || (!isPlaying && !isPaused);
        recordingLoopBtn.disabled = !canPlay || isPlaying || !hasSelection;
        
        recordingPauseBtn.innerHTML = isPaused ? '<i class="fas fa-play"></i> Continue' : '<i class="fas fa-pause"></i> Pause';
        recordingLoopBtn.innerHTML = recordingLoopState.active ? '<i class="fas fa-spinner fa-spin"></i> Looping...' : '<i class="fas fa-redo-alt"></i> Loop';
    }

    /**
     * Executes a single step of the recording playback or loop.
     */
    async function executeRecordingStep() {
        const isLooping = recordingLoopState.active;
        let currentIndex = recordingLoopState.currentIndex;

        if ((!isLooping && recordingPlaybackState !== 'playing') || currentIndex >= recordingPlaybackData.length) {
             if (isLooping && recordingPlaybackData.length > 0) {
                // Loop back to the start
                recordingLoopState.currentIndex = 0;
            } else {
                // Stop playback
                recordingPlaybackState = 'stopped';
                recordingLoopState.active = false;
                clearTimeout(recordingLoopState.timeoutId);
                updateRecordingPlaybackControls();
                addLog("Recording playback finished.", 'info');
                return;
            }
        }
        
        // Refetch current index after potential loop reset
        currentIndex = recordingLoopState.currentIndex;
        const currentData = recordingPlaybackData[currentIndex];

        if (!currentData) {
            recordingPlaybackState = 'stopped';
            recordingLoopState.active = false;
            updateRecordingPlaybackControls();
            return;
        }
        
        // Use the time interval that was saved during recording
        const timeToNextPoint = currentData.interval > 0 ? currentData.interval : (RECORDING_INTERVAL_MS / 1000.0);

        const jointPositions = {
            j1: currentData.j1, j2: currentData.j2, j3: currentData.j3,
            j4: currentData.j4, j5: currentData.j5, j6: currentData.j6,
            time_from_start: timeToNextPoint
        };

        // We use sendCommandToBackend directly instead of the debounced version for precise timing
        const response = await sendCommandToBackend('/api/set_joint_positions', jointPositions, 'play recording step');
        
        // Check if playback was stopped while the command was in flight
        if (recordingPlaybackState === 'stopped') {
            return;
        }

        if (response.success) {
            Object.assign(currentRobotCoords, jointPositions);
            updateManualControlSlidersAndUI();

            recordingLoopState.currentIndex++;
            // The timeout should match the movement duration for smooth playback
            recordingLoopState.timeoutId = setTimeout(executeRecordingStep, timeToNextPoint * 1000);
        } else {
            addLog("Recording playback interrupted due to command failure.", 'error');
            recordingPlaybackState = 'stopped';
            recordingLoopState.active = false;
            clearTimeout(recordingLoopState.timeoutId);
            updateRecordingPlaybackControls();
        }
    }

    // --- Drag Teaching Event Listeners (NEW) ---
    torqueToggleBtn.addEventListener('click', async () => {
        if (isRobotConnected) {
            addLog("Cannot change torque while connected. Please disconnect first.", 'error');
            return;
        }
        const response = await sendCommandToBackend('/api/set_torque', { port: selectedPort, enable: !isTorqueOn }, 'Set Torque');
        if (response.success) {
            isTorqueOn = !isTorqueOn;
            updateTorqueButton();
            updateDragTeachControlsState();
            addLog(`Torque successfully turned ${isTorqueOn ? 'ON' : 'OFF'}.`, 'success');
        } else {
            addLog(`Failed to set torque: ${response.message}. Port might be in use.`, 'error');
        }
    });

    // Test communication button
    testCommBtn.addEventListener('click', async () => {
        if (!selectedPort) {
            addLog("Please select a port first.", 'error');
            return;
        }
        
        addLog("Testing servo communication...", 'info');
        const response = await sendCommandToBackend('/api/test_servo_communication', { port: selectedPort }, 'Test Servo Communication');
        if (response.success) {
            addLog(`Communication test successful: ${response.message}`, 'success');
        } else {
            addLog(`Communication test failed: ${response.message}`, 'error');
        }
    });

    // Ping button for basic connectivity
    pingBtn.addEventListener('click', async () => {
        if (!selectedPort) {
            addLog("Please select a port first.", 'error');
            return;
        }
        
        addLog("Pinging servo 1...", 'info');
        const response = await sendCommandToBackend('/api/ping_servo', { port: selectedPort, servo_id: 1 }, 'Ping Servo');
        if (response.success) {
            addLog(`Ping successful: ${response.message}`, 'success');
        } else {
            addLog(`Ping failed: ${response.message}`, 'error');
        }
    });

    recordBtn.addEventListener('click', async () => {
        if (isRecording) {
            // --- Stop Recording ---
            isRecording = false;
            clearInterval(recordingPollInterval);
            updateRecordButton();
            addLog("Recording stopped.", "info");
            if (currentRecordingData.length > 10) { // Only save if enough points
                const recordingName = prompt("Enter a name for this recording:", `Teach Recording ${teachRecordings.length + 1}`);
                if (recordingName && recordingName.trim() !== "") {
                    const saveResult = await sendCommandToBackend('/api/save_recording', { name: recordingName.trim(), recording_data: currentRecordingData }, 'Save Recording');
                    if (saveResult.success) {
                        await loadRecordingsList(); // Reload list to show new recording
                        addLog(`Recording "${recordingName.trim()}" saved.`, 'success');
                    }
                } else {
                    addLog("Recording not saved (name cancelled or empty).", 'warn');
                }
            } else {
                addLog("Recording too short, not saved.", 'warn');
            }
            currentRecordingData = []; // Clear current recording buffer
        } else {
            // --- Start Recording ---
            if (isTorqueOn) {
                addLog("Torque must be OFF to start recording. Please turn torque off first.", 'warn');
                return;
            }
            
            // Test communication before starting recording
            addLog("Testing communication before starting recording...", 'info');
            const testResponse = await sendCommandToBackend('/api/test_servo_communication', { port: selectedPort }, 'Test Communication Before Recording');
            if (!testResponse.success) {
                addLog(`Cannot start recording: ${testResponse.message}. Check servo connections and port.`, 'error');
                return;
            }
            
            isRecording = true;
            currentRecordingData = [];
            updateRecordButton();
            addLog("Starting recording... Move the robot arm manually.", 'info');
            let lastPollTime = Date.now();
            recordingPollInterval = setInterval(async () => {
                const response = await sendCommandToBackend('/api/get_current_joints', { port: selectedPort }, 'Get Current Joints for Recording');
                if (response.success && response.joints) {
                    const now = Date.now();
                    const interval = (now - lastPollTime) / 1000.0; // interval in seconds
                    lastPollTime = now;
                    const point = { ...response.joints, interval: interval };
                    currentRecordingData.push(point);
                } else {
                    addLog(`Recording failed to get joint position: ${response.message}`, 'error');
                    // Stop recording on repeated failures
                    if (currentRecordingData.length > 0) {
                        addLog("Stopping recording due to communication failures.", 'warn');
                        recordBtn.click(); // Trigger stop recording
                    }
                }
            }, RECORDING_INTERVAL_MS);
        }
    });

    recordingsUl.addEventListener('click', async (e) => {
        const li = e.target.closest('li');
        if (!li || li.textContent.includes('No recordings')) return;
        const recName = li.dataset.name;

        if (e.target.closest('.delete-btn')) {
            e.stopPropagation(); // Prevent selection when deleting
            if (confirm(`Are you sure you want to delete the recording "${recName.replace('.json', '')}"?`)) {
                const response = await sendCommandToBackend('/api/delete_recording', { name: recName }, 'Delete Recording');
                if (response.success) {
                    if (selectedRecordingName === recName) {
                         selectedRecordingName = null;
                         recordingPlaybackData = [];
                    }
                    await loadRecordingsList(); // Reload list after deletion
                }
            }
        } else {
            selectedRecordingName = recName;
            renderRecordings(); // Re-render to show selection
            const response = await sendCommandToBackend('/api/load_recording', { name: recName }, 'Load Recording');
            if (response.success && response.recording_data) {
                recordingPlaybackData = response.recording_data;
                addLog(`Loaded recording: ${recName.replace('.json', '')} with ${recordingPlaybackData.length} points.`, 'success');
            } else {
                 addLog(`Failed to load recording: ${response.message}`, 'error');
                 recordingPlaybackData = [];
            }
            updateRecordingPlaybackControls();
        }
    });

    recordingPlayBtn.addEventListener('click', async () => {
        if (!selectedRecordingName || recordingPlaybackData.length === 0) {
            addLog("Please select a valid recording to play.", 'warn');
            return;
        }
        if (recordingPlaybackState === 'paused') {
            recordingPlaybackState = 'playing';
            addLog("Resuming recording playback.", 'info');
            executeRecordingStep(); // Continues from saved index
        } else {
            recordingPlaybackState = 'playing';
            recordingLoopState.active = false; // Ensure not looping unless explicitly set
            recordingLoopState.currentIndex = 0;
            addLog("Starting recording playback.", 'info');
            executeRecordingStep();
        }
        updateRecordingPlaybackControls();
    });

    recordingLoopBtn.addEventListener('click', () => {
        if (!selectedRecordingName || recordingPlaybackData.length === 0) {
            addLog("Please select a valid recording to loop.", 'warn');
            return;
        }
        if (recordingLoopState.active) {
            recordingLoopState.active = false;
            clearTimeout(recordingLoopState.timeoutId);
            recordingPlaybackState = 'stopped';
            addLog("Recording loop stopped.", 'info');
        } else {
            recordingLoopState.active = true;
            recordingPlaybackState = 'playing'; // Set to playing when looping
            recordingLoopState.currentIndex = 0;
            addLog("Starting recording loop.", 'info');
            executeRecordingStep();
        }
        updateRecordingPlaybackControls();
    });

    recordingPauseBtn.addEventListener('click', () => {
        if (recordingPlaybackState === 'playing' || recordingLoopState.active) {
            recordingPlaybackState = 'paused';
            recordingLoopState.active = false; // Pause also stops looping
            clearTimeout(recordingLoopState.timeoutId);
            addLog("Recording playback paused.", 'info');
        } else if (recordingPlaybackState === 'paused') {
            recordingPlaybackState = 'playing';
            addLog("Resuming recording playback.", 'info');
            executeRecordingStep(); // Resumes from paused index
        }
        updateRecordingPlaybackControls();
    });

    recordingStopBtn.addEventListener('click', () => {
        recordingPlaybackState = 'stopped';
        recordingLoopState.active = false;
        clearTimeout(recordingLoopState.timeoutId);
        addLog("Recording playback stopped.", 'info');
        updateRecordingPlaybackControls();
    });

    // Waypoint Go Home button functionality
    const waypointGoHomeBtn = document.getElementById('waypoint-go-home-btn');
    if (waypointGoHomeBtn) {
        waypointGoHomeBtn.addEventListener('click', async () => {
            if (!isRobotConnected) {
                addLog("Robot not connected! Cannot send 'Go Home' command.", 'error');
                return;
            }
            
            // Respect speed = 0% as a hard stop
            const speedPercentage = parseFloat(speedSlider.value);
            if (isNaN(speedPercentage) || speedPercentage <= 0) {
                addLog("Speed is 0%. 'Go Home' is blocked.", 'warn');
                return;
            }
            
            addLog("Sending robot to home position (all joints to 0 radians)...", 'info');

            // For 'Go Home', use current speed setting
            const minTime = 2.0;
            const maxTime = 5.0;
            const normalizedSpeed = speedPercentage / 100.0;
            const invertedNormalizedSpeed = 1.0 - normalizedSpeed;
            const timeFromStartSec = minTime + (maxTime - minTime) * invertedNormalizedSpeed;

            const homeJoints = {
                j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 0.0, j6: 0.0,
                time_from_start: timeFromStartSec // Use current speed setting for Go Home
            };

            // Update UI sliders and coordinates immediately
            Object.assign(currentRobotCoords, homeJoints);
            updateManualControlSlidersAndUI(); // Update UI with home position

            const response = await sendCommandToBackend('/api/set_joint_positions', homeJoints, 'Go Home');
            if (response.success) {
                addLog("Robot successfully commanded to home position.", 'success');
            }
        });
    }

    // This is the CORRECT way to call the backend for Python script execution
    if (runScriptBtn) {
        runScriptBtn.addEventListener('click', async () => { // Note the 'async' here
            if (!isRobotConnected) {
                addLog("Robot not connected. Cannot run script.", 'error', scriptConsole);
                return;
            }

            const scriptContent = pythonCodeEditor.textContent; // Get the full script content
            if (!scriptContent.trim()) {
                addLog("Script editor is empty.", 'warn', scriptConsole);
                return;
            }
            scriptConsole.innerHTML = ''; // Clear previous console output
            addLog("Sending Python script to backend for execution...", 'info', scriptConsole);

            // This is the key line: calling your sendCommandToBackend function!
            const result = await sendCommandToBackend('/api/run_script', { script_code: scriptContent }, 'Run Script');
            if (result.success) {
                addLog(`Script execution result: ${result.output || result.message || 'Script finished successfully.'}`, 'success', scriptConsole);
            } else {
                addLog(`Script execution failed: ${result.message || 'Unknown error.'}`, 'error', scriptConsole);
            }
        });
    }

    // Code Editor Syntax Highlighting on input (existing logic)
    if (pythonCodeEditor) {
        pythonCodeEditor.addEventListener('input', highlightPythonCode);
    }


    // --- Initial UI Setup ---
    populateUsbPorts(); // Fetch and populate ports on load
    updateRobotCoordinatesUI(); // Set initial coords to 0
    document.querySelector('.nav-item[data-module="dashboard"]').click(); // Activate Dashboard
    document.querySelector('.tool-btn[data-tool="soft-gripper"]').click(); // Activate soft gripper by default in gripper ops
    addLog("UI Initialized. Ready to connect to Rotatum Chotu.", 'info'); // First log message
    updateConnectButtonState(); // Set initial button state based on port availability
    updateManualControlSlidersAndUI(); // Initialize slider positions and values
    renderWaypoints(); // Initial render of the (empty) waypoint list
    loadRecordingsList(); // Initial load of recordings list (NEW)
    updateTorqueButton(); // Initial state for torque button (NEW)
    updateRecordButton(); // Initial state for record button (NEW)
    updateRecordingPlaybackControls(); // Initial state for recording playback controls (NEW)
    updateDragTeachControlsState(); // Initial state for drag teach controls (NEW)
});
