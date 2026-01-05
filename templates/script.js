// ============================================================================
// RASPBOT V2 - COMPLETE SCRIPT WITH SSE REAL-TIME STREAMING
// âœ… SSE client for 10Hz updates
// âœ… Real-time distance, servo, mode updates
// âœ… Keyboard control with state tracking
// ============================================================================




const ROBOT_API = 'http://192.168.100.14:5000';




let currentSpeed = 150;
let currentPan = 90;
let currentTilt = 45;




// Keyboard state tracking
let keysPressed = {};
let lastMotorCommand = 0;
const MOTOR_DEBOUNCE = 50; // ms




// Chat focus tracking
let chatInputFocused = false;




// SSE connection
let eventSource = null;
let sseConnected = false;
let reconnectInterval = null;




// ============================================================================
// INITIALIZATION
// ============================================================================




window.addEventListener('load', () => {
    console.log('ðŸ¤– Raspbot V2 - SSE Enabled');
   
    initCamera();
    setupKeyboard();
    setupChatFocus();
    startSSE();  // Start SSE connection
   
    addMsg('bot', 'âœ… System ready!');
    addMsg('bot', 'ðŸ“º Keyboard: W/A/S/D move, Q/E rotate, SPACE stop');
    addMsg('bot', 'ðŸ”— Go-Around: W+A, W+D, S+A, S+D');
    addMsg('bot', 'ðŸ“¡ Real-time streaming enabled');
});




// Stop SSE when page unloads
window.addEventListener('beforeunload', () => {
    stopSSE();
});




// ============================================================================
// SSE CONNECTION MANAGEMENT
// ============================================================================




function startSSE() {
    if (eventSource && sseConnected) {
        console.log('SSE already connected');
        return;
    }
   
    console.log('ðŸ“¡ Starting SSE connection...');
   
    const sseUrl = `${ROBOT_API}/api/autonomous/stream`;
    eventSource = new EventSource(sseUrl);
   
    // Connection opened
    eventSource.onopen = function(event) {
        console.log('âœ… SSE connection established');
        sseConnected = true;
        updateConnectionStatus(true);
       
        if (reconnectInterval) {
            clearInterval(reconnectInterval);
            reconnectInterval = null;
        }
    };
   
    // Receiving messages
    eventSource.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            handleSSEData(data);
        } catch (error) {
            console.error('Error parsing SSE data:', error);
        }
    };
   
    // Connection error
    eventSource.onerror = function(event) {
        console.error('âŒ SSE connection error');
        sseConnected = false;
        updateConnectionStatus(false);
       
        if (eventSource) {
            eventSource.close();
            eventSource = null;
        }
       
        // Try to reconnect after 5 seconds
        if (!reconnectInterval) {
            reconnectInterval = setInterval(() => {
                console.log('Attempting to reconnect SSE...');
                startSSE();
            }, 5000);
        }
    };
}




function stopSSE() {
    console.log('Stopping SSE connection...');
   
    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }
   
    sseConnected = false;
    updateConnectionStatus(false);
   
    if (reconnectInterval) {
        clearInterval(reconnectInterval);
        reconnectInterval = null;
    }
}




// ============================================================================
// SSE DATA HANDLING
// ============================================================================




function handleSSEData(data) {
    // 1. Update distance gauge
    if (data.distance !== undefined) {
        updateDistanceGauge(data.distance);
    }
   
    // 2. Update servo positions
    if (data.pan !== undefined && data.tilt !== undefined) {
        updateServoDisplay(data.pan, data.tilt);
    }
   
    // 3. Update robot mode/status
    if (data.mode !== undefined) {
        updateRobotMode(data.mode);
    }
   
    // 4. Update line sensors (if displayed)
    if (data.line_sensors !== undefined) {
        updateLineSensors(data.line_sensors);
    }
   
    // 5. Update last command
    if (data.last_command !== undefined) {
        updateLastCommand(data.last_command);
    }
   
    // 6. Update speed indicator
    if (data.speed !== undefined) {
        updateSpeedIndicator(data.speed);
    }
   
    // 7. Update autonomous status panel
    if (data.state !== undefined) {
        const el = document.getElementById('robot-state');
        if (el) el.textContent = data.state;
    }
    if (data.distance !== undefined) {
        const el = document.getElementById('robot-distance');
        if (el) el.textContent = data.distance.toFixed(1) + ' cm';
    }
    if (data.pan !== undefined) {
        const el = document.getElementById('robot-pan');
        if (el) el.textContent = data.pan + 'Â°';
    }
    if (data.tilt !== undefined) {
        const el = document.getElementById('robot-tilt');
        if (el) el.textContent = data.tilt + 'Â°';
    }
}




// ============================================================================
// UI UPDATE FUNCTIONS
// ============================================================================




function updateDistanceGauge(distance) {
    const distResult = document.getElementById('dist-result');
    const distBar = document.getElementById('dist-bar');
    const distStatus = document.getElementById('dist-status');
   
    if (distResult) {
        distResult.textContent = distance.toFixed(1);
    }
   
    // Color gauge based on distance
    let color = '#22c55e'; // Green
    let status = 'âœ… SAFE';
    let width = Math.min((distance / 100) * 100, 100);
   
    if (distance < 50) {
        color = '#f59e0b'; // Yellow
        status = 'âš ï¸ CAUTION';
    }
    if (distance < 20) {
        color = '#ef4444'; // Red
        status = 'ðŸš¨ DANGER';
    }
   
    if (distResult) distResult.style.color = color;
    if (distBar) {
        distBar.style.background = color;
        distBar.style.width = width + '%';
    }
    if (distStatus) distStatus.textContent = status;
}




function updateServoDisplay(pan, tilt) {
    // Update camera display
    const panEl = document.getElementById('cam-pan');
    const tiltEl = document.getElementById('cam-tilt');
   
    if (panEl) panEl.textContent = pan + 'Â°';
    if (tiltEl) tiltEl.textContent = tilt + 'Â°';
   
    // Update status panel
    const panDisplay = document.getElementById('pan-value');
    const tiltDisplay = document.getElementById('tilt-value');
   
    if (panDisplay) panDisplay.textContent = `Pan: ${pan}Â°`;
    if (tiltDisplay) tiltDisplay.textContent = `Tilt: ${tilt}Â°`;
   
    // Sync with local state
    currentPan = pan;
    currentTilt = tilt;
}




function updateRobotMode(mode) {
    const modeDisplay = document.getElementById('robot-mode');
    if (!modeDisplay) return;
   
    modeDisplay.textContent = mode;
   
    // Color coding
    const colors = {
        'IDLE': '#6b7280',
        'MANUAL': '#3b82f6',
        'PATROL': '#10b981',
        'OBSTACLE_AVOIDANCE': '#f59e0b',
        'LINE_FOLLOWING': '#8b5cf6',
        'FACE_TRACKING': '#ec4899'
    };
   
    modeDisplay.style.color = colors[mode] || '#6b7280';
}




function updateLineSensors(sensors) {
    // sensors = [L1, L2, R1, R2]
    for (let i = 0; i < sensors.length; i++) {
        const sensorEl = document.getElementById(`line-sensor-${i}`);
        if (sensorEl) {
            sensorEl.classList.toggle('active', sensors[i] === 1);
        }
    }
}




function updateLastCommand(command) {
    const cmdDisplay = document.getElementById('last-command');
    if (cmdDisplay) {
        cmdDisplay.textContent = `Last: ${command}`;
    }
}




function updateSpeedIndicator(speed) {
    const speedDisplay = document.getElementById('speed-value');
    if (!speedDisplay) return;
   
    speedDisplay.textContent = `${speed}`;
   
    // Speed bar (optional)
    const speedBar = document.getElementById('speed-bar');
    if (speedBar) {
        const percentage = (speed / 255) * 100;
        speedBar.style.width = `${percentage}%`;
    }
}




function updateConnectionStatus(connected) {
    const statusEl = document.getElementById('sse-status');
    if (!statusEl) return;
   
    if (connected) {
        statusEl.textContent = 'ðŸŸ¢ Live';
        statusEl.style.color = '#10b981';
    } else {
        statusEl.textContent = 'ðŸ”´ Offline';
        statusEl.style.color = '#ef4444';
    }
}




// ============================================================================
// CAMERA
// ============================================================================




function initCamera() {
    const cameraFeed = document.getElementById('camera-feed');
    const cameraPlaceholder = document.getElementById('camera-placeholder');
   
    if (!cameraFeed) return;
   
    const streamUrl = `${ROBOT_API}/video_feed`;
    cameraFeed.src = streamUrl;
    cameraFeed.style.display = 'block';
    cameraPlaceholder.style.display = 'none';
   
    cameraFeed.onload = () => {
        console.log('âœ… Camera connected');
        addMsg('bot', 'ðŸ“¹ Camera stream OK');
    };
   
    cameraFeed.onerror = () => {
        console.warn('âš ï¸ Camera error');
        cameraFeed.style.display = 'none';
        cameraPlaceholder.style.display = 'flex';
    };
}




// ============================================================================
// MOTOR COMMANDS (WITH DEBOUNCE)
// ============================================================================




function sendMotorCommand(action, speed = currentSpeed) {
    const now = Date.now();
    if (now - lastMotorCommand < MOTOR_DEBOUNCE) {
        return; // Skip if too soon
    }
    lastMotorCommand = now;
   
    console.log(`[MOTOR] ${action} @ ${speed}`);
   
    fetch(`${ROBOT_API}/api/command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            action: action,
            speed: speed
        })
    })
    .then(r => r.json())
    .catch(e => console.error('[ERROR]', e));
}




function sendMotorAngle(angle, speed = currentSpeed) {
    const now = Date.now();
    if (now - lastMotorCommand < MOTOR_DEBOUNCE) {
        return;
    }
    lastMotorCommand = now;
   
    console.log(`[MOTOR] GO_ANGLE ${angle}Â° @ ${speed}`);
   
    fetch(`${ROBOT_API}/api/command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            action: 'go_angle',
            angle: angle,
            speed: speed
        })
    })
    .catch(e => console.error('[ERROR]', e));
}




function setSpeed(val) {
    currentSpeed = parseInt(val);
    document.getElementById('speed-val').textContent = currentSpeed;
    console.log(`âš™ï¸ Speed: ${currentSpeed}`);
}




// ============================================================================
// SERVO CONTROL
// ============================================================================




function panCam(panDelta, tiltDelta) {
    if (panDelta !== 0) {
        currentPan -= panDelta;
        currentPan = Math.max(0, Math.min(180, currentPan));
    }
   
    if (tiltDelta !== 0) {
        currentTilt += tiltDelta;
        currentTilt = Math.max(10, Math.min(90, currentTilt));
    }
   
    if (panDelta === 0 && tiltDelta === 0) {
        currentPan = 90;
        currentTilt = 45;
    }
   
    updateServo();
}




function updateServo() {
    document.getElementById('pan-val').textContent = currentPan + 'Â°';
    document.getElementById('tilt-val').textContent = currentTilt + 'Â°';
   
    fetch(`${ROBOT_API}/api/servo`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            pan: currentPan,
            tilt: currentTilt
        })
    })
    .catch(e => console.error('[ERROR]', e));
}




// ============================================================================
// AUTONOMOUS MODES
// ============================================================================




function startFindFace() {
    addMsg('bot', 'ðŸ”Ž FIND FACE started...');
   
    fetch(`${ROBOT_API}/api/autonomous/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: 'face_tracking' })
    })
    .then(r => r.json())
    .then(d => {
        console.log('âœ… FIND FACE active');
        addMsg('bot', 'ðŸ¤– Searching for faces...');
    })
    .catch(e => {
        console.error('[ERROR]', e);
        addMsg('bot', `âŒ Error: ${e.message}`);
    });
}




function startAutoObstacle() {
    addMsg('bot', 'ðŸš¨ AUTO OBSTACLE started...');
   
    fetch(`${ROBOT_API}/api/autonomous/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: 'obstacle_avoidance' })
    })
    .then(r => r.json())
    .then(d => {
        console.log('âœ… AUTO OBSTACLE active');
        addMsg('bot', 'ðŸ¤– Moving and avoiding obstacles...');
    })
    .catch(e => {
        console.error('[ERROR]', e);
        addMsg('bot', `âŒ Error: ${e.message}`);
    });
}




function startFaceTracking() {
    addMsg('bot', 'ðŸ‘¤ FACE TRACKING started...');
   
    fetch(`${ROBOT_API}/api/autonomous/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: 'face_tracking' })
    })
    .then(r => r.json())
    .then(d => {
        console.log('âœ… FACE TRACKING active');
        addMsg('bot', 'ðŸ¤– Tracking faces...');
    })
    .catch(e => {
        console.error('[ERROR]', e);
        addMsg('bot', `âŒ Error: ${e.message}`);
    });
}




function stopAutonomous() {
    addMsg('bot', 'â¹ï¸ Autonomous stopped');
   
    fetch(`${ROBOT_API}/api/autonomous/stop`, {
        method: 'POST'
    })
    .catch(e => console.error('[ERROR]', e));
}




// ============================================================================
// KEYBOARD CONTROL (WITH STATE TRACKING)
// ============================================================================




function setupKeyboard() {
    document.addEventListener('keydown', (e) => {
        // Skip if chat input focused
        if (chatInputFocused) return;
       
        const key = e.key.toLowerCase();
       
        // Prevent default for movement keys
        if (['w', 'a', 's', 'd', 'q', 'e', ' '].includes(key)) {
            e.preventDefault();
        }
       
        // Update key state
        keysPressed[key] = true;
       
        // Single key commands
        if (key === 'w') {
            sendMotorCommand('forward');
        }
        else if (key === 's') {
            sendMotorCommand('backward');
        }
        else if (key === 'a') {
            sendMotorCommand('left');
        }
        else if (key === 'd') {
            sendMotorCommand('right');
        }
        else if (key === 'q') {
            sendMotorCommand('rotate_left');
        }
        else if (key === 'e') {
            sendMotorCommand('rotate_right');
        }
        else if (key === ' ') {
            sendMotorCommand('stop');
        }
       
        // Camera
        else if (key === 'arrowup') {
            e.preventDefault();
            panCam(0, 10);
        }
        else if (key === 'arrowdown') {
            e.preventDefault();
            panCam(0, -10);
        }
        else if (key === 'arrowleft') {
            e.preventDefault();
            panCam(-10, 0);
        }
        else if (key === 'arrowright') {
            e.preventDefault();
            panCam(10, 0);
        }
        else if (key === 'r') {
            e.preventDefault();
            panCam(0, 0);
        }
       
        // Check for Go-Around combinations
        checkGoAround();
    });
   
    document.addEventListener('keyup', (e) => {
        const key = e.key.toLowerCase();
        keysPressed[key] = false;
    });
   
    console.log('âŒ¨ï¸ Keyboard setup complete');
}




function checkGoAround() {
    const w = keysPressed['w'];
    const s = keysPressed['s'];
    const a = keysPressed['a'];
    const d = keysPressed['d'];
   
    if (w && a) {
        sendMotorAngle(45);  // Forward-Left
    }
    else if (w && d) {
        sendMotorAngle(315); // Forward-Right
    }
    else if (s && a) {
        sendMotorAngle(225); // Backward-Left
    }
    else if (s && d) {
        sendMotorAngle(135); // Backward-Right
    }
}




// ============================================================================
// CHAT FOCUS
// ============================================================================




function setupChatFocus() {
    const chatInput = document.getElementById('cmd-input');
   
    if (!chatInput) return;
   
    chatInput.addEventListener('focus', () => {
        chatInputFocused = true;
        console.log('ðŸ’¬ Chat focused - Keyboard disabled');
    });
   
    chatInput.addEventListener('blur', () => {
        chatInputFocused = false;
        console.log('ðŸ’¬ Chat unfocused - Keyboard enabled');
    });
}




// ============================================================================
// CHAT
// ============================================================================




function enterCmd(event) {
    if (event.key === 'Enter') {
        sendCmd();
    }
}




function sendCmd() {
    const input = document.getElementById('cmd-input');
    const message = input.value.trim();
   
    if (!message) return;
   
    console.log('[SEND]', message);
    addMsg('user', message);
    input.value = '';
   
    addMsg('bot', 'ðŸ’¬ Message received');
}




function addMsg(role, text) {
    const messagesDiv = document.getElementById('ai-messages');  // â† FIXED!
    const msgDiv = document.createElement('div');
    msgDiv.className = `ai-msg ${role}`;  // â† FIXED!
    msgDiv.textContent = text;
   
    messagesDiv.appendChild(msgDiv);
    messagesDiv.scrollTop = messagesDiv.scrollHeight;
   
    return msgDiv;
}




function clearChat() {
    const messagesDiv = document.getElementById('ai-messages');  // â† FIXED!
    messagesDiv.innerHTML = '';
    addMsg('bot', 'ðŸ’¬ Chat cleared');
}




// ============================================================================
// MODE SWITCHING
// ============================================================================




function setMode(mode) {
    const manualMode = document.getElementById('manual-mode');
    const autoMode = document.getElementById('auto-mode');
    const buttons = document.querySelectorAll('.tab-btn');
   
    buttons.forEach(btn => btn.classList.remove('active'));
   
    if (mode === 'manual') {
        manualMode.classList.remove('hidden');
        autoMode.classList.add('hidden');
        buttons[0].classList.add('active');
        stopAutonomous();
    } else {
        manualMode.classList.add('hidden');
        autoMode.classList.remove('hidden');
        buttons[1].classList.add('active');
    }
   
    console.log(`ðŸ”€ Mode: ${mode}`);
}




// ============================================================================
// BUTTON COMMANDS
// ============================================================================




function cmd(action) {
    sendMotorCommand(action);
}




function goAround(direction) {
    let angle = 45;
    if (direction === 'forward_left') angle = 45;
    else if (direction === 'forward_right') angle = 315;
    else if (direction === 'backward_left') angle = 225;
    else if (direction === 'backward_right') angle = 135;
   
    sendMotorAngle(angle);
}




function checkFace() {
    addMsg('bot', 'ðŸ‘¤ Face detection...');
}




function checkObstacle() {
    addMsg('bot', 'ðŸš« Obstacle check...');
}




function readLine() {
    addMsg('bot', 'ðŸ“ Line sensor read...');
}
// ============================================================================
// MODE 3 - AI-ASSISTED CONTROL
// Added: 2025-12-28
// Features: Natural language commands, PC AI integration, real-time chat
// ============================================================================




let currentRobotMode = 'manual';  // 'manual', 'auto', 'ai'
let aiCommandsCount = 0;
let pcServerConnected = false;
let aiCheckInterval = null;




// Check PC server status
async function checkPCServer() {
    try {
        const response = await fetch(`${ROBOT_API}/api/mode/status`);
        if (response.ok) {
            const data = await response.json();
            pcServerConnected = data.pc_available || false;
           
            // Update AI connection status
            const statusEl = document.getElementById('ai-connection-status');
            if (statusEl) {
                if (pcServerConnected) {
                    statusEl.classList.add('connected');
                    statusEl.innerHTML = '<div class="status-dot"></div><span>PC Server: âœ… Connected (' + (data.pc_ip || 'Unknown IP') + ')</span>';
                } else {
                    statusEl.classList.remove('connected');
                    statusEl.innerHTML = '<div class="status-dot"></div><span>PC Server: âš ï¸ Not available - Start PC server</span>';
                }
            }
           
            // Update MODE 3 status panel
            const pcStatusEl = document.getElementById('pc-server-status');
            const aiModelEl = document.getElementById('ai-model-name');
           
            if (pcStatusEl) {
                pcStatusEl.textContent = pcServerConnected ? 'âœ… Online' : 'âŒ Offline';
                pcStatusEl.style.color = pcServerConnected ? '#22c55e' : '#ef4444';
            }
           
            if (aiModelEl) {
                aiModelEl.textContent = pcServerConnected ? 'qwen2.5vl:7b' : '-';
            }
           
            return pcServerConnected;
        }
    } catch (error) {
        console.error('[MODE3] Error checking PC server:', error);
        pcServerConnected = false;
       
        const statusEl = document.getElementById('ai-connection-status');
        if (statusEl) {
            statusEl.classList.remove('connected');
            statusEl.innerHTML = '<div class="status-dot"></div><span>PC Server: âŒ Connection Error</span>';
        }
    }
    return false;
}




// Set robot mode (manual/auto/ai)
async function setRobotMode(mode) {
    try {
        const response = await fetch(`${ROBOT_API}/api/mode/set`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mode: mode })
        });
       
        if (response.ok) {
            const data = await response.json();
            currentRobotMode = data.mode;
            console.log(`[MODE3] âœ… Switched to: ${currentRobotMode}`);
           
            // Show/hide MODE 3 status panel
            const mode3Panel = document.getElementById('mode3-status-panel');
            if (mode3Panel) {
                mode3Panel.style.display = (mode === 'ai') ? 'block' : 'none';
            }
           
            // Add notification to chat
            addMsg('bot', `ðŸ”„ Mode switched to: ${mode.toUpperCase()}`);
           
            return true;
        } else {
            console.error('[MODE3] Failed to set mode:', response.status);
            addMsg('bot', `âŒ Failed to switch mode`);
        }
    } catch (error) {
        console.error('[MODE3] Error setting mode:', error);
        addMsg('bot', `âŒ Error: ${error.message}`);
    }
    return false;
}




// Send AI command
// PATCH FOR script.js
// Replace sendAICommand function (lines 846-931)
// WITH THIS COMPLETE FIXED VERSION:


// Send AI command WITH AUTO CAMERA CAPTURE
async function sendAICommand() {
    const input = document.getElementById('ai-input');
    if (!input) return;
   
    const message = input.value.trim();
    if (!message) return;
   
    // Check PC server first
    if (!pcServerConnected) {
        addAIMsg('system', '⚠️ PC Server not connected!');
        return;
    }
   
    addAIMsg('user', message);
    input.value = '';
   
    // Disable input
    input.disabled = true;
    const sendBtn = document.querySelector('.ai-send-btn');
    if (sendBtn) {
        sendBtn.disabled = true;
        sendBtn.textContent = 'Sending...';
    }
   
    try {
        // ✅ VISION QUERY DETECTION (GIỐNG CŨ)
        let imageData = null;
       
        const visionKeywords = [
            'see', 'camera', 'color', 'colour', 'image', 'picture',
            'what do', 'describe', 'identify', 'detect', 'analyze',
            'look at', 'show me', 'visible', 'reveal', 'what main',
            'what is', 'what are', 'tell me what', 'colors', 'visible'
        ];
        const isVisionQuery = visionKeywords.some(kw => message.toLowerCase().includes(kw));
       
        // ✅ MỚI - DÙNG SNAPSHOT API THAY VÌ CANVAS
        if (isVisionQuery) {
            try {
                console.log('[AI-CHAT] 📸 Vision query detected, fetching snapshot...');
               
                // Show warning about processing time
                addAIMsg('system', '⏳ Vision processing may take 20-30 seconds...');
                addAIMsg('system', '📸 Capturing camera snapshot...');
               
                // Update button text for vision queries
                if (sendBtn) {
                    sendBtn.textContent = 'Processing vision (20-30s)...';
                }
               
                // ✅ FETCH SNAPSHOT FROM PI API (NO CORS ISSUES!)
                const snapshotResponse = await fetch(`${ROBOT_API}/api/camera/snapshot`, {
                    method: 'GET',
                    headers: { 'Accept': 'application/json' }
                });
               
                if (snapshotResponse.ok) {
                    const snapshotData = await snapshotResponse.json();
                   
                    if (snapshotData.status === 'ok' && snapshotData.image) {
                        imageData = snapshotData.image;
                        const sizeKB = Math.round(snapshotData.image.length / 1024);
                       
                        console.log(`[AI-CHAT] ✅ Snapshot captured: ${sizeKB}KB (${snapshotData.width}x${snapshotData.height})`);
                        addAIMsg('system', `✅ Snapshot ready: ${sizeKB}KB`);
                        addAIMsg('system', '📤 Sending to AI for analysis...');
                    } else {
                        console.warn('[AI-CHAT] ⚠️ Snapshot failed:', snapshotData.message);
                        addAIMsg('system', '⚠️ Camera snapshot failed, sending text only');
                    }
                } else {
                    console.warn('[AI-CHAT] ⚠️ Snapshot request failed:', snapshotResponse.status);
                    addAIMsg('system', '⚠️ Camera not available, sending text only');
                }
            } catch (err) {
                console.error('[AI-CHAT] ⚠️ Snapshot error:', err);
                addAIMsg('system', '⚠️ Camera capture failed, sending text only');
            }
        }
       
        // ✅ PREPARE REQUEST (GIỐNG CŨ)
        const requestBody = { message: message };
        if (imageData) {
            requestBody.image = imageData;
            console.log('[AI-CHAT] 📤 Sending message WITH image');
        } else {
            console.log('[AI-CHAT] 📤 Sending message (text only)');
        }
       
        // ✅ SEND TO PC AI (GIỐNG CŨ)
        const response = await fetch(`${ROBOT_API}/api/ai/chat`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(requestBody)
        });
       
        if (response.ok) {
            const data = await response.json();
           
            if (data.ai_response) {
                addAIMsg('bot', data.ai_response);
            }
           
            if (data.command && data.command.action) {
                const action = data.command.action;
                if (action !== 'unknown' && action !== 'none') {
                    addAIMsg('system', `🤖 Executing: ${action.replace(/_/g, ' ')}`);
                   
                    aiCommandsCount++;
                    const counterEl = document.getElementById('ai-commands-count');
                    if (counterEl) {
                        counterEl.textContent = aiCommandsCount;
                    }
                }
            }
           
            addMsg('user', `💬 ${message}`);
            if (data.ai_response) {
                addMsg('bot', `🧠 AI: ${data.ai_response}`);
            }
           
        } else {
            const errorData = await response.json().catch(() => ({}));
            const errorMsg = errorData.message || 'Unknown error';
            addAIMsg('system', `❌ Error: ${errorMsg}`);
            addMsg('bot', `❌ AI Error: ${errorMsg}`);
        }
    } catch (error) {
        console.error('[MODE3] AI command error:', error);
        addAIMsg('system', `❌ Network error: ${error.message}`);
        addMsg('bot', `❌ Connection error: ${error.message}`);
    } finally {
        // Re-enable input
        input.disabled = false;
        if (sendBtn) {
            sendBtn.disabled = false;
            sendBtn.textContent = 'Send';
        }
        input.focus();
    }
}


// AI quick command buttons
async function aiQuickCommand(command) {
    const input = document.getElementById('ai-input');
    if (input) {
        input.value = command;
        await sendAICommand();
    }
}




// Add message to AI chat
function addAIMsg(type, text) {
    const messagesDiv = document.getElementById('ai-messages');
    if (!messagesDiv) return;
   
    const msgDiv = document.createElement('div');
    msgDiv.className = `ai-msg ${type}`;
    msgDiv.textContent = text;
   
    messagesDiv.appendChild(msgDiv);
   
    // Auto-scroll to bottom
    messagesDiv.scrollTop = messagesDiv.scrollHeight;
   
    // Keep only last 50 messages
    while (messagesDiv.children.length > 50) {
        messagesDiv.removeChild(messagesDiv.firstChild);
    }
}




// Enter key for AI chat
function aiEnterCommand(event) {
    if (event.key === 'Enter') {
        event.preventDefault();
        sendAICommand();
    }
}




// Enhanced setMode function with MODE 3 support
const originalSetMode = window.setMode;
window.setMode = async function(mode) {
    console.log(`[MODE3] setMode called: ${mode}`);
   
    // Update tab buttons
    const tabs = document.querySelectorAll('.tab-btn');
    tabs.forEach((tab, index) => {
        if ((mode === 'manual' && index === 0) ||
            (mode === 'auto' && index === 1) ||
            (mode === 'ai' && index === 2)) {
            tab.classList.add('active');
        } else {
            tab.classList.remove('active');
        }
    });
   
    // Show/hide mode content
    const manualMode = document.getElementById('manual-mode');
    const autoMode = document.getElementById('auto-mode');
    const aiMode = document.getElementById('ai-mode');
   
    if (manualMode) manualMode.classList.toggle('hidden', mode !== 'manual');
    if (autoMode) autoMode.classList.toggle('hidden', mode !== 'auto');
    if (aiMode) aiMode.classList.toggle('hidden', mode !== 'ai');
   
    // MODE 3 specific actions
    if (mode === 'ai') {
        // Check PC server status
        await checkPCServer();
       
        // Set robot backend mode
        await setRobotMode('ai');
       
        // Show welcome message (only first time)
        if (aiCommandsCount === 0) {
            setTimeout(() => {
                addAIMsg('bot', 'ðŸŽ¯ MODE 3 activated! Try saying things like:');
                setTimeout(() => addAIMsg('system', 'ðŸ’¡ "go forward"'), 300);
                setTimeout(() => addAIMsg('system', 'ðŸ’¡ "find a face"'), 600);
                setTimeout(() => addAIMsg('system', 'ðŸ’¡ "start patrol"'), 900);
            }, 500);
        }
       
        // Focus AI input
        const aiInput = document.getElementById('ai-input');
        if (aiInput) {
            setTimeout(() => aiInput.focus(), 600);
        }
       
    } else if (mode === 'manual') {
        await setRobotMode('manual');
    } else if (mode === 'auto') {
        await setRobotMode('autonomous');
    }
}




// Setup AI input focus tracking
window.addEventListener('load', () => {
    const aiInput = document.getElementById('ai-input');
    if (aiInput) {
        aiInput.addEventListener('focus', () => {
            chatInputFocused = true;
        });
        aiInput.addEventListener('blur', () => {
            chatInputFocused = false;
        });
    }
});




// Check PC server status periodically (every 15 seconds)
setInterval(() => {
    if (currentRobotMode === 'ai') {
        checkPCServer();
    }
}, 15000);




// Initial PC check after page load
setTimeout(() => {
    checkPCServer();
}, 3000);




console.log('ðŸ§  MODE 3 - AI-Assisted Control Loaded!');
console.log('ðŸ“¡ Natural language commands enabled');
console.log('ðŸ¤– PC AI integration ready');








