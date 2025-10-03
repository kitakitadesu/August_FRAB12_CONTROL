/* ROS2 Keyboard Interface JavaScript - WebSocket Only */

class WebSocketKeyboardInterface {
    constructor() {
        this.websocket = null;
        this.connected = false;
        this.keysSentCount = 0;
        this.heldKeys = new Map();
        this.pendingRequests = new Map();
        this.requestId = 0;
        
        // WebSocket configuration - optimized for fast connection
        this.currentHost = window.location.hostname || 'localhost';
        this.websocketUrl = `ws://${this.currentHost}:8765`;
        this.reconnectDelay = 2000; // Reduced from 5000ms
        this.maxReconnectAttempts = 15; // Increased attempts
        this.reconnectAttempts = 0;
        this.connectionTimeout = 5000; // Add connection timeout
        
        // Status tracking
        this.serverStatus = {
            running: false,
            connectedClients: 0,
            uptime: 0,
            version: 'unknown'
        };
        
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.initializeInterface();
        // Connect WebSocket immediately without waiting for window load
        setTimeout(() => this.connectWebSocket(), 100);
    }

    initializeInterface() {
        // Initialize UI elements and status display
        this.updateConnectionMode();
        this.addDebugMessage('Initializing WebSocket keyboard interface...');
        
        // Set initial status
        this.updateStatus({
            running: false,
            connectedClients: 0,
            uptime: 0,
            version: 'connecting...'
        });
    }

    setupEventListeners() {
        // Keyboard events
        document.addEventListener('keydown', (event) => this.handleKeyDown(event));
        document.addEventListener('keyup', (event) => this.handleKeyUp(event));
        
        // Focus management
        document.addEventListener('click', () => document.body.focus());
        
        // Window events
        window.addEventListener('load', () => this.onWindowLoad());
        window.addEventListener('beforeunload', () => this.cleanup());
        
        // Page visibility for connection management
        document.addEventListener('visibilitychange', () => {
            if (document.visibilityState === 'visible' && !this.connected) {
                this.connectWebSocket();
            }
        });
    }

    onWindowLoad() {
        this.addDebugMessage('Page loaded, WebSocket already initializing...');
        
        this.addDebugMessage(`Current host: ${this.currentHost}`);
        this.addDebugMessage(`WebSocket URL: ${this.websocketUrl}`);
        
        // Start periodic status updates once connected
        this.startStatusUpdates();
        
        // If not connected yet, try connecting again
        if (!this.connected && (!this.websocket || this.websocket.readyState === WebSocket.CLOSED)) {
            this.addDebugMessage('Page loaded but WebSocket not connected, attempting immediate connection...');
            this.connectWebSocket();
        }
    }

    generateRequestId() {
        return ++this.requestId;
    }

    async sendWebSocketMessage(message, expectResponse = false) {
        return new Promise((resolve, reject) => {
            if (!this.websocket || this.websocket.readyState !== WebSocket.OPEN) {
                reject(new Error('WebSocket not connected'));
                return;
            }

            try {
                if (expectResponse) {
                    const requestId = this.generateRequestId();
                    message.request_id = requestId;
                    
                    // Store promise for response handling
                    this.pendingRequests.set(requestId, { resolve, reject });
                    
                    // Set timeout for request
                    setTimeout(() => {
                        if (this.pendingRequests.has(requestId)) {
                            this.pendingRequests.delete(requestId);
                            reject(new Error('Request timeout'));
                        }
                    }, 5000);
                }

                this.websocket.send(JSON.stringify(message));
                
                if (!expectResponse) {
                    resolve(true);
                }
            } catch (error) {
                reject(error);
            }
        });
    }

    async testConnection() {
        try {
            this.addDebugMessage('Testing WebSocket connection...');
            
            const response = await this.sendWebSocketMessage({
                type: 'test_connection',
                timestamp: Date.now()
            }, true);
            
            if (response.success) {
                this.updateStatus(true);
                this.addDebugMessage(`Connection test successful (latency: ${response.latency_ms}ms)`);
            } else {
                this.updateStatus(false, response.error);
                this.addDebugMessage(`Connection test failed: ${response.error}`);
            }
            
            return response.success;
        } catch (error) {
            this.updateStatus(false, error.message);
            this.addDebugMessage(`Connection test error: ${error.message}`);
            return false;
        }
    }

    async getStatus() {
        try {
            const response = await this.sendWebSocketMessage({
                type: 'get_status',
                timestamp: Date.now()
            }, true);
            
            if (response.success) {
                this.serverStatus = response.data;
                this.updateStats(response.data);
                return response.data;
            } else {
                this.addDebugMessage(`Status error: ${response.error}`);
                return null;
            }
        } catch (error) {
            this.addDebugMessage(`Status request error: ${error.message}`);
            return null;
        }
    }

    async sendKey(key, keyCode, isHeld = false) {
        try {
            const response = await this.sendWebSocketMessage({
                type: 'send_key',
                key: key,
                key_code: keyCode,
                is_held: isHeld,
                timestamp: Date.now()
            }, true);

            if (response.success || response.published) {
                this.keysSentCount++;
                this.updateKeyDisplay(key, keyCode, true);
                this.updateKeysSentDisplay();
                this.addDebugMessage(`Key sent: '${key}' (${keyCode}) ${isHeld ? '[HELD]' : ''}`);
                return true;
            } else {
                this.addDebugMessage(`Key send failed: ${response.error || 'Unknown error'}`);
                return false;
            }
        } catch (error) {
            this.addDebugMessage(`Error sending key: ${error.message}`);
            return false;
        }
    }

    async sendKeyBatch(keysInput) {
        try {
            const response = await this.sendWebSocketMessage({
                type: 'send_key_batch',
                keys_input: keysInput,
                timestamp: Date.now()
            }, true);

            if (response.success) {
                this.keysSentCount += response.processed_count;
                this.updateKeysSentDisplay();
                this.addDebugMessage(`Batch sent: ${response.processed_count} keys processed`);
                return response;
            } else {
                this.addDebugMessage(`Batch send failed: ${response.error}`);
                return response;
            }
        } catch (error) {
            this.addDebugMessage(`Error sending key batch: ${error.message}`);
            return { success: false, error: error.message };
        }
    }

    handleKeyDown(event) {
        event.preventDefault();

        if (event.repeat) {
            return;
        }

        let key = event.key;
        let keyCode = event.keyCode || event.which;

        key = this.normalizeKeyName(key);

        this.startKeyHold(key, keyCode);
    }

    handleKeyUp(event) {
        event.preventDefault();

        let keyCode = event.keyCode || event.which;
        this.stopKeyHold(keyCode);
    }

    normalizeKeyName(key) {
        const keyMap = {
            'ArrowUp': 'UP',
            'ArrowDown': 'DOWN',
            'ArrowLeft': 'LEFT',
            'ArrowRight': 'RIGHT',
            ' ': 'SPACE',
            'Enter': 'ENTER',
            'Escape': 'ESC',
            'Backspace': 'BACKSPACE',
            'Tab': 'TAB'
        };
        
        return keyMap[key] || key;
    }

    async startKeyHold(key, keyCode) {
        if (this.heldKeys.has(keyCode)) {
            return;
        }

        this.heldKeys.set(keyCode, { key, keyCode });
        
        // Send key down via WebSocket
        try {
            await this.sendWebSocketMessage({
                type: 'key_down',
                key: key,
                key_code: keyCode,
                timestamp: Date.now()
            });

            this.addDebugMessage(`Key hold started: '${key}' (${keyCode})`);
            this.updateKeyDisplay(key, keyCode, true);
            this.keysSentCount++;
            this.updateKeysSentDisplay();
        } catch (error) {
            this.addDebugMessage(`Error starting key hold: ${error.message}`);
        }
    }

    async stopKeyHold(keyCode) {
        if (this.heldKeys.has(keyCode)) {
            const keyInfo = this.heldKeys.get(keyCode);
            this.heldKeys.delete(keyCode);
            
            // Send key up via WebSocket
            try {
                await this.sendWebSocketMessage({
                    type: 'key_up',
                    key: keyInfo.key,
                    key_code: keyCode,
                    timestamp: Date.now()
                });

                this.addDebugMessage(`Key hold stopped: '${keyInfo.key}' (${keyCode})`);
                this.updateKeyDisplay(keyInfo.key, keyCode, false);
                
                setTimeout(() => {
                    const keyDisplay = document.getElementById('keyDisplay');
                    if (keyDisplay && this.heldKeys.size === 0) {
                        keyDisplay.innerHTML = '<p style="border: 2px solid #333; padding: 2rem; margin: 0.5rem 0; min-height: 4rem; display: flex; align-items: center; justify-content: center; text-align: center;"><strong>Press and hold any key...</strong></p>';
                    }
                }, 1000);
            } catch (error) {
                this.addDebugMessage(`Error stopping key hold: ${error.message}`);
            }
        }
    }

    connectWebSocket() {
        // Close existing connection if any
        if (this.websocket && this.websocket.readyState !== WebSocket.CLOSED) {
            if (this.websocket.readyState === WebSocket.CONNECTING) {
                this.addDebugMessage('WebSocket already connecting, skipping...');
                return;
            }
            this.websocket.close();
        }

        try {
            this.addDebugMessage(`🔌 Connecting to WebSocket: ${this.websocketUrl}`);
            this.websocket = new WebSocket(this.websocketUrl);
            
            // Add connection timeout
            const connectionTimer = setTimeout(() => {
                if (this.websocket && this.websocket.readyState === WebSocket.CONNECTING) {
                    this.addDebugMessage('⏰ WebSocket connection timeout, closing and retrying...');
                    this.websocket.close();
                }
            }, this.connectionTimeout);

            this.websocket.onopen = (event) => {
                clearTimeout(connectionTimer); // Clear timeout on successful connection
                this.addDebugMessage('✅ WebSocket connection opened - full communication enabled');
                this.connected = true;
                this.reconnectAttempts = 0;
                this.updateConnectionMode();
                
                // Send initial connection message immediately
                this.sendWebSocketMessage({
                    type: 'client_connected',
                    timestamp: Date.now(),
                    user_agent: navigator.userAgent,
                    client_type: 'web_interface'
                });

                // Test connection and get initial status
                setTimeout(() => {
                    this.testConnection();
                    this.getStatus();
                }, 50);
            };

            this.websocket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    this.handleWebSocketMessage(data);
                } catch (e) {
                    this.addDebugMessage(`WebSocket parse error: ${e.message}`);
                }
            };

            this.websocket.onclose = (event) => {
                clearTimeout(connectionTimer); // Clear timeout on close
                this.addDebugMessage(`❌ WebSocket connection closed (code: ${event.code})`);
                this.connected = false;
                this.websocket = null;
                this.updateConnectionMode();
                
                // Immediate retry for certain close codes, then progressive delay
                if (this.reconnectAttempts < this.maxReconnectAttempts) {
                    this.reconnectAttempts++;
                    let delay;
                    
                    // Immediate retry for server restart (code 1006) or going away (code 1001)
                    if (event.code === 1006 || event.code === 1001) {
                        delay = 100; // Almost immediate
                    } else {
                        delay = Math.min(500 * this.reconnectAttempts, this.reconnectDelay); // Progressive delay
                    }
                    
                    this.addDebugMessage(`🔄 Reconnection attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts} in ${delay}ms`);
                    setTimeout(() => {
                        this.connectWebSocket();
                    }, delay);
                } else {
                    this.addDebugMessage('❌ Max reconnection attempts reached. Please refresh the page.');
                }
            };

            this.websocket.onerror = (error) => {
                this.addDebugMessage(`⚠️ WebSocket connection error: ${error.message || 'Connection failed'}`);
                this.connected = false;
                this.updateConnectionMode();
            };

        } catch (error) {
            this.addDebugMessage(`WebSocket connection failed: ${error.message}`);
        }
    }

    handleWebSocketMessage(data) {
        const messageType = data.type || 'unknown';
        const timestamp = new Date(data.timestamp || Date.now()).toLocaleTimeString();
        
        // Handle responses to pending requests
        if (data.request_id && this.pendingRequests.has(data.request_id)) {
            const { resolve } = this.pendingRequests.get(data.request_id);
            this.pendingRequests.delete(data.request_id);
            resolve(data);
            return;
        }
        
        // Handle broadcast messages and events
        switch (messageType) {
            case 'welcome':
                this.addDebugMessage(`Server: ${data.message}`);
                break;
                
            case 'key_event':
            case 'key_down_event':
            case 'key_up_event':
                this.addDebugMessage(`Server event: ${messageType} '${data.key}'`);
                break;
                
            case 'status_broadcast':
                this.updateStats(data.data);
                break;
                
            case 'error':
                this.addDebugMessage(`Server error: ${data.message || data.error}`);
                break;
                
            default:
                this.addDebugMessage(`Server message (${messageType}): ${JSON.stringify(data).substring(0, 100)}`);
        }
    }

    startStatusUpdates() {
        // Get status every 10 seconds
        setInterval(async () => {
            if (this.connected) {
                await this.getStatus();
            }
        }, 10000);
    }

    updateStatus(isConnected, errorMessage = null) {
        const statusContent = document.getElementById('status-content');
        if (statusContent) {
            if (isConnected) {
                statusContent.innerHTML = '<p><strong>Status:</strong> <span style="color: green;">✅ Connected via WebSocket</span></p>';
            } else {
                const error = errorMessage ? ` (${errorMessage})` : '';
                statusContent.innerHTML = `<p><strong>Status:</strong> <span style="color: red;">❌ Disconnected${error}</span></p>`;
            }
        }
    }

    updateStats(data) {
        const updates = {
            'serverStatus': data.server_running ? 'Running' : 'Stopped',
            'connectedClients': data.connected_clients || 0,
            'serverUptime': data.uptime || 0,
            'keysSent': this.keysSentCount,
            'keysHeld': this.heldKeys.size
        };

        // Add LED state if available
        if (data.ros2 && typeof data.ros2.led_state !== 'undefined') {
            const ledElement = document.getElementById('ledState');
            if (ledElement) {
                const isOn = data.ros2.led_state;
                ledElement.textContent = isOn ? '💡 ON' : '⚫ OFF';
                ledElement.style.color = isOn ? '#00aa00' : '#666666';
                ledElement.style.backgroundColor = isOn ? '#e8f5e8' : '#f5f5f5';
            }
        }

        for (const [elementId, value] of Object.entries(updates)) {
            const element = document.getElementById(elementId);
            if (element) {
                element.textContent = value;
            }
        }
    }

    updateKeyDisplay(key, keyCode, isPressed) {
        const keyDisplay = document.getElementById('keyDisplay');
        if (keyDisplay) {
            if (isPressed) {
                const action = this.getKeyAction(key, keyCode);
                keyDisplay.innerHTML = `
                    <div style="border: 2px solid #0066cc; padding: 1rem; margin: 0.5rem 0; background: #f0f8ff;">
                        <p style="margin: 0; font-size: 1.2em;"><strong>Key: ${key}</strong> (Code: ${keyCode})</p>
                        <p style="margin: 0.5rem 0 0 0; color: #0066cc;">${action}</p>
                    </div>
                `;
            }
        }
    }

    getKeyAction(key, keyCode) {
        const actions = {
            76: '💡 LED Toggle',           // L
            87: '🔼 Moving Forward',      // W
            83: '🔽 Moving Backward',     // S
            65: '↪️ Turning Left',        // A
            68: '↩️ Turning Right',       // D
            70: '🔄 Servo Toggle',        // F
            108: '💡 LED Toggle'          // l
        };
        return actions[keyCode] || '⏹️ Stop Movement';
    }

    updateKeysSentDisplay() {
        const element = document.getElementById('keysSent');
        if (element) {
            element.textContent = this.keysSentCount;
        }
    }

    updateConnectionMode() {
        const modeIndicator = document.getElementById('connectionMode');
        if (modeIndicator) {
            if (this.connected) {
                modeIndicator.textContent = 'WebSocket (Real-time)';
                modeIndicator.style.color = '#008000';
            } else {
                modeIndicator.textContent = 'WebSocket (Connecting...)';
                modeIndicator.style.color = '#FFA500';
            }
        }
    }

    addDebugMessage(message) {
        const timestamp = new Date().toLocaleTimeString();
        const liveEventsDiv = document.getElementById('live-events');
        
        if (liveEventsDiv) {
            const messageElement = document.createElement('div');
            messageElement.style.cssText = 'padding: 0.25rem; border-bottom: 1px solid #eee; font-size: 0.9em;';
            messageElement.innerHTML = `<span style="color: #666;">[${timestamp}]</span> ${message}`;
            
            liveEventsDiv.appendChild(messageElement);
            
            // Keep only last 50 messages
            while (liveEventsDiv.children.length > 50) {
                liveEventsDiv.removeChild(liveEventsDiv.firstChild);
            }
            
            // Auto-scroll to bottom
            liveEventsDiv.scrollTop = liveEventsDiv.scrollHeight;
        }
        
        console.log(`[${timestamp}] ${message}`);
    }

    clearMessages() {
        const liveEventsDiv = document.getElementById('live-events');
        if (liveEventsDiv) {
            liveEventsDiv.innerHTML = '';
        }
        this.addDebugMessage('Debug messages cleared');
    }

    async sendTestKeys() {
        this.addDebugMessage('Sending test key sequence...');
        
        const testKeys = [
            { key: 'W', code: 87, name: 'Forward' },
            { key: 'S', code: 83, name: 'Backward' },
            { key: 'A', code: 65, name: 'Left' },
            { key: 'D', code: 68, name: 'Right' }
        ];
        
        for (const testKey of testKeys) {
            this.addDebugMessage(`Testing ${testKey.name}...`);
            await this.sendKey(testKey.key, testKey.code);
            await new Promise(resolve => setTimeout(resolve, 500));
        }
        
        this.addDebugMessage('Test key sequence completed');
    }

    async toggleLED() {
        this.addDebugMessage('Toggling LED...');
        await this.sendKey('L', 76);
        this.addDebugMessage('LED toggle command sent');
    }

    cleanup() {
        // Clear all held keys
        this.heldKeys.clear();
        
        // Close WebSocket connection
        if (this.websocket) {
            this.websocket.close();
            this.websocket = null;
        }
        
        // Clear pending requests
        this.pendingRequests.clear();
    }

    // Manual form submission handlers (for backward compatibility)
    async submitManualKeyForm(formData) {
        const key = formData.get('key');
        const keyCode = parseInt(formData.get('key_code'));
        const isHeld = formData.get('is_held') === 'true';
        
        const result = await this.sendKey(key, keyCode, isHeld);
        
        return {
            success: result,
            message: result ? `Key '${key}' sent successfully` : `Failed to send key '${key}'`
        };
    }

    async submitBatchKeyForm(formData) {
        const keysInput = formData.get('keys_input');
        
        const result = await this.sendKeyBatch(keysInput);
        
        return {
            success: result.success,
            message: result.success ? 
                `Batch processed: ${result.processed_count} keys` : 
                `Batch failed: ${result.error}`
        };
    }
}

// Global instance
let keyboardInterface;

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    keyboardInterface = new WebSocketKeyboardInterface();
    
    // Make functions available globally for button onclick handlers
    window.testConnection = () => keyboardInterface.testConnection();
    window.getStatus = () => keyboardInterface.getStatus();
    window.clearMessages = () => keyboardInterface.clearMessages();
    window.sendTestKeys = () => keyboardInterface.sendTestKeys();
    window.toggleLED = () => keyboardInterface.toggleLED();
    
    // Handle manual form submissions
    window.submitManualKey = async (event) => {
        event.preventDefault();
        const formData = new FormData(event.target);
        const result = await keyboardInterface.submitManualKeyForm(formData);
        
        const resultDiv = document.getElementById('manual-result');
        if (resultDiv) {
            resultDiv.innerHTML = `<p style="color: ${result.success ? 'green' : 'red'};">${result.message}</p>`;
        }
    };
    
    window.submitBatchKey = async (event) => {
        event.preventDefault();
        const formData = new FormData(event.target);
        const result = await keyboardInterface.submitBatchKeyForm(formData);
        
        const resultDiv = document.getElementById('batch-result');
        if (resultDiv) {
            resultDiv.innerHTML = `<p style="color: ${result.success ? 'green' : 'red'};">${result.message}</p>`;
        }
    };
});