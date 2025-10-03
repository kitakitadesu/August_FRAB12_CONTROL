/* ROS2 Keyboard Interface JavaScript */

class KeyboardInterface {
    constructor() {
        this.connected = false;
        this.websocket = null;
        this.keysSentCount = 0;
        this.websocketEnabled = true;
        this.heldKeys = new Map(); // Track held keys
        this.keyIntervals = new Map(); // Track intervals for held keys
        
        // Dynamic API base URL based on current hostname
        this.currentHost = window.location.hostname;
        this.apiBase = `http://${this.currentHost}:5000`;
        this.websocketUrl = `ws://${this.currentHost}:8765`;
        
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.initializeInterface();
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
    }

    onWindowLoad() {
        this.addDebugMessage('Page loaded, testing REST API connection...');
        
        // Update URL displays
        this.updateUrlDisplays();
        
        this.addDebugMessage(`Current host: ${this.currentHost}`);
        this.addDebugMessage(`Using API base URL: ${this.apiBase}`);
        this.addDebugMessage(`Web interface: http://${this.currentHost}:8000`);
        
        this.testConnection();
        
        // Start WebSocket if enabled
        if (this.websocketEnabled) {
            this.connectWebSocket();
        }
        
        document.body.focus(); // Focus the body to receive keyboard events
        
        // Update status periodically
        setInterval(() => this.getStatus(), 10000); // Every 10 seconds
    }

    updateUrlDisplays() {
        const elements = {
            'currentHost': this.currentHost,
            'apiUrl': this.apiBase,
            'webUrl': `http://${this.currentHost}:8000`
        };
        
        Object.entries(elements).forEach(([id, value]) => {
            const element = document.getElementById(id);
            if (element) element.textContent = value;
        });
    }

    initializeInterface() {
        // Set up HTMX event listeners
        document.body.addEventListener('htmx:beforeRequest', (event) => {
            this.addDebugMessage(`HTMX request: ${event.detail.requestConfig.verb} ${event.detail.requestConfig.path}`);
        });

        document.body.addEventListener('htmx:responseError', (event) => {
            this.addDebugMessage(`HTMX error: ${event.detail.xhr.status} ${event.detail.xhr.statusText}`);
        });

        // Set up form submissions
        this.setupFormHandlers();
    }

    setupFormHandlers() {
        // Handle manual key input form
        const keyForm = document.getElementById('manualKeyForm');
        if (keyForm) {
            keyForm.addEventListener('htmx:afterRequest', (event) => {
                if (event.detail.successful) {
                    this.keysSentCount++;
                    this.updateStats();
                }
            });
        }
    }

    addDebugMessage(message) {
        const debugMessages = document.getElementById('debugMessages');
        if (debugMessages) {
            const timestamp = new Date().toLocaleTimeString();
            const newMessage = `[${timestamp}] ${message}\n`;
            debugMessages.innerHTML += newMessage;
            debugMessages.scrollTop = debugMessages.scrollHeight;
        }
        console.log(`[${new Date().toLocaleTimeString()}] ${message}`);
    }

    updateStatus(isConnected, message = '') {
        const statusDiv = document.getElementById('status');
        if (!statusDiv) return;
        
        this.connected = isConnected;
        if (isConnected) {
            statusDiv.textContent = 'Connected to REST API server';
            statusDiv.className = 'status connected';
            this.addDebugMessage('REST API connection established');
        } else {
            statusDiv.textContent = `Disconnected from REST API server ${message}`;
            statusDiv.className = 'status disconnected';
            this.addDebugMessage(`REST API disconnected: ${message}`);
        }
    }

    updateStats(data) {
        if (data) {
            const elements = {
                'serverStatus': data.server_running ? 'Running' : 'Stopped',
                'connectedClients': data.connected_clients || 0,
                'serverUptime': data.uptime ? `${data.uptime}s` : '0s'
            };
            
            Object.entries(elements).forEach(([id, value]) => {
                const element = document.getElementById(id);
                if (element) element.textContent = value;
            });
        }
        
        const keysSentElement = document.getElementById('keysSent');
        const keysHeldElement = document.getElementById('keysHeld');
        
        if (keysSentElement) keysSentElement.textContent = this.keysSentCount;
        if (keysHeldElement) keysHeldElement.textContent = this.heldKeys.size;
    }

    async testConnection() {
        try {
            const response = await fetch(`${this.apiBase}/api/status`);
            const result = await response.json();
            
            if (result.success) {
                this.updateStatus(true);
                this.updateStats(result.data);
                this.addDebugMessage('Connection test successful');
            } else {
                this.updateStatus(false, result.error);
                this.addDebugMessage(`Connection test failed: ${result.error}`);
            }
        } catch (error) {
            this.updateStatus(false, error.message);
            this.addDebugMessage(`Connection test error: ${error.message}`);
        }
    }

    async getStatus() {
        try {
            const response = await fetch(`${this.apiBase}/api/status`);
            const result = await response.json();
            
            if (result.success) {
                this.updateStats(result.data);
            } else {
                this.addDebugMessage(`Status error: ${result.error}`);
            }
        } catch (error) {
            this.addDebugMessage(`Status request error: ${error.message}`);
        }
    }

    async sendKey(key, keyCode, isHeld = false) {
        try {
            const response = await fetch(`${this.apiBase}/api/key`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    key: key,
                    key_code: keyCode,
                    timestamp: Date.now(),
                    is_held: isHeld
                })
            });

            const result = await response.json();

            if (result.success) {
                this.keysSentCount++;
                this.updateStats();

                // Update display
                this.updateKeyDisplay(key, keyCode, isHeld);

                if (!isHeld) {
                    this.addDebugMessage(`Sent key: '${key}' (${keyCode}) - ${result.message}`);
                }
            } else {
                this.addDebugMessage(`Failed to send key: ${result.error}`);
            }
        } catch (error) {
            this.addDebugMessage(`Send key error: ${error.message}`);
        }
    }

    updateKeyDisplay(key, keyCode, isHeld = false) {
        const keyDisplay = document.getElementById('keyDisplay');
        if (!keyDisplay) return;
        
        const statusText = isHeld ? ' (HELD)' : '';
        const backgroundColor = isHeld ? 'background: #fff8e1;' : '';
        keyDisplay.innerHTML = `
            <p style="border: 2px solid #333; padding: 2rem; margin: 0.5rem 0; min-height: 4rem; display: flex; align-items: center; justify-content: center; text-align: center; ${backgroundColor}">
                <strong>Key: "${key}"${statusText}<br>Code: ${keyCode}</strong>
            </p>
        `;
    }

    handleKeyDown(event) {
        event.preventDefault(); // Prevent default browser behavior

        // Ignore repeat events (when key is held down)
        if (event.repeat) {
            return;
        }

        let key = event.key;
        let keyCode = event.keyCode || event.which;

        // Handle special keys
        key = this.normalizeKeyName(key);

        this.startKeyHold(key, keyCode);
    }

    handleKeyUp(event) {
        event.preventDefault(); // Prevent default browser behavior

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

    startKeyHold(key, keyCode) {
        // Don't start if already holding this key
        if (this.heldKeys.has(keyCode)) {
            return;
        }

        this.heldKeys.set(keyCode, { key, keyCode });
        
        // Send initial key press
        this.sendKey(key, keyCode, false);
        this.addDebugMessage(`Key hold started: '${key}' (${keyCode})`);
        
        // Start continuous sending while held
        const interval = setInterval(() => {
            if (this.heldKeys.has(keyCode)) {
                this.sendKey(key, keyCode, true);
            } else {
                clearInterval(interval);
                this.keyIntervals.delete(keyCode);
            }
        }, 100); // Send every 100ms while held
        
        this.keyIntervals.set(keyCode, interval);
    }

    stopKeyHold(keyCode) {
        if (this.heldKeys.has(keyCode)) {
            const keyInfo = this.heldKeys.get(keyCode);
            this.heldKeys.delete(keyCode);
            
            // Clear the interval
            if (this.keyIntervals.has(keyCode)) {
                clearInterval(this.keyIntervals.get(keyCode));
                this.keyIntervals.delete(keyCode);
            }
            
            this.addDebugMessage(`Key hold stopped: '${keyInfo.key}' (${keyCode})`);
            
            // Update display to show key released
            this.updateKeyDisplay(keyInfo.key, keyCode, false);
            setTimeout(() => {
                const keyDisplay = document.getElementById('keyDisplay');
                if (keyDisplay && this.heldKeys.size === 0) {
                    keyDisplay.innerHTML = '<p style="border: 2px solid #333; padding: 2rem; margin: 0.5rem 0; min-height: 4rem; display: flex; align-items: center; justify-content: center; text-align: center;"><strong>Press and hold any key...</strong></p>';
                }
            }, 1000);
        }
    }

    connectWebSocket() {
        if (this.websocket) {
            return;
        }

        try {
            this.websocket = new WebSocket(this.websocketUrl);

            this.websocket.onopen = (event) => {
                this.addDebugMessage('WebSocket connection opened');
                this.connected = true;
            };

            this.websocket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    this.handleWebSocketEvent(data);
                } catch (e) {
                    this.addDebugMessage(`WebSocket parse error: ${e.message}`);
                }
            };

            this.websocket.onclose = (event) => {
                this.addDebugMessage('WebSocket connection closed');
                this.connected = false;
                this.websocket = null;
                
                // Reconnect after delay
                setTimeout(() => {
                    if (this.websocketEnabled) this.connectWebSocket();
                }, 5000);
            };

            this.websocket.onerror = (error) => {
                this.addDebugMessage(`WebSocket connection error: ${error.message || 'Unknown error'}`);
            };

        } catch (error) {
            this.addDebugMessage(`WebSocket connection failed: ${error.message}`);
        }
    }

    handleWebSocketEvent(data) {
        const eventType = data.type || 'unknown';
        const timestamp = new Date(data.timestamp || Date.now()).toLocaleTimeString();
        
        // Add to live events display
        const liveEventsDiv = document.getElementById('live-events');
        if (liveEventsDiv) {
            const eventDiv = document.createElement('div');
            eventDiv.style.marginBottom = '2px';
            eventDiv.style.fontSize = '0.8em';
            
            switch (eventType) {
                case 'welcome':
                    eventDiv.innerHTML = `<span style="color: #0a84ff;">[${timestamp}] ${data.message || ''}</span>`;
                    break;
                case 'key_received':
                    const heldText = data.is_held ? ' (HELD)' : '';
                    eventDiv.innerHTML = `<span style="color: #32d74b;">[${timestamp}] Key: '${data.key}' (${data.key_code})${heldText}</span>`;
                    break;
                case 'key_down':
                    eventDiv.innerHTML = `<span style="color: #ff9f0a;">[${timestamp}] DOWN: '${data.key}' (${data.key_code})</span>`;
                    break;
                case 'key_up':
                    eventDiv.innerHTML = `<span style="color: #ff453a;">[${timestamp}] UP: '${data.key}' (${data.key_code})</span>`;
                    break;
                case 'keys_batch':
                    eventDiv.innerHTML = `<span style="color: #bf5af2;">[${timestamp}] Batch: ${data.keys?.length || 0} keys</span>`;
                    break;
                default:
                    eventDiv.innerHTML = `<span style="color: #8e8e93;">[${timestamp}] ${eventType}</span>`;
            }
            
            liveEventsDiv.appendChild(eventDiv);
            
            // Keep only last 50 messages
            while (liveEventsDiv.children.length > 50) {
                liveEventsDiv.removeChild(liveEventsDiv.firstChild);
            }
            
            // Auto-scroll to bottom
            liveEventsDiv.scrollTop = liveEventsDiv.scrollHeight;
        }
        
        // Also add to debug messages
        switch (eventType) {
            case 'welcome':
                this.addDebugMessage(`[WS ${timestamp}] ${data.message || ''}`);
                break;
            case 'key_received':
                const heldText = data.is_held ? ' (HELD)' : '';
                this.addDebugMessage(`[WS ${timestamp}] Server processed key: '${data.key}' (${data.key_code})${heldText}`);
                break;
            case 'key_down':
                this.addDebugMessage(`[WS ${timestamp}] Key DOWN: '${data.key}' (${data.key_code})`);
                break;
            case 'key_up':
                this.addDebugMessage(`[WS ${timestamp}] Key UP: '${data.key}' (${data.key_code})`);
                break;
            case 'keys_batch':
                this.addDebugMessage(`[WS ${timestamp}] Server processed batch: ${data.keys?.length || 0} keys`);
                break;
        }
    }

    toggleWebSocket() {
        this.websocketEnabled = !this.websocketEnabled;
        
        if (this.websocketEnabled) {
            this.connectWebSocket();
            this.addDebugMessage('WebSocket enabled');
        } else {
            if (this.websocket) {
                this.websocket.close();
                this.websocket = null;
                this.connected = false;
            }
            this.addDebugMessage('WebSocket disabled');
        }
    }
    
    clearMessages() {
        const liveEventsDiv = document.getElementById('live-events');
        if (liveEventsDiv) {
            liveEventsDiv.innerHTML = '<div style="color: #666;">Messages cleared...</div>';
        }
        this.addDebugMessage('Live events cleared');
    }

    cleanup() {
        // Clear all intervals
        this.keyIntervals.forEach(interval => clearInterval(interval));
        this.keyIntervals.clear();
        this.heldKeys.clear();
        
        // Close WebSocket connection
        if (this.websocket) {
            this.websocket.close();
            this.websocket = null;
        }
    }

    // Public methods for button handlers
    async sendTestKeys() {
        const testKeys = [
            { key: 'h', key_code: 104 },
            { key: 'e', key_code: 101 },
            { key: 'l', key_code: 108 },
            { key: 'l', key_code: 108 },
            { key: 'o', key_code: 111 }
        ];

        for (const keyData of testKeys) {
            await this.sendKey(keyData.key, keyData.key_code);
            await new Promise(resolve => setTimeout(resolve, 200));
        }

        this.addDebugMessage('Test keys sent: HELLO');
    }

    async sendKeyBatch() {
        const keys = [
            { key: 'a', key_code: 97 },
            { key: 'b', key_code: 98 },
            { key: 'c', key_code: 99 }
        ];

        try {
            const response = await fetch(`${this.apiBase}/api/keys/batch`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ keys })
            });

            const result = await response.json();
            
            if (result.success) {
                this.keysSentCount += keys.length;
                this.updateStats();
                this.addDebugMessage(`Batch sent: ${result.message}`);
            } else {
                this.addDebugMessage(`Batch failed: ${result.error}`);
            }
        } catch (error) {
            this.addDebugMessage(`Batch error: ${error.message}`);
        }
    }
}

// Initialize the interface when the page loads
let keyboardInterface;

document.addEventListener('DOMContentLoaded', () => {
    keyboardInterface = new KeyboardInterface();
    
    // Make functions available globally for button onclick handlers
    window.testConnection = () => keyboardInterface.testConnection();
    window.getStatus = () => keyboardInterface.getStatus();
    window.toggleWebSocket = () => keyboardInterface.toggleWebSocket();
    window.clearMessages = () => keyboardInterface.clearMessages();
    window.sendTestKeys = () => keyboardInterface.sendTestKeys();
    window.sendKeyBatch = () => keyboardInterface.sendKeyBatch();
});

// Export for module usage
if (typeof module !== 'undefined' && module.exports) {
    module.exports = KeyboardInterface;
}