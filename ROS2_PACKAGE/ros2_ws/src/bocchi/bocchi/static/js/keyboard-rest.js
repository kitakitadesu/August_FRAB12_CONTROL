/**
 * REST API Keyboard Interface for Bocchi Robot Controller
 * Provides keyboard control via HTTP REST API calls instead of WebSocket
 */

class RESTKeyboardInterface {
    constructor() {
        this.baseUrl = `${window.location.protocol}//${window.location.hostname}:8080`;
        this.connected = false;
        this.keysSentCount = 0;
        this.heldKeys = new Map();
        this.pendingRequests = new Map();
        this.requestId = 0;
        
        // REST API configuration
        this.requestTimeout = 5000; // 5 second timeout
        this.retryAttempts = 3;
        this.retryDelay = 1000; // 1 second between retries
        
        // Status tracking
        this.serverStatus = {
            running: false,
            uptime: 0,
            version: 'unknown'
        };
        
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.initializeInterface();
        this.testConnection(); // Test REST API connection
    }

    initializeInterface() {
        // Initialize UI elements and status display
        this.updateConnectionMode();
        this.addDebugMessage('Initializing REST API keyboard interface...');
        
        // Set initial status
        this.updateStatus({
            running: false,
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
        
        // Page visibility for connection management
        document.addEventListener('visibilitychange', () => {
            if (document.visibilityState === 'visible' && !this.connected) {
                this.testConnection();
            }
        });
    }

    onWindowLoad() {
        this.addDebugMessage('Page loaded, REST API interface ready...');
        this.addDebugMessage(`Base URL: ${this.baseUrl}`);
    }

    generateRequestId() {
        return ++this.requestId;
    }

    async makeRequest(endpoint, method = 'GET', data = null) {
        const url = `${this.baseUrl}${endpoint}`;
        const requestId = this.generateRequestId();
        
        const config = {
            method: method,
            headers: {
                'Content-Type': 'application/json',
            },
        };
        
        if (data) {
            config.body = JSON.stringify(data);
        }
        
        // Add timeout
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.requestTimeout);
        config.signal = controller.signal;
        
        try {
            this.addDebugMessage(`Making ${method} request to ${endpoint}`);
            const response = await fetch(url, config);
            clearTimeout(timeoutId);
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
            const result = await response.json();
            this.addDebugMessage(`Request successful: ${endpoint}`);
            return result;
            
        } catch (error) {
            clearTimeout(timeoutId);
            
            if (error.name === 'AbortError') {
                this.addDebugMessage(`Request timeout: ${endpoint}`);
                throw new Error('Request timeout');
            }
            
            this.addDebugMessage(`Request failed: ${endpoint} - ${error.message}`);
            throw error;
        }
    }

    async testConnection() {
        try {
            this.addDebugMessage('Testing REST API connection...');
            
            const response = await this.makeRequest('/api/test');
            
            if (response.success) {
                this.connected = true;
                this.updateStatus(true);
                this.addDebugMessage('REST API connection test successful');
                return true;
            } else {
                this.updateStatus(false, response.error);
                this.addDebugMessage(`REST API test failed: ${response.error}`);
                return false;
            }
            
        } catch (error) {
            this.connected = false;
            this.updateStatus(false, error.message);
            this.addDebugMessage(`REST API connection test error: ${error.message}`);
            return false;
        }
    }

    async getStatus() {
        try {
            const response = await this.makeRequest('/api/status');
            
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
            const data = {
                key: key,
                key_code: keyCode,
                is_held: isHeld,
                timestamp: Date.now()
            };
            
            const response = await this.makeRequest('/api/key', 'POST', data);
            
            if (response.success) {
                this.keysSentCount++;
                this.updateKeyDisplay(key, keyCode, true);
                this.updateKeysSentDisplay();
                this.addDebugMessage(`Key sent via REST: '${key}' (${keyCode}) ${isHeld ? '[HELD]' : ''}`);
                return true;
            } else {
                this.addDebugMessage(`Key send failed: ${response.error || 'Unknown error'}`);
                return false;
            }
        } catch (error) {
            this.addDebugMessage(`Error sending key via REST: ${error.message}`);
            return false;
        }
    }

    async sendKeyBatch(keysInput) {
        try {
            const data = {
                keys_input: keysInput,
                timestamp: Date.now()
            };
            
            const response = await this.makeRequest('/api/keys/batch', 'POST', data);
            
            if (response.success) {
                this.keysSentCount += response.processed_keys ? response.processed_keys.length : 0;
                this.updateKeysSentDisplay();
                this.addDebugMessage(`Batch sent via REST: ${response.message}`);
                return response;
            } else {
                this.addDebugMessage(`Batch send failed: ${response.error}`);
                return response;
            }
        } catch (error) {
            this.addDebugMessage(`Error sending key batch via REST: ${error.message}`);
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
        
        // Send key down via REST API
        try {
            await this.sendKey(key, keyCode, true);
            this.addDebugMessage(`Key hold started: '${key}' (${keyCode})`);
            this.updateKeyDisplay(key, keyCode, true);
        } catch (error) {
            this.addDebugMessage(`Error starting key hold: ${error.message}`);
        }
    }

    async stopKeyHold(keyCode) {
        if (this.heldKeys.has(keyCode)) {
            const keyInfo = this.heldKeys.get(keyCode);
            this.heldKeys.delete(keyCode);
            
            // Send key up via REST API
            try {
                const data = {
                    key: keyInfo.key,
                    key_code: keyCode,
                    timestamp: Date.now()
                };
                
                await this.makeRequest('/api/key/up', 'POST', data);
                
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

    updateStatus(isConnected, errorMessage = null) {
        const statusContent = document.getElementById('status-content');
        if (statusContent) {
            if (isConnected) {
                statusContent.innerHTML = '<p><strong>Status:</strong> <span style="color: green;">✅ Connected via REST API</span></p>';
            } else {
                const error = errorMessage ? ` (${errorMessage})` : '';
                statusContent.innerHTML = `<p><strong>Status:</strong> <span style="color: red;">❌ Disconnected${error}</span></p>`;
            }
        }
    }

    updateStats(data) {
        const updates = {
            'serverStatus': data.server_running ? 'Running' : 'Stopped',
            'connectedClients': 'N/A (REST)',
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
            modeIndicator.textContent = 'REST API';
            modeIndicator.style.color = '#FFA500';
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
        this.addDebugMessage('Sending test key sequence via REST API...');
        
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
        this.addDebugMessage('Toggling LED via REST API...');
        await this.sendKey('L', 76);
        this.addDebugMessage('LED toggle command sent');
    }

    cleanup() {
        // Clear all held keys
        this.heldKeys.clear();
        
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
                `Batch processed: ${result.processed_keys ? result.processed_keys.length : 0} keys` : 
                `Batch failed: ${result.error}`
        };
    }
}

// Make class globally available
window.RESTKeyboardInterface = RESTKeyboardInterface;

// Global instance
let restKeyboardInterface;

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    // Only initialize if we're in REST mode
    if (typeof currentMode !== 'undefined' && currentMode === 'rest') {
        restKeyboardInterface = new RESTKeyboardInterface();

        // Make functions available globally for button onclick handlers
        window.testConnection = () => restKeyboardInterface.testConnection();
        window.getStatus = () => restKeyboardInterface.getStatus();
        window.clearMessages = () => restKeyboardInterface.clearMessages();
        window.sendTestKeys = () => restKeyboardInterface.sendTestKeys();
        window.toggleLED = () => restKeyboardInterface.toggleLED();

        // Handle manual form submissions
        window.submitManualKey = async (event) => {
            event.preventDefault();
            const formData = new FormData(event.target);
            const result = await restKeyboardInterface.submitManualKeyForm(formData);

            const resultDiv = document.getElementById('manual-result');
            if (resultDiv) {
                resultDiv.innerHTML = `<p style="color: ${result.success ? 'green' : 'red'};">${result.message}</p>`;
            }
        };

        window.submitBatchKey = async (event) => {
            event.preventDefault();
            const formData = new FormData(event.target);
            const result = await restKeyboardInterface.submitBatchKeyForm(formData);

            const resultDiv = document.getElementById('batch-result');
            if (resultDiv) {
                resultDiv.innerHTML = `<p style="color: ${result.success ? 'green' : 'red'};">${result.message}</p>`;
            }
        };
    }
});