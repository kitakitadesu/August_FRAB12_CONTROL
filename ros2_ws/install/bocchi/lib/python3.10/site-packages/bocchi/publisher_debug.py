# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import signal
from std_msgs.msg import String, Int32
import sys
import asyncio
import websockets
import threading
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os
import tempfile
import socket
import time

minimal_publisher = None

def signal_handler(sig, frame):
    print("\nSIGINT (Ctrl+C) received! Performing graceful shutdown.")
    if minimal_publisher:
        minimal_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32, 'keyboard', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.latest_key = None
        self.websocket_clients = set()
        self.websocket_server = None

        # Check if ports are available
        self.check_ports()

        # Start the web server and websocket server in separate threads
        self.start_web_server()
        self.start_websocket_server()

        print("=" * 50)
        print("ROS2 Web Keyboard Interface Started")
        print("=" * 50)
        print(f"Web interface: http://localhost:8000")
        print(f"WebSocket server: ws://localhost:8765")
        print("Press any key in the web browser to publish to /keyboard topic")
        print("=" * 50)

    def check_ports(self):
        """Check if ports 8000 and 8765 are available"""
        def is_port_available(port):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', port))
                return True
            except socket.error:
                return False

        if not is_port_available(8000):
            print("WARNING: Port 8000 is already in use. HTTP server may not start.")
        if not is_port_available(8765):
            print("WARNING: Port 8765 is already in use. WebSocket server may not start.")

    def create_html_file(self):
        """Create the HTML file for keyboard input"""
        html_content = """
<!DOCTYPE html>
<html>
<head>
    <title>ROS2 Keyboard Input</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        .status {
            padding: 10px;
            margin: 10px 0;
            border-radius: 5px;
            font-weight: bold;
        }
        .connected { background-color: #d4edda; color: #155724; }
        .disconnected { background-color: #f8d7da; color: #721c24; }
        .key-display {
            font-size: 24px;
            padding: 20px;
            margin: 20px 0;
            background-color: #e9ecef;
            border-radius: 5px;
            text-align: center;
            min-height: 60px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .instructions {
            background-color: #cce7ff;
            padding: 15px;
            border-radius: 5px;
            margin: 20px 0;
        }
        .debug-info {
            background-color: #fff3cd;
            padding: 15px;
            border-radius: 5px;
            margin: 20px 0;
            font-family: monospace;
            font-size: 12px;
            max-height: 200px;
            overflow-y: auto;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ROS2 Keyboard Input Interface</h1>

        <div id="status" class="status disconnected">
            Connecting to WebSocket server...
        </div>

        <div class="instructions">
            <h3>Instructions:</h3>
            <ul>
                <li>Click anywhere on this page to focus it</li>
                <li>Press any key to send it to the ROS2 publisher</li>
                <li>Press 'q' to quit the publisher</li>
                <li>The key code will be published to the 'keyboard' topic</li>
            </ul>
        </div>

        <div class="key-display" id="keyDisplay">
            Press any key...
        </div>

        <div class="debug-info" id="debugInfo">
            <strong>Debug Log:</strong><br>
            <div id="debugMessages"></div>
        </div>
    </div>

    <script>
        let socket = null;
        let connected = false;
        let reconnectAttempts = 0;
        const maxReconnectAttempts = 10;

        function addDebugMessage(message) {
            const debugMessages = document.getElementById('debugMessages');
            const timestamp = new Date().toLocaleTimeString();
            debugMessages.innerHTML += `[${timestamp}] ${message}<br>`;
            debugMessages.scrollTop = debugMessages.scrollHeight;
            console.log(`[${timestamp}] ${message}`);
        }

        function updateStatus(isConnected, message = '') {
            const statusDiv = document.getElementById('status');
            connected = isConnected;
            if (isConnected) {
                statusDiv.textContent = 'Connected to WebSocket server';
                statusDiv.className = 'status connected';
                reconnectAttempts = 0;
                addDebugMessage('WebSocket connected successfully');
            } else {
                statusDiv.textContent = `Disconnected from WebSocket server ${message}`;
                statusDiv.className = 'status disconnected';
                addDebugMessage(`WebSocket disconnected: ${message}`);
            }
        }

        function connectWebSocket() {
            if (reconnectAttempts >= maxReconnectAttempts) {
                addDebugMessage(`Max reconnection attempts (${maxReconnectAttempts}) reached. Stopping.`);
                return;
            }

            try {
                // Get the current host from the address bar
                const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                const host = window.location.hostname;
                const wsUrl = `${protocol}//${host}:8765`;

                addDebugMessage(`Attempting to connect to: ${wsUrl}`);
                socket = new WebSocket(wsUrl);

                socket.onopen = function(event) {
                    addDebugMessage('WebSocket connection opened');
                    updateStatus(true);
                };

                socket.onclose = function(event) {
                    addDebugMessage(`WebSocket closed. Code: ${event.code}, Reason: ${event.reason || 'No reason provided'}`);
                    updateStatus(false, `(Code: ${event.code})`);

                    // Try to reconnect after delay
                    reconnectAttempts++;
                    const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 10000);
                    addDebugMessage(`Attempting reconnection in ${delay/1000} seconds (attempt ${reconnectAttempts}/${maxReconnectAttempts})`);
                    setTimeout(connectWebSocket, delay);
                };

                socket.onerror = function(error) {
                    addDebugMessage(`WebSocket error occurred`);
                    console.error('WebSocket error:', error);
                    updateStatus(false, '(Error occurred)');
                };

                socket.onmessage = function(event) {
                    addDebugMessage(`Received message: ${event.data}`);
                };

            } catch (error) {
                addDebugMessage(`Failed to create WebSocket: ${error.message}`);
                updateStatus(false, `(${error.message})`);
                reconnectAttempts++;
                setTimeout(connectWebSocket, 3000);
            }
        }

        function sendKey(key, keyCode) {
            if (connected && socket && socket.readyState === WebSocket.OPEN) {
                const message = {
                    key: key,
                    keyCode: keyCode,
                    timestamp: Date.now()
                };

                try {
                    socket.send(JSON.stringify(message));

                    // Update display
                    const keyDisplay = document.getElementById('keyDisplay');
                    keyDisplay.innerHTML = `
                        <div>
                            <div>Key: "${key}"</div>
                            <div>Code: ${keyCode}</div>
                        </div>
                    `;

                    addDebugMessage(`Sent key: '${key}' (${keyCode})`);
                } catch (error) {
                    addDebugMessage(`Failed to send key: ${error.message}`);
                }
            } else {
                addDebugMessage(`Cannot send key - WebSocket not connected (state: ${socket ? socket.readyState : 'null'})`);
            }
        }

        // Handle keyboard events
        document.addEventListener('keydown', function(event) {
            event.preventDefault(); // Prevent default browser behavior

            let key = event.key;
            let keyCode = event.keyCode || event.which;

            // Handle special keys
            if (key === 'ArrowUp') key = 'UP';
            else if (key === 'ArrowDown') key = 'DOWN';
            else if (key === 'ArrowLeft') key = 'LEFT';
            else if (key === 'ArrowRight') key = 'RIGHT';
            else if (key === ' ') key = 'SPACE';
            else if (key === 'Enter') key = 'ENTER';
            else if (key === 'Escape') key = 'ESC';
            else if (key === 'Backspace') key = 'BACKSPACE';
            else if (key === 'Tab') key = 'TAB';

            sendKey(key, keyCode);
        });

        // Make sure the page can receive keyboard events
        document.addEventListener('click', function() {
            document.body.focus();
        });

        // Connect to WebSocket when page loads
        window.addEventListener('load', function() {
            addDebugMessage('Page loaded, starting WebSocket connection...');
            connectWebSocket();
            document.body.focus(); // Focus the body to receive keyboard events
        });

        // Add connection test button
        document.addEventListener('DOMContentLoaded', function() {
            const testButton = document.createElement('button');
            testButton.textContent = 'Test Connection';
            testButton.style.margin = '10px';
            testButton.onclick = function() {
                addDebugMessage('Manual connection test initiated');
                connectWebSocket();
            };
            document.querySelector('.container').appendChild(testButton);
        });
    </script>
</body>
</html>
        """

        # Create a temporary directory for the HTML file
        self.temp_dir = tempfile.mkdtemp()
        self.html_file = os.path.join(self.temp_dir, 'index.html')

        with open(self.html_file, 'w') as f:
            f.write(html_content)

        return self.temp_dir

    def start_web_server(self):
        """Start the HTTP server in a separate thread"""
        def run_server():
            try:
                # Change to the directory containing the HTML file
                os.chdir(self.create_html_file())

                class CustomHTTPRequestHandler(SimpleHTTPRequestHandler):
                    def log_message(self, format, *args):
                        # Only log errors
                        if '404' in str(args) or '500' in str(args):
                            print(f"HTTP Server: {format % args}")

                server = HTTPServer(('0.0.0.0', 8000), CustomHTTPRequestHandler)
                print("HTTP Server started successfully on port 8000")
                server.serve_forever()
            except Exception as e:
                print(f"Failed to start HTTP server: {e}")

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    def start_websocket_server(self):
        """Start the WebSocket server in a separate thread"""
        def run_websocket_server():
            try:
                async def handle_client(websocket, path):
                    client_address = websocket.remote_address
                    self.websocket_clients.add(websocket)
                    print(f"WebSocket client connected from {client_address}. Total clients: {len(self.websocket_clients)}")

                    try:
                        async for message in websocket:
                            try:
                                data = json.loads(message)
                                key = data.get('key', '')
                                key_code = data.get('keyCode', 0)

                                # Store the latest key for the ROS2 publisher
                                self.latest_key = key_code

                                print(f"Received key: '{key}' (code: {key_code}) from {client_address}")

                                # Handle quit command
                                if key.lower() == 'q':
                                    print("Quit command received - stopping publisher")
                                    # Signal shutdown
                                    os.kill(os.getpid(), signal.SIGINT)

                            except json.JSONDecodeError as e:
                                print(f"Invalid JSON received from {client_address}: {message}")
                            except Exception as e:
                                print(f"Error processing message from {client_address}: {e}")

                    except websockets.exceptions.ConnectionClosed:
                        print(f"WebSocket connection closed normally for {client_address}")
                    except Exception as e:
                        print(f"WebSocket error for {client_address}: {e}")
                    finally:
                        self.websocket_clients.discard(websocket)
                        print(f"WebSocket client {client_address} disconnected. Total clients: {len(self.websocket_clients)}")

                async def start_server():
                    try:
                        # Bind to all interfaces (0.0.0.0) to allow connections from other machines
                        server = await websockets.serve(
                            handle_client,
                            "0.0.0.0",
                            8765,
                            ping_interval=20,
                            ping_timeout=10
                        )
                        print("WebSocket server started successfully on port 8765")
                        await server.wait_closed()
                    except Exception as e:
                        print(f"Failed to start WebSocket server: {e}")
                        raise

                # Create and run the event loop
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

                try:
                    loop.run_until_complete(start_server())
                except Exception as e:
                    print(f"WebSocket server error: {e}")
                finally:
                    loop.close()

            except Exception as e:
                print(f"Critical error in WebSocket server thread: {e}")
                import traceback
                traceback.print_exc()

        self.websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
        self.websocket_thread.start()

        # Give the server a moment to start
        time.sleep(0.5)

    def timer_callback(self):
        """Timer callback to publish the latest key"""
        if self.latest_key is not None:
            msg = Int32()
            msg.data = self.latest_key
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing key code: {self.latest_key}')
            self.latest_key = None  # Clear the key after publishing

    def __del__(self):
        """Cleanup when the node is destroyed"""
        # Clean up temporary files
        if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
            import shutil
            shutil.rmtree(self.temp_dir)

def main(args=None):
    global minimal_publisher
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Destroy the node explicitly
        if minimal_publisher:
            minimal_publisher.destroy_node()
        rclpy.shutdown()
        print("ROS2 node shutdown complete")

if __name__ == '__main__':
    main()
