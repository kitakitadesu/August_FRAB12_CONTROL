import rclpy
from rclpy.node import Node
import signal
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import sys
import threading
import json
import os
import tempfile
import socket
import time
from flask import Flask, request, jsonify, send_from_directory, render_template_string
from flask_cors import CORS
from datetime import datetime
from mako.template import Template
from mako.lookup import TemplateLookup
import socket
from threading import Lock
import asyncio
import websockets

minimal_publisher = None

def signal_handler(sig, frame):
    print("\nSIGINT (Ctrl+C) received! Performing graceful shutdown.")
    if minimal_publisher:
        minimal_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

class WebSocketManager:
    """Manages WebSocket connections and broadcasting"""

    def __init__(self):
        self.connected_clients = set()
        self.lock = Lock()

    def add_client(self, websocket):
        with self.lock:
            self.connected_clients.add(websocket)

    def remove_client(self, websocket):
        with self.lock:
            self.connected_clients.discard(websocket)

    async def broadcast(self, message):
        """Broadcast message to all connected WebSocket clients"""
        if not self.connected_clients:
            return
        
        disconnected_clients = set()
        message_str = json.dumps(message)
        
        for client in self.connected_clients.copy():
            try:
                await client.send(message_str)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
            except Exception as e:
                print(f"Error sending to WebSocket client: {e}")
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        with self.lock:
            for client in disconnected_clients:
                self.connected_clients.discard(client)

class KeyStateManager:
    """Manages key state to prevent duplicate publications"""

    def __init__(self, debounce_time=0.05):
        self.debounce_time = debounce_time
        self.current_keys = set()
        self.servo_position = 0  # Track servo position (0 or 180)
        self.last_key_time = 0
        self.key_changed = False
        self.servo_changed = False
        self.lock = Lock()

    def update_key(self, key_code, is_pressed=True):
        with self.lock:
            current_time = time.time()
            
            # Handle F key for servo toggle
            if key_code == 70 and is_pressed:  # F key
                self.servo_position = 180 if self.servo_position == 0 else 0
                self.servo_changed = True
                self.last_key_time = current_time
                return True
            
            if is_pressed:
                if key_code not in self.current_keys:
                    self.current_keys.add(key_code)
                    self.last_key_time = current_time
                    self.key_changed = True
                    return True
            else:
                if key_code in self.current_keys:
                    self.current_keys.remove(key_code)
                    self.last_key_time = current_time
                    self.key_changed = True
                    return True
            return False

    def get_twist_if_changed(self):
        with self.lock:
            if self.key_changed:
                self.key_changed = False
                return self._calculate_twist()
            return None
    
    def _calculate_twist(self):
        """Calculate Twist message based on current pressed keys"""
        twist = Twist()
        
        # Key mappings based on WASD keys
        # W = forward (87)
        # S = backward (83)
        # A = turn left (65)
        # D = turn right (68)
        
        # Linear velocity (forward/backward)
        if 87 in self.current_keys:  # W
            twist.linear.x = 0.5
        elif 83 in self.current_keys:  # S
            twist.linear.x = -0.5
        else:
            twist.linear.x = 0.0
            
        # Angular velocity (left/right turn)
        if 65 in self.current_keys:  # A
            twist.angular.z = 0.5
        elif 68 in self.current_keys:  # D
            twist.angular.z = -0.5
        else:
            twist.angular.z = 0.0
            
        # All other fields remain 0.0 by default
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        
        return twist
    
    def get_servo_if_changed(self):
        with self.lock:
            if self.servo_changed:
                self.servo_changed = False
                return self.servo_position
            return None

class NetworkInfoCache:
    """Caches network information to reduce system calls"""

    def __init__(self, cache_duration=30):
        self.cache_duration = cache_duration
        self.cached_info = None
        self.last_update = 0
        self.lock = Lock()

    def get_network_info(self):
        with self.lock:
            current_time = time.time()
            if (self.cached_info is None or
                current_time - self.last_update > self.cache_duration):

                self.cached_info = self._fetch_network_info()
                self.last_update = current_time

            return self.cached_info

    def _fetch_network_info(self):
        """Fetch fresh network information"""
        try:
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)

            # Try to get more network interfaces
            available_ips = [local_ip]
            try:
                import subprocess
                result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    available_ips = result.stdout.strip().split()
            except Exception:
                try:
                    # Fallback method to get IP
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    available_ips = [s.getsockname()[0]]
                    s.close()
                except Exception:
                    available_ips = [local_ip]

            return {
                'hostname': hostname,
                'primary_ip': local_ip,
                'ips': [ip for ip in available_ips if ip != '127.0.0.1']
            }
        except Exception as e:
            print(f"Network info error: {e}")
            return {
                'hostname': 'localhost',
                'primary_ip': '127.0.0.1',
                'ips': ['127.0.0.1']
            }

class KeyboardAPI:
    """REST API for keyboard input with optimized SSE support"""

    def __init__(self, publisher_node):
        self.publisher_node = publisher_node
        self.app = Flask(__name__)
        CORS(self.app)  # Enable CORS for all routes
        self.server_start_time = time.time()
        self.network_cache = NetworkInfoCache(cache_duration=30)
        self.websocket_manager = WebSocketManager()

        # Setup Mako template lookup
        template_dir = os.path.join(os.path.dirname(__file__), 'templates')
        static_dir = os.path.join(os.path.dirname(__file__), 'static')
        self.template_lookup = TemplateLookup(
            directories=[template_dir],
            imports=['import builtins', 'import operator'],
            default_filters=['str']
        )

        # Configure Flask static folder
        self.app.static_folder = static_dir

        self.setup_routes()

        # Debug: Print all registered routes
        print("Registered Flask routes:")
        for rule in self.app.url_map.iter_rules():
            print(f"  {rule.rule} -> {rule.endpoint} [{', '.join(rule.methods)}]")

    def broadcast_event(self, event_data):
        """Broadcast event to all connected WebSocket clients"""
        # Use thread-safe approach to schedule async broadcast
        if self.websocket_manager.connected_clients:
            threading.Thread(
                target=self._schedule_broadcast, 
                args=(event_data,), 
                daemon=True
            ).start()
    
    def _schedule_broadcast(self, event_data):
        """Schedule async broadcast in a new event loop"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.websocket_manager.broadcast(event_data))
            loop.close()
        except Exception as e:
            print(f"Error broadcasting to WebSocket clients: {e}")

    def setup_routes(self):
        """Setup REST API routes"""

        @self.app.route('/', methods=['GET'])
        def serve_index():
            """Serve the main HTML interface using Mako template"""
            try:
                template = self.template_lookup.get_template('index.html')

                # Get cached network information
                network_info = self.network_cache.get_network_info()

                context = {
                    'timestamp': int(time.time() * 1000),
                    'server_status': 'Running',
                    'connected_clients': self.connected_clients,
                    'uptime': int(time.time() - self.server_start_time),
                    'version': '1.0.0',
                    'current_host': network_info.get('hostname', 'localhost'),
                    'current_url': request.url,
                    'protocol': request.scheme,
                    'api_base': f"http://{network_info.get('hostname', 'localhost')}:5000",
                    'web_url': f"http://{network_info.get('hostname', 'localhost')}:8000",
                    'available_ips': ','.join(network_info.get('ips', ['localhost'])),
                    'server_info': 'ROS2 REST API Keyboard Interface'
                }

                return template.render(**context)
            except Exception as e:
                print(f"Template error: {e}")
                return f"<h1>Template Error</h1><p>{str(e)}</p>", 500

        @self.app.route('/static/<path:filename>')
        def static_files(filename):
            """Serve static files"""
            return send_from_directory(self.app.static_folder, filename)

        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            """Get server status - returns JSON for API calls or HTML for HTMX"""
            try:
                uptime = int(time.time() - self.server_start_time)
                status_data = {
                    'server_running': True,
                    'connected_clients': self.connected_clients,
                    'version': '1.0.0',
                    'uptime': uptime,
                    'timestamp': int(time.time() * 1000)
                }

                # Check if request is from HTMX
                if request.headers.get('HX-Request'):
                    try:
                        template = self.template_lookup.get_template('status.html')
                        context = {
                            'status_class': 'connected',
                            'server_running': True,
                            'connected_clients': self.connected_clients,
                            'uptime': uptime,
                            'version': '1.0.0',
                            'error_message': None
                        }
                        return template.render(**context)
                    except Exception as e:
                        return f"<div class='status disconnected'>Template error: {str(e)}</div>"

                # Regular JSON API response
                return jsonify({
                    'success': True,
                    'data': status_data
                })
            except Exception as e:
                error_response = {
                    'success': False,
                    'error': str(e),
                    'timestamp': int(time.time() * 1000)
                }

                if request.headers.get('HX-Request'):
                    try:
                        template = self.template_lookup.get_template('status.html')
                        context = {
                            'status_class': 'disconnected',
                            'server_running': False,
                            'connected_clients': 0,
                            'uptime': 0,
                            'version': '1.0.0',
                            'error_message': str(e)
                        }
                        return template.render(**context)
                    except:
                        return f"<div class='status disconnected'>Error: {str(e)}</div>"

                return jsonify(error_response), 500

        @self.app.route('/api/key', methods=['POST'])
        def send_key():
            """Send a single key press with state management"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({
                        'success': False,
                        'error': 'No JSON data provided',
                        'timestamp': int(time.time() * 1000)
                    }), 400

                key = data.get('key', '')
                key_code = data.get('key_code', 0)
                timestamp = data.get('timestamp', int(time.time() * 1000))
                is_held = data.get('is_held', False)

                # Update key state only if changed (reduces ROS2 publications)
                if self.publisher_node.key_manager.update_key(key_code):
                    held_status = " (HELD)" if is_held else ""
                    print(f"Key state updated: '{key}' (code: {key_code}){held_status}")

                    # Broadcast to WebSocket clients
                    event_data = {
                        'type': 'key_received',
                        'key': key,
                        'key_code': key_code,
                        'is_held': is_held,
                        'timestamp': timestamp
                    }
                    self.broadcast_event(event_data)

                    # Handle quit command (only on initial press, not while held)
                    if key.lower() == 'q' and not is_held:
                        print("Quit command received - stopping publisher")
                        # Signal shutdown in a separate thread
                        threading.Thread(target=lambda: os.kill(os.getpid(), signal.SIGINT), daemon=True).start()

                return jsonify({
                    'success': True,
                    'message': f"Key '{key}' processed successfully",
                    'timestamp': int(time.time() * 1000)
                })

            except Exception as e:
                print(f"Error processing key: {e}")
                return jsonify({
                    'success': False,
                    'error': str(e),
                    'timestamp': int(time.time() * 1000)
                }), 500

        @self.app.route('/api/keys/batch', methods=['POST'])
        def send_keys_batch():
            """Send multiple keys at once with optimized processing"""
            try:
                data = request.get_json()
                if not data or 'keys' not in data:
                    return jsonify({
                        'success': False,
                        'error': 'No keys data provided',
                        'timestamp': int(time.time() * 1000)
                    }), 400

                keys = data['keys']
                if not isinstance(keys, list):
                    return jsonify({
                        'success': False,
                        'error': 'Keys must be an array',
                        'timestamp': int(time.time() * 1000)
                    }), 400

                processed_keys = []
                last_key_code = None

                for key_data in keys:
                    key = key_data.get('key', '')
                    key_code = key_data.get('key_code', 0)

                    # Only process if key changed (avoid duplicate processing)
                    if last_key_code != key_code:
                        if self.publisher_node.key_manager.update_key(key_code):
                            processed_keys.append({'key': key, 'key_code': key_code})
                            print(f"Batch key: '{key}' (code: {key_code})")
                        last_key_code = key_code

                if processed_keys:
                    # Broadcast to WebSocket clients
                    event_data = {
                        'type': 'keys_batch',
                        'keys': processed_keys,
                        'timestamp': int(time.time() * 1000)
                    }
                    self.broadcast_event(event_data)

                return jsonify({
                    'success': True,
                    'message': f"Processed {len(processed_keys)} unique keys",
                    'processed_keys': processed_keys,
                    'timestamp': int(time.time() * 1000)
                })

            except Exception as e:
                print(f"Error processing batch keys: {e}")
                return jsonify({
                    'success': False,
                    'error': str(e),
                    'timestamp': int(time.time() * 1000)
                }), 500

        @self.app.route('/api/key/down', methods=['POST'])
        def key_down():
            """Handle key down event with state tracking"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({
                        'success': False,
                        'error': 'No JSON data provided',
                        'timestamp': int(time.time() * 1000)
                    }), 400

                key = data.get('key', '')
                key_code = data.get('key_code', 0)
                timestamp = data.get('timestamp', int(time.time() * 1000))

                # Update key state only if changed
                if self.publisher_node.key_manager.update_key(key_code, is_pressed=True):
                    print(f"Key DOWN: '{key}' (code: {key_code})")

                    # Broadcast to WebSocket clients
                    event_data = {
                        'type': 'key_down',
                        'key': key,
                        'key_code': key_code,
                        'timestamp': timestamp
                    }
                    self.broadcast_event(event_data)

                return jsonify({
                    'success': True,
                    'message': f"Key down '{key}' processed successfully",
                    'timestamp': int(time.time() * 1000)
                })

            except Exception as e:
                print(f"Error processing key down: {e}")
                return jsonify({
                    'success': False,
                    'error': str(e),
                    'timestamp': int(time.time() * 1000)
                }), 500

        @self.app.route('/api/key/up', methods=['POST'])
        def key_up():
            """Handle key up event"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({
                        'success': False,
                        'error': 'No JSON data provided',
                        'timestamp': int(time.time() * 1000)
                    }), 400

                key = data.get('key', '')
                key_code = data.get('key_code', 0)
                timestamp = data.get('timestamp', int(time.time() * 1000))

                # Update key state for key release
                self.publisher_node.key_manager.update_key(key_code, is_pressed=False)
                print(f"Key UP: '{key}' (code: {key_code})")

                # Broadcast to WebSocket clients
                event_data = {
                    'type': 'key_up',
                    'key': key,
                    'key_code': key_code,
                    'timestamp': timestamp
                }
                self.broadcast_event(event_data)

                return jsonify({
                    'success': True,
                    'message': f"Key up '{key}' processed successfully",
                    'timestamp': int(time.time() * 1000)
                })

            except Exception as e:
                print(f"Error processing key up: {e}")
                return jsonify({
                    'success': False,
                    'error': str(e),
                    'timestamp': int(time.time() * 1000)
                }), 500

        @self.app.route('/api/websocket-info', methods=['GET'])
        def websocket_info():
            """WebSocket connection information endpoint"""
            try:
                connected_count = len(self.websocket_manager.connected_clients)
                return jsonify({
                    'success': True,
                    'websocket_url': 'ws://localhost:8765',
                    'connected_clients': connected_count,
                    'server_start_time': self.server_start_time,
                    'uptime_seconds': int(time.time() - self.server_start_time),
                    'timestamp': int(time.time() * 1000)
                })
            except Exception as e:
                print(f"Error in WebSocket info endpoint: {e}")
                return jsonify({
                    'success': False,
                    'error': 'WebSocket info endpoint error',
                    'details': str(e)
                }), 500

        @self.app.route('/api/test', methods=['GET'])
        def test_endpoint():
            """Simple test endpoint to verify route registration"""
            return jsonify({
                'success': True,
                'message': 'Test endpoint working',
                'timestamp': int(time.time() * 1000)
            })

        @self.app.route('/api/debug/routes', methods=['GET'])
        def list_routes():
            """Debug endpoint to list all available routes"""
            routes = []
            for rule in self.app.url_map.iter_rules():
                routes.append({
                    'endpoint': rule.endpoint,
                    'methods': list(rule.methods),
                    'rule': rule.rule
                })
            return jsonify({
                'success': True,
                'routes': routes,
                'total_routes': len(routes)
            })

        @self.app.errorhandler(404)
        def not_found(error):
            return jsonify({
                'success': False,
                'error': 'Endpoint not found',
                'path': request.path,
                'method': request.method,
                'available_endpoints': [rule.rule for rule in self.app.url_map.iter_rules()]
            }), 404

        @self.app.errorhandler(500)
        def internal_error(error):
            return jsonify({
                'success': False,
                'error': 'Internal server error',
                'timestamp': int(time.time() * 1000)
            }), 500

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_publisher_ = self.create_publisher(Int32, 'servo_position', 10)

        # Optimized timer - only publish when key changes
        timer_period = 0.05  # Reduced frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize key state manager
        self.key_manager = KeyStateManager(debounce_time=0.02)
        self.network_cache = NetworkInfoCache(cache_duration=60)
        self.flask_app = None
        self.websocket_server = None

        # Check if ports are available
        self.check_ports()

        # Start the REST API server and WebSocket server
        self.start_rest_api_server()
        self.start_websocket_server()

        print("=" * 50)
        print("ROS2 REST API Keyboard Interface Started")
        print("=" * 50)
        print("Services started on all network interfaces (0.0.0.0):")
        print(f"  Web interface: http://localhost:8000")
        print(f"  REST API: http://localhost:5000")
        print(f"  WebSocket: ws://localhost:8765")
        print("")
        print("Network access URLs (replace <your-ip> with actual IP):")
        print(f"  Web interface: http://<your-ip>:8000")
        print(f"  REST API: http://<your-ip>:5000")
        print(f"  WebSocket: ws://<your-ip>:8765")
        print("")
        print("Features enabled:")
        print("  ✓ Real-time WebSocket communication")
        print("  ✓ Key state debouncing to prevent duplicate ROS2 publications")
        print("  ✓ Network info caching to reduce system calls")
        print("  ✓ Multi-key support for combined movements")
        print("Press WASD keys for movement and F key for servo toggle")
        print("W: Forward, S: Backward, A: Turn Left, D: Turn Right, F: Servo 0°/180°")
        print("=" * 50)
        self.print_network_info()

    def check_ports(self):
        """Check if ports 8000, 5000, and 8765 are available"""
        def is_port_available(port):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', port))
                return True
            except socket.error:
                return False

        if not is_port_available(8000):
            print("WARNING: Port 8000 is already in use. HTTP server may not start.")
        if not is_port_available(5000):
            print("WARNING: Port 5000 is already in use. REST API server may not start.")
        if not is_port_available(8765):
            print("WARNING: Port 8765 is already in use. WebSocket server may not start.")

    def print_network_info(self):
        """Print network diagnostic information using cached data"""
        try:
            network_info = self.network_cache.get_network_info()

            print("\nNetwork Diagnostics:")
            print(f"  Hostname: {network_info['hostname']}")
            print(f"  Primary IP: {network_info['primary_ip']}")

            available_ips = network_info.get('ips', [])
            if available_ips:
                print(f"  Available IPs: {', '.join(available_ips)}")
                print("\nAccess URLs:")
                for ip in available_ips:
                    if ip != '127.0.0.1':
                        print(f"  Web interface: http://{ip}:8000")
                        print(f"  REST API: http://{ip}:5000")
                        print(f"  WebSocket: ws://{ip}:8765")
                        print()

            print("  Docker/Container notes:")
            print("  - If in container, use container IP for external access")
            print("  - Common container IP ranges: 192.168.x.x, 172.x.x.x")
            print()
        except Exception as e:
            print(f"Network diagnostics failed: {e}")

    def create_html_file(self):
        """Legacy method - now using Mako templates"""
        # Create a temporary directory for backwards compatibility
        self.temp_dir = tempfile.mkdtemp()
        return self.temp_dir

    def start_rest_api_server(self):
        """Start the REST API server with WebSocket support"""
        def run_rest_server():
            try:
                print("Initializing REST API server with WebSocket and Mako templating...")

                # Create API handler
                api_handler = KeyboardAPI(self)
                self.flask_app = api_handler.app
                self.flask_app.config['api_handler'] = api_handler

                # Ensure HTML file is created (for backwards compatibility)
                self.create_html_file()

                print("✓ REST API server starting on 0.0.0.0:5000")
                print("✓ Mako templates loaded from bocchi/templates/")
                print("✓ Static files served from bocchi/static/")
                print("✓ Real-time WebSocket communication enabled")
                print("✓ HTMX integration enabled")
                print("✓ Event batching and key debouncing enabled")

                # Run Flask server
                self.flask_app.run(
                    host='0.0.0.0',
                    port=5000,
                    debug=False,
                    use_reloader=False,
                    threaded=True
                )

            except Exception as e:
                print(f"✗ Failed to start REST API server: {e}")
                import traceback
                traceback.print_exc()

        print("Starting optimized REST API server thread...")
        self.api_thread = threading.Thread(target=run_rest_server, daemon=True)
        self.api_thread.start()

        # Give the server time to start
        print("Waiting for REST API server to initialize...")
        time.sleep(2)

    def start_websocket_server(self):
        """Start the WebSocket server for real-time communication"""
        def run_websocket_server():
            try:
                print("Initializing WebSocket server...")
                
                async def handle_websocket(websocket, path):
                    """Handle WebSocket connections"""
                    self.flask_app.config['api_handler'].websocket_manager.add_client(websocket)
                    client_address = websocket.remote_address
                    print(f"WebSocket client connected from {client_address}")
                    
                    try:
                        # Send welcome message
                        welcome_msg = {
                            'type': 'welcome',
                            'message': f'Connected to bocchi robot controller',
                            'timestamp': int(time.time() * 1000)
                        }
                        await websocket.send(json.dumps(welcome_msg))
                        
                        # Keep connection alive and handle incoming messages
                        async for message in websocket:
                            try:
                                data = json.loads(message)
                                print(f"Received WebSocket message: {data}")
                                # Handle any client-side messages if needed
                            except json.JSONDecodeError:
                                print(f"Invalid JSON received from WebSocket client")
                                
                    except websockets.exceptions.ConnectionClosed:
                        print(f"WebSocket client {client_address} disconnected")
                    except Exception as e:
                        print(f"WebSocket error: {e}")
                    finally:
                        self.flask_app.config['api_handler'].websocket_manager.remove_client(websocket)
                
                async def start_server():
                    try:
                        server = await websockets.serve(
                            handle_websocket,
                            "0.0.0.0",
                            8765,
                            ping_interval=20,
                            ping_timeout=10
                        )
                        print("✓ WebSocket server started successfully on port 8765")
                        await server.wait_closed()
                    except Exception as e:
                        print(f"✗ Failed to start WebSocket server: {e}")
                
                # Run the WebSocket server
                asyncio.run(start_server())
                
            except Exception as e:
                print(f"✗ WebSocket server error: {e}")
                import traceback
                traceback.print_exc()

        print("Starting WebSocket server thread...")
        self.websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
        self.websocket_thread.start()
        
        # Give the WebSocket server time to start
        print("Waiting for WebSocket server to initialize...")
        time.sleep(1)

    def timer_callback(self):
        """Optimized timer callback - only publish when key state changes"""
        twist_msg = self.key_manager.get_twist_if_changed()
        if twist_msg is not None:
            self.publisher_.publish(twist_msg)
            self.get_logger().info(f'Publishing cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')
        
        servo_position = self.key_manager.get_servo_if_changed()
        if servo_position is not None:
            servo_msg = Int32()
            servo_msg.data = servo_position
            self.servo_publisher_.publish(servo_msg)
            self.get_logger().info(f'Publishing servo position: {servo_position}°')

    def __del__(self):
        """Cleanup when the node is destroyed"""
        # Clean up Flask server - Flask will stop when thread ends

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
