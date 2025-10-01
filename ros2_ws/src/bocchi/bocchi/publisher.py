import rclpy
from rclpy.node import Node
import signal
from std_msgs.msg import Int32
import sys
import threading
import json
import os
import tempfile
import socket
import time
from flask import Flask, request, jsonify, Response, send_from_directory, render_template_string
from flask_cors import CORS
import queue
from datetime import datetime
from mako.template import Template
from mako.lookup import TemplateLookup
import socket
from collections import deque
from threading import Lock

minimal_publisher = None

def signal_handler(sig, frame):
    print("\nSIGINT (Ctrl+C) received! Performing graceful shutdown.")
    if minimal_publisher:
        minimal_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

class EventBatcher:
    """Batches events to reduce network packets"""

    def __init__(self, batch_size=5, timeout=0.1):
        self.batch_size = batch_size
        self.timeout = timeout
        self.events = deque()
        self.last_flush = time.time()
        self.lock = Lock()

    def add_event(self, event):
        with self.lock:
            self.events.append(event)
            return len(self.events) >= self.batch_size or (time.time() - self.last_flush) > self.timeout

    def get_batch(self):
        with self.lock:
            if not self.events:
                return []
            batch = list(self.events)
            self.events.clear()
            self.last_flush = time.time()
            return batch

class KeyStateManager:
    """Manages key state to prevent duplicate publications"""

    def __init__(self, debounce_time=0.05):
        self.debounce_time = debounce_time
        self.current_key = None
        self.last_key_time = 0
        self.key_changed = False
        self.lock = Lock()

    def update_key(self, key_code):
        with self.lock:
            current_time = time.time()

            # Check if key has changed or debounce time has passed
            if (self.current_key != key_code or
                current_time - self.last_key_time > self.debounce_time):

                self.current_key = key_code
                self.last_key_time = current_time
                self.key_changed = True
                return True
            return False

    def get_key_if_changed(self):
        with self.lock:
            if self.key_changed:
                self.key_changed = False
                return self.current_key
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
        self.connected_clients = 0
        self.server_start_time = time.time()
        self.event_batcher = EventBatcher(batch_size=5, timeout=0.1)
        self.network_cache = NetworkInfoCache(cache_duration=30)

        # Event queues for different clients
        self.client_queues = {}
        self.client_counter = 0

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
        """Broadcast event to all connected SSE clients efficiently"""
        if self.event_batcher.add_event(event_data):
            # Time to flush batch
            batch = self.event_batcher.get_batch()
            if batch:
                batched_event = {
                    'type': 'batch_events',
                    'events': batch,
                    'count': len(batch),
                    'timestamp': int(time.time() * 1000)
                }

                # Send to all client queues
                for client_queue in list(self.client_queues.values()):
                    try:
                        client_queue.put_nowait(batched_event)
                    except queue.Full:
                        # Client queue full, skip
                        pass

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

                    # Add to event batcher for SSE clients
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
                    # Add to event batcher for SSE clients
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
                if self.publisher_node.key_manager.update_key(key_code):
                    print(f"Key DOWN: '{key}' (code: {key_code})")

                    # Add to event batcher for SSE clients
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

                print(f"Key UP: '{key}' (code: {key_code})")

                # Add to event batcher for SSE clients
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

        @self.app.route('/api/events', methods=['GET'])
        def events():
            """Server-Sent Events endpoint with optimized batching"""
            print("SSE endpoint /api/events called")
            try:
                def event_stream():
                    self.connected_clients += 1
                    self.client_counter += 1
                    client_id = f"client_{self.client_counter}_{int(time.time())}"

                    # Create dedicated queue for this client
                    client_queue = queue.Queue(maxsize=100)
                    self.client_queues[client_id] = client_queue

                    print(f"SSE client connected: {client_id}. Total SSE clients: {self.connected_clients}")

                    try:
                        # Send welcome event
                        yield f"data: {json.dumps({'type': 'welcome', 'message': 'Connected to ROS2 Publisher SSE', 'client_id': client_id, 'timestamp': int(time.time() * 1000)})}\n\n"

                        while True:
                            try:
                                # Get event with timeout
                                event_data = client_queue.get(timeout=30)
                                yield f"data: {json.dumps(event_data)}\n\n"
                                client_queue.task_done()
                            except queue.Empty:
                                # Send keepalive ping (less frequent)
                                yield f"data: {json.dumps({'type': 'ping', 'timestamp': int(time.time() * 1000)})}\n\n"
                            except Exception as e:
                                print(f"SSE stream error for {client_id}: {e}")
                                break

                    except Exception as e:
                        print(f"SSE client error for {client_id}: {e}")
                    finally:
                        # Clean up client
                        if client_id in self.client_queues:
                            del self.client_queues[client_id]
                        self.connected_clients -= 1
                        print(f"SSE client disconnected: {client_id}. Total SSE clients: {self.connected_clients}")

                return Response(event_stream(), mimetype='text/event-stream',
                              headers={
                                  'Cache-Control': 'no-cache',
                                  'Connection': 'keep-alive',
                                  'Access-Control-Allow-Origin': '*',
                                  'Access-Control-Allow-Headers': 'Cache-Control'
                              })
            except Exception as e:
                print(f"Error in SSE endpoint: {e}")
                return jsonify({
                    'success': False,
                    'error': 'SSE endpoint error',
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
        self.publisher_ = self.create_publisher(Int32, 'keyboard', 10)

        # Optimized timer - only publish when key changes
        timer_period = 0.05  # Reduced frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize key state manager
        self.key_manager = KeyStateManager(debounce_time=0.02)
        self.network_cache = NetworkInfoCache(cache_duration=60)
        self.flask_app = None

        # Check if ports are available
        self.check_ports()

        # Start the REST API server
        self.start_rest_api_server()

        print("=" * 50)
        print("ROS2 REST API Keyboard Interface Started (Optimized)")
        print("=" * 50)
        print("Services started on all network interfaces (0.0.0.0):")
        print(f"  Web interface: http://localhost:8000")
        print(f"  REST API: http://localhost:5000")
        print(f"  SSE Events: http://localhost:5000/api/events")
        print("")
        print("Network access URLs (replace <your-ip> with actual IP):")
        print(f"  Web interface: http://<your-ip>:8000")
        print(f"  REST API: http://<your-ip>:5000")
        print(f"  SSE Events: http://<your-ip>:5000/api/events")
        print("")
        print("Optimizations enabled:")
        print("  ✓ Event batching for reduced SSE packets")
        print("  ✓ Key state debouncing to prevent duplicate ROS2 publications")
        print("  ✓ Network info caching to reduce system calls")
        print("  ✓ Efficient client queue management")
        print("Press any key in the web browser to publish to /keyboard topic")
        print("=" * 50)
        self.print_network_info()

    def check_ports(self):
        """Check if ports 8000 and 5000 are available"""
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
                        print(f"  SSE Events: http://{ip}:5000/api/events")
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
        """Start the REST API server with optimized SSE support"""
        def run_rest_server():
            try:
                print("Initializing optimized REST API server with Mako templating...")

                # Create API handler
                api_handler = KeyboardAPI(self)
                self.flask_app = api_handler.app

                # Ensure HTML file is created (for backwards compatibility)
                self.create_html_file()

                print("✓ REST API server starting on 0.0.0.0:5000")
                print("✓ Mako templates loaded from bocchi/templates/")
                print("✓ Static files served from bocchi/static/")
                print("✓ Optimized Server-Sent Events available at /api/events")
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

    def timer_callback(self):
        """Optimized timer callback - only publish when key state changes"""
        key_code = self.key_manager.get_key_if_changed()
        if key_code is not None:
            msg = Int32()
            msg.data = key_code
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing key state change: {key_code}')

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
