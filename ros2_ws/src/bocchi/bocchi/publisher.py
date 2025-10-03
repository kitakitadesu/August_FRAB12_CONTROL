import rclpy
from rclpy.node import Node
import signal
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
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
from typing import Dict, List, Any, Optional, Callable
from collections import deque
import weakref
import statistics

minimal_publisher = None

def signal_handler(sig, frame):
    print("\nSIGINT (Ctrl+C) received! Performing graceful shutdown.")
    if minimal_publisher:
        minimal_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

class WebSocketManager:
    """Enhanced WebSocket manager with Docker container compatibility and improved error handling"""

    def __init__(self):
        self.connected_clients = set()
        self.lock = Lock()
        self.stats = {
            'connections_total': 0,
            'connections_active': 0,
            'messages_sent': 0,
            'messages_received': 0,
            'errors': 0
        }

    def add_client(self, websocket):
        """Add a WebSocket client with improved tracking"""
        with self.lock:
            self.connected_clients.add(websocket)
            self.stats['connections_total'] += 1
            self.stats['connections_active'] += 1
            
        client_address = getattr(websocket, 'remote_address', 'unknown')
        print(f"WebSocket client added: {client_address} (total: {len(self.connected_clients)})")

    def remove_client(self, websocket):
        """Remove a WebSocket client with improved tracking"""
        with self.lock:
            if websocket in self.connected_clients:
                self.connected_clients.discard(websocket)
                self.stats['connections_active'] -= 1
                
        client_address = getattr(websocket, 'remote_address', 'unknown')
        print(f"WebSocket client removed: {client_address} (remaining: {len(self.connected_clients)})")

    async def broadcast(self, message):
        """Enhanced broadcast with better error handling and timeout protection"""
        if not self.connected_clients:
            return 0
        
        disconnected_clients = set()
        message_str = json.dumps(message)
        successful_sends = 0
        
        # Create a copy to avoid modification during iteration
        clients_copy = self.connected_clients.copy()
        
        for client in clients_copy:
            try:
                # Add timeout protection for send operation
                await asyncio.wait_for(client.send(message_str), timeout=5.0)
                successful_sends += 1
                self.stats['messages_sent'] += 1
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
                print(f"WebSocket client disconnected during broadcast")
            except asyncio.TimeoutError:
                disconnected_clients.add(client)
                print(f"WebSocket client send timeout - removing client")
                self.stats['errors'] += 1
            except Exception as e:
                disconnected_clients.add(client)
                print(f"Error broadcasting to WebSocket client: {e}")
                self.stats['errors'] += 1
        
        # Remove disconnected clients
        if disconnected_clients:
            with self.lock:
                for client in disconnected_clients:
                    if client in self.connected_clients:
                        self.connected_clients.discard(client)
                        self.stats['connections_active'] -= 1
        
        return successful_sends

    async def send_to_client(self, websocket, message):
        """Send message to specific client with error handling"""
        try:
            message_str = json.dumps(message)
            await asyncio.wait_for(websocket.send(message_str), timeout=5.0)
            self.stats['messages_sent'] += 1
            return True
        except websockets.exceptions.ConnectionClosed:
            self.remove_client(websocket)
            return False
        except asyncio.TimeoutError:
            print(f"Timeout sending to WebSocket client")
            self.remove_client(websocket)
            self.stats['errors'] += 1
            return False
        except Exception as e:
            print(f"Error sending to WebSocket client: {e}")
            self.remove_client(websocket)
            self.stats['errors'] += 1
            return False

    def get_stats(self):
        """Get WebSocket manager statistics"""
        return self.stats.copy()
    
    def get_client_count(self):
        """Get current number of connected clients"""
        return len(self.connected_clients)

class MessageBatchProcessor:
    """
    Processes batched messages from optimized WebSocket clients.
    Reduces server load and improves performance by handling multiple messages efficiently.
    """
    
    def __init__(self, key_state_manager=None, publisher_node=None):
        self.key_state_manager = key_state_manager
        self.publisher_node = publisher_node
        
        # Batch processing settings
        self.max_batch_size = 50
        self.batch_timeout_ms = 100
        self.key_batch_debounce_ms = 16  # ~60fps key processing
        
        # Key state optimization
        self.key_state_buffer = {}
        self.last_key_flush_time = 0
        self.key_flush_timer = None
        
        # Statistics
        self.stats = {
            'batches_processed': 0,
            'messages_in_batches': 0,
            'key_batches_processed': 0,
            'key_state_changes': 0,
            'processing_time_avg': 0
        }
        
        print("MessageBatchProcessor initialized")

    async def process_message_batch(self, batch_data: Dict[str, Any], websocket) -> Dict[str, Any]:
        """Process a batch of messages from the client."""
        start_time = time.time()
        
        try:
            messages = batch_data.get('messages', [])
            batch_id = batch_data.get('batch_id', 'unknown')
            
            if len(messages) > self.max_batch_size:
                print(f"Batch size {len(messages)} exceeds maximum {self.max_batch_size}")
                messages = messages[:self.max_batch_size]
            
            # Process messages by type for efficiency
            results = await self._process_messages_by_type(messages, websocket)
            
            # Update statistics
            processing_time = (time.time() - start_time) * 1000
            self.stats['batches_processed'] += 1
            self.stats['messages_in_batches'] += len(messages)
            self.stats['processing_time_avg'] = (
                (self.stats['processing_time_avg'] * 0.9) + 
                (processing_time * 0.1)
            )
            
            return {
                'type': 'batch_response',
                'batch_id': batch_id,
                'success': True,
                'messages_processed': len(messages),
                'processing_time_ms': round(processing_time, 2),
                'results': results,
                'timestamp': int(time.time() * 1000)
            }
            
        except Exception as e:
            print(f"Error processing message batch: {e}")
            return {
                'type': 'batch_response',
                'batch_id': batch_data.get('batch_id', 'unknown'),
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }

    async def _process_messages_by_type(self, messages: List[Dict], websocket) -> Dict[str, Any]:
        """Group and process messages by type for maximum efficiency."""
        # Group messages by type
        message_groups = {}
        for msg in messages:
            msg_type = msg.get('type', 'unknown')
            if msg_type not in message_groups:
                message_groups[msg_type] = []
            message_groups[msg_type].append(msg)
        
        results = {}
        
        # Process each type optimally
        for msg_type, type_messages in message_groups.items():
            if msg_type == 'key_batch':
                results[msg_type] = await self._process_key_batch_messages(type_messages)
            elif msg_type == 'keyboard':
                results[msg_type] = await self._process_keyboard_messages(type_messages)
            elif msg_type == 'ping':
                results[msg_type] = await self._process_ping_messages(type_messages, websocket)
            else:
                # Process other message types individually
                results[msg_type] = await self._process_individual_messages(type_messages, websocket)
        
        return results

    async def _process_key_batch_messages(self, messages: List[Dict]) -> Dict[str, Any]:
        """Process batched key state changes efficiently."""
        if not self.key_state_manager:
            return {'processed': 0, 'error': 'No key state manager available'}
        
        total_changes = 0
        key_state_updates = {}
        
        # Consolidate all key changes from all messages
        for msg in messages:
            changes = msg.get('changes', [])
            for change in changes:
                key_code = change.get('key_code')
                pressed = change.get('pressed', False)
                timestamp = change.get('timestamp', time.time() * 1000)
                
                if key_code is not None:
                    # Keep only the most recent state for each key
                    if key_code not in key_state_updates or timestamp > key_state_updates[key_code]['timestamp']:
                        key_state_updates[key_code] = {
                            'pressed': pressed,
                            'timestamp': timestamp
                        }
                    total_changes += 1
        
        # Apply consolidated key states
        state_changes_applied = 0
        for key_code, state in key_state_updates.items():
            if self.key_state_manager.update_key(key_code, state['pressed']):
                state_changes_applied += 1
        
        # Publish movement if any keys changed
        if state_changes_applied > 0:
            twist = self.key_state_manager.get_twist_if_changed()
            if twist is not None and self.publisher_node:
                self.publisher_node.twist_publisher.publish(twist)
                
            # Handle servo changes
            if hasattr(self.key_state_manager, 'servo_changed') and self.key_state_manager.servo_changed:
                servo_msg = Int32()
                servo_msg.data = self.key_state_manager.servo_position
                if self.publisher_node and hasattr(self.publisher_node, 'servo_publisher'):
                    self.publisher_node.servo_publisher.publish(servo_msg)
                self.key_state_manager.servo_changed = False
        
        self.stats['key_batches_processed'] += len(messages)
        self.stats['key_state_changes'] += state_changes_applied
        
        return {
            'processed': len(messages),
            'total_changes': total_changes,
            'state_changes_applied': state_changes_applied,
            'movement_published': state_changes_applied > 0
        }

    async def _process_keyboard_messages(self, messages: List[Dict]) -> Dict[str, Any]:
        """Process individual keyboard messages in batch."""
        if not self.key_state_manager:
            return {'processed': 0, 'error': 'No key state manager available'}
        
        processed = 0
        movement_updates = 0
        
        for msg in messages:
            key_code = msg.get('key_code', 0)
            is_held = msg.get('is_held', False)
            
            if self.key_state_manager.update_key(key_code, not is_held):  # is_held means key up
                movement_updates += 1
            processed += 1
        
        # Publish final movement state
        if movement_updates > 0:
            twist = self.key_state_manager.get_twist_if_changed()
            if twist is not None and self.publisher_node:
                self.publisher_node.twist_publisher.publish(twist)
        
        return {
            'processed': processed,
            'movement_updates': movement_updates,
            'movement_published': movement_updates > 0
        }

    async def _process_ping_messages(self, messages: List[Dict], websocket) -> Dict[str, Any]:
        """Process ping messages efficiently by sending batched pongs."""
        pong_responses = []
        current_time = int(time.time() * 1000)
        
        for msg in messages:
            pong_responses.append({
                'type': 'pong',
                'timestamp': current_time,
                'original_timestamp': msg.get('timestamp', current_time)
            })
        
        # Send all pong responses
        try:
            for pong in pong_responses:
                await websocket.send(json.dumps(pong))
        except Exception as e:
            print(f"Error sending pong responses: {e}")
        
        return {
            'processed': len(messages),
            'pongs_sent': len(pong_responses)
        }

    async def _process_individual_messages(self, messages: List[Dict], websocket) -> Dict[str, Any]:
        """Process other message types individually."""
        processed = 0
        errors = 0
        
        for msg in messages:
            try:
                # Placeholder for individual message processing
                processed += 1
            except Exception as e:
                print(f"Error processing individual message: {e}")
                errors += 1
        
        return {
            'processed': processed,
            'errors': errors
        }

    def get_stats(self) -> Dict[str, Any]:
        """Get batch processing statistics."""
        return {
            **self.stats,
            'key_buffer_size': len(self.key_state_buffer),
            'avg_processing_time_ms': round(self.stats['processing_time_avg'], 2)
        }

class OptimizedWebSocketHandler:
    """WebSocket handler that integrates with MessageBatchProcessor for improved performance."""
    
    def __init__(self, key_state_manager=None, publisher_node=None):
        self.batch_processor = MessageBatchProcessor(key_state_manager, publisher_node)
        self.fallback_handlers = {}  # For non-batch message handling
        
    def set_fallback_handler(self, message_type: str, handler: Callable):
        """Set fallback handler for non-batched message types."""
        self.fallback_handlers[message_type] = handler
    
    async def handle_message(self, data: Dict[str, Any], websocket) -> Optional[Dict[str, Any]]:
        """Main message handler that routes to batch processor or fallback handlers."""
        message_type = data.get('type', 'unknown')
        
        try:
            if message_type == 'message_batch':
                # Handle batched messages
                response = await self.batch_processor.process_message_batch(data, websocket)
                await websocket.send(json.dumps(response))
                return response
                
            elif message_type == 'key_batch':
                # Handle key batch messages directly
                messages = [data]  # Wrap single message as batch
                result = await self.batch_processor._process_key_batch_messages(messages)
                response = {
                    'type': 'key_batch_response',
                    'success': True,
                    'result': result,
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(response))
                return response
                
            else:
                # Use fallback handlers for individual messages
                if message_type in self.fallback_handlers:
                    return await self.fallback_handlers[message_type](data, websocket)
                else:
                    return None  # Let original handler process
                    
        except Exception as e:
            print(f"Error handling message type {message_type}: {e}")
            error_response = {
                'type': 'error',
                'success': False,
                'error': str(e),
                'message_type': message_type,
                'timestamp': int(time.time() * 1000)
            }
            try:
                await websocket.send(json.dumps(error_response))
            except:
                pass  # Connection might be closed
            return error_response
    
    def get_stats(self) -> Dict[str, Any]:
        """Get comprehensive statistics from the batch processor."""
        return self.batch_processor.get_stats()
    
    async def cleanup(self):
        """Cleanup method to flush any pending operations."""
        if hasattr(self.batch_processor, 'flush_key_state_buffer'):
            await self.batch_processor.flush_key_state_buffer()

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
        self.key_state_manager = KeyStateManager()
        
        # Initialize WebSocket optimization
        self.optimization_enabled = True
        self.optimized_handler = None
        
        try:
            self.optimized_handler = OptimizedWebSocketHandler(
                key_state_manager=self.key_state_manager,
                publisher_node=self.publisher_node
            )
            print("✅ WebSocket optimization enabled")
        except Exception as e:
            print(f"⚠️  Failed to initialize WebSocket optimization: {e}")
            self.optimization_enabled = False

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
                    'connected_clients': len(self.websocket_manager.connected_clients),
                    'uptime': int(time.time() - self.server_start_time),
                    'version': '1.0.0',
                    'current_host': network_info.get('hostname', 'localhost'),
                    'current_url': request.url,
                    'protocol': request.scheme,
                    'api_base': f"http://{network_info.get('hostname', 'localhost')}:8080",
                    'web_url': f"http://{network_info.get('hostname', 'localhost')}:8080",
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
                    'connected_clients': len(self.websocket_manager.connected_clients),
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
                            'connected_clients': len(self.websocket_manager.connected_clients),
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
                    'connected_clients': len(self.websocket_manager.connected_clients),
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

        # Optimization routes
        @self.app.route('/optimized')
        def optimized_interface():
            """Serve the optimized WebSocket interface"""
            try:
                template_dir = os.path.dirname(__file__)
                template_path = os.path.join(template_dir, 'templates', 'index-optimized.html')
                if os.path.exists(template_path):
                    with open(template_path, 'r') as f:
                        template_content = f.read()
                    return template_content
                else:
                    # Fallback to standard interface
                    return serve_index()
            except Exception as e:
                print(f"Error serving optimized interface: {e}")
                return serve_index()
        
        @self.app.route('/api/optimization/stats')
        def optimization_stats():
            """Get optimization performance statistics"""
            if self.optimization_enabled and self.optimized_handler:
                stats = self.optimized_handler.get_stats()
                websocket_stats = self.websocket_manager.get_stats()
                return jsonify({
                    'optimization_enabled': True,
                    'batch_stats': stats,
                    'websocket_stats': websocket_stats,
                    'timestamp': int(time.time() * 1000)
                })
            else:
                return jsonify({
                    'optimization_enabled': False,
                    'message': 'WebSocket optimization not available',
                    'timestamp': int(time.time() * 1000)
                })
        
        @self.app.route('/api/optimization/config', methods=['GET', 'POST'])
        def optimization_config():
            """Get or set optimization configuration"""
            if request.method == 'GET':
                if self.optimization_enabled and self.optimized_handler:
                    return jsonify({
                        'batch_size': self.optimized_handler.batch_processor.max_batch_size,
                        'batch_timeout': self.optimized_handler.batch_processor.batch_timeout_ms,
                        'key_debounce': self.optimized_handler.batch_processor.key_batch_debounce_ms
                    })
                else:
                    return jsonify({'error': 'Optimization not enabled'})
            
            elif request.method == 'POST':
                if self.optimization_enabled and self.optimized_handler:
                    config = request.json
                    processor = self.optimized_handler.batch_processor
                    
                    if 'batch_size' in config:
                        processor.max_batch_size = min(max(config['batch_size'], 1), 100)
                    if 'batch_timeout' in config:
                        processor.batch_timeout_ms = min(max(config['batch_timeout'], 10), 1000)
                    if 'key_debounce' in config:
                        processor.key_batch_debounce_ms = min(max(config['key_debounce'], 1), 100)
                    
                    return jsonify({'success': True, 'message': 'Configuration updated'})
                else:
                    return jsonify({'error': 'Optimization not enabled'})

        # Serve optimized JavaScript files
        @self.app.route('/static/js/keyboard-optimized.js')
        def serve_optimized_js():
            """Serve the optimized keyboard JavaScript file"""
            static_dir = os.path.join(os.path.dirname(__file__), 'static')
            js_path = os.path.join(static_dir, 'js', 'keyboard-optimized.js')
            if os.path.exists(js_path):
                return send_from_directory(os.path.join(static_dir, 'js'), 'keyboard-optimized.js')
            else:
                # Return a placeholder if optimized version doesn't exist
                return "console.log('Optimized WebSocket client not found. Using fallback.');", 200, {'Content-Type': 'application/javascript'}

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
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_publisher = self.create_publisher(Int32, 'servo_position', 10)
        self.led_publisher = self.create_publisher(Bool, 'toggle_led', 10)
        self.led_status_subscriber = self.create_subscription(Bool, 'led_status', self.led_status_callback, 10)

        # Optimized timer - only publish when key changes
        timer_period = 0.05  # Reduced frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize key state manager
        self.key_manager = KeyStateManager(debounce_time=0.02)
        self.network_cache = NetworkInfoCache(cache_duration=60)
        self.flask_app = None
        self.api_handler = None
        self.websocket_server = None
        self.led_state = False
        self.optimization_enabled = False
        self.optimized_handler = None

        # Check if ports are available
        self.check_ports()

        # Start the REST API server and WebSocket server
        self.start_rest_api_server()
        self.start_websocket_server()

        print("=" * 50)
        print("ROS2 REST API Keyboard Interface Started")
        print("=" * 50)
        print("Services started on all network interfaces (0.0.0.0):")
        print(f"  Web interface: http://localhost:8080")
        print(f"  REST API: http://localhost:8080")
        print(f"  WebSocket: ws://localhost:8765")
        print("")
        print("Network access URLs (replace <your-ip> with actual IP):")
        print(f"  Web interface: http://<your-ip>:8080")
        print(f"  REST API: http://<your-ip>:8080")
        print(f"  WebSocket: ws://<your-ip>:8765")
        print("")
        print("Features enabled:")
        print("  ✓ Real-time WebSocket communication")
        print("  ✓ Key state debouncing to prevent duplicate ROS2 publications")
        print("  ✓ Network info caching to reduce system calls")
        print("  ✓ Multi-key support for combined movements")
        print("Press WASD keys for movement, F key for servo toggle, L key for LED toggle")
        print("=" * 50)
        self.print_network_info()

    def check_ports(self):
        """Check if ports 8000, 8080, and 8765 are available"""
        def is_port_available(port):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', port))
                return True
            except socket.error:
                return False

        # Port 8000 is not used - web interface runs on port 8080
        if not is_port_available(8080):
            print("WARNING: Port 8080 is already in use. REST API server may not start.")
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
                        print(f"  Web interface: http://{ip}:8080")
                        print(f"  REST API: http://{ip}:8080")
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
                self.api_handler = api_handler

                # Ensure HTML file is created (for backwards compatibility)
                self.create_html_file()

                print("✓ REST API server starting on 0.0.0.0:8080")
                print("✓ Mako templates loaded from bocchi/templates/")
                print("✓ Static files served from bocchi/static/")
                print("✓ Real-time WebSocket communication enabled")
                print("✓ HTMX integration enabled")
                print("✓ Event batching and key debouncing enabled")

                # Run Flask server
                self.flask_app.run(
                    host='0.0.0.0',
                    port=8080,
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
        """Start the enhanced WebSocket server with Docker container compatibility"""
        def run_websocket_server():
            try:
                print("Initializing enhanced WebSocket server...")
                
                # Ensure we have a clean event loop
                try:
                    loop = asyncio.get_event_loop()
                    if loop.is_closed():
                        loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(loop)
                except RuntimeError:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                
                async def handle_websocket_connection(websocket, path=None):
                    """Enhanced WebSocket connection handler with robust error handling"""
                    api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
                    client_address = getattr(websocket, 'remote_address', 'unknown')
                    client_id = f"{client_address}_{int(time.time()*1000)}"
                    
                    print(f"WebSocket client connected from {client_address}")
                    
                    # Add client to manager
                    if api_handler:
                        api_handler.websocket_manager.add_client(websocket)
                        api_handler.websocket_manager.stats['messages_received'] += 1
                    
                    try:
                        # Send enhanced welcome message
                        welcome_msg = {
                            'type': 'welcome',
                            'message': 'Connected to bocchi robot controller',
                            'client_id': client_id,
                            'server_time': time.strftime('%Y-%m-%d %H:%M:%S'),
                            'timestamp': int(time.time() * 1000),
                            'version': '1.0.0',
                            'features': ['keyboard_control', 'status_monitoring', 'batch_commands']
                        }
                        
                        # Send welcome with timeout protection
                        await asyncio.wait_for(websocket.send(json.dumps(welcome_msg)), timeout=5.0)
                        
                        # Main message handling loop with enhanced error handling
                        async for message in websocket:
                            try:
                                # Update message stats
                                if api_handler:
                                    api_handler.websocket_manager.stats['messages_received'] += 1
                                
                                data = json.loads(message)
                                message_type = data.get('type', 'unknown')
                                
                                print(f"Received WebSocket message type '{message_type}' from {client_address}")
                                
                                # Enhanced message routing with optimization support
                                try:
                                    # Try optimized handler first
                                    if self.optimization_enabled and self.optimized_handler:
                                        optimized_response = await self.optimized_handler.handle_message(data, websocket)
                                        if optimized_response:
                                            continue  # Message was handled by optimization layer
                                    
                                    # Original message routing for non-optimized messages
                                    if message_type == 'keyboard':
                                        await self.handle_websocket_keyboard_input(data, websocket)
                                    elif message_type == 'key_down':
                                        await self.handle_websocket_key_down(data, websocket)
                                    elif message_type == 'key_up':
                                        await self.handle_websocket_key_up(data, websocket)
                                    elif message_type == 'get_status':
                                        await self.handle_websocket_status_request(data, websocket)
                                    elif message_type == 'test_connection':
                                        await self.handle_websocket_test_connection(data, websocket)
                                    elif message_type == 'send_key':
                                        await self.handle_websocket_send_key(data, websocket)
                                    elif message_type == 'send_key_batch':
                                        await self.handle_websocket_send_key_batch(data, websocket)
                                    elif message_type == 'get_routes':
                                        await self.handle_websocket_get_routes(data, websocket)
                                    elif message_type == 'ping':
                                        # Handle ping/pong for connection monitoring
                                        pong_response = {
                                            'type': 'pong',
                                            'timestamp': int(time.time() * 1000)
                                        }
                                        if api_handler:
                                            await api_handler.websocket_manager.send_to_client(websocket, pong_response)
                                        else:
                                            await websocket.send(json.dumps(pong_response))
                                    elif message_type == 'client_connected':
                                        print(f"Client connected: {data.get('client', 'unknown')}")
                                    else:
                                        await self.handle_websocket_unknown_message(data, websocket)
                                        
                                except Exception as handler_error:
                                    print(f"Error in message handler for {message_type}: {handler_error}")
                                    error_response = {
                                        'type': 'handler_error',
                                        'message': f'Error processing {message_type}',
                                        'error': str(handler_error),
                                        'timestamp': int(time.time() * 1000)
                                    }
                                    if api_handler:
                                        await api_handler.websocket_manager.send_to_client(websocket, error_response)
                                        api_handler.websocket_manager.stats['errors'] += 1
                                    else:
                                        await websocket.send(json.dumps(error_response))
                                    
                            except json.JSONDecodeError as json_error:
                                print(f"Invalid JSON received from WebSocket client {client_address}: {json_error}")
                                error_response = {
                                    'type': 'json_error',
                                    'message': 'Invalid JSON format',
                                    'error': str(json_error),
                                    'timestamp': int(time.time() * 1000)
                                }
                                if api_handler:
                                    await api_handler.websocket_manager.send_to_client(websocket, error_response)
                                    api_handler.websocket_manager.stats['errors'] += 1
                                else:
                                    await websocket.send(json.dumps(error_response))
                            except Exception as msg_error:
                                print(f"Error processing message from {client_address}: {msg_error}")
                                if api_handler:
                                    api_handler.websocket_manager.stats['errors'] += 1
                                
                    except websockets.exceptions.ConnectionClosed:
                        print(f"WebSocket client {client_address} disconnected normally")
                    except websockets.exceptions.WebSocketException as ws_error:
                        print(f"WebSocket protocol error for client {client_address}: {ws_error}")
                        if api_handler:
                            api_handler.websocket_manager.stats['errors'] += 1
                    except Exception as e:
                        print(f"Unexpected WebSocket error for client {client_address}: {e}")
                        if api_handler:
                            api_handler.websocket_manager.stats['errors'] += 1
                        import traceback
                        traceback.print_exc()
                    finally:
                        # Enhanced cleanup
                        print(f"Cleaning up WebSocket client {client_address}")
                        if api_handler:
                            api_handler.websocket_manager.remove_client(websocket)
                
                async def start_server():
                    try:
                        # Enhanced server configuration for Docker container compatibility
                        import websockets
                        
                        # Debug: Print websockets version
                        version_info = getattr(websockets, '__version__', 'unknown')
                        print(f"WebSocket library version: {version_info}")
                        
                        # Try newer websockets API first, fallback to older API
                        try:
                            # Try with single parameter handler (newer versions)
                            async def websocket_handler(websocket):
                                return await handle_websocket_connection(websocket, None)
                            
                            print("Attempting to start WebSocket server with new handler signature...")
                            server = await websockets.serve(
                                websocket_handler,
                                "0.0.0.0",  # Bind to all interfaces for Docker compatibility
                                8765,
                                ping_interval=20,       # Keep connections alive
                                ping_timeout=10,        # Timeout for ping responses
                                max_size=1024*1024,     # 1MB max message size
                                max_queue=32,           # Max queued messages per client
                                compression=None,       # Disable compression for lower latency
                                write_limit=1024*64     # 64KB write buffer limit
                            )
                            print("✓ WebSocket server started with new handler signature")
                            
                        except TypeError as e:
                            print(f"New handler failed ({e}), trying legacy handler signature...")
                            # Fallback to older websockets API (with path parameter)
                            server = await websockets.serve(
                                handle_websocket_connection,
                                "0.0.0.0",  # Bind to all interfaces for Docker compatibility
                                8765,
                                ping_interval=20,       # Keep connections alive
                                ping_timeout=10,        # Timeout for ping responses
                                max_size=1024*1024,     # 1MB max message size
                                max_queue=32,           # Max queued messages per client
                                write_limit=1024*64,    # 64KB write buffer limit
                                compression=None        # Disable compression for lower latency
                            )
                            print("✓ WebSocket server started with legacy handler signature")
                        
                        print("✓ Enhanced WebSocket server started successfully on 0.0.0.0:8765")
                        print("✓ WebSocket URL: ws://0.0.0.0:8765")
                        print("✓ Docker container compatible configuration active")
                        
                        # Keep server running
                        await server.wait_closed()
                        
                    except Exception as e:
                        print(f"✗ Failed to start WebSocket server: {e}")
                        import traceback
                        traceback.print_exc()
                        raise
                
                # Run the enhanced WebSocket server
                try:
                    loop.run_until_complete(start_server())
                except Exception as server_error:
                    print(f"✗ WebSocket server error: {server_error}")
                    import traceback
                    traceback.print_exc()
                    
            except Exception as e:
                print(f"✗ WebSocket server thread error: {e}")
                import traceback
                traceback.print_exc()
            finally:
                try:
                    if 'loop' in locals():
                        if not loop.is_closed():
                            loop.close()
                except:
                    pass

        print("Starting enhanced WebSocket server thread...")
        self.websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
        self.websocket_thread.start()
        
        # Give the WebSocket server more time to properly initialize
        print("Waiting for enhanced WebSocket server to initialize...")
        time.sleep(2)
        print("✓ WebSocket server initialization complete")

    async def handle_websocket_keyboard_input(self, data, websocket):
        """Handle keyboard input received via WebSocket"""
        try:
            key = data.get('key', '')
            key_code = data.get('key_code', 0)
            is_held = data.get('is_held', False)
            timestamp = data.get('timestamp', int(time.time() * 1000))
            request_id = data.get('request_id')
            
            # Process the key input using the existing key state manager
            api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
            if not api_handler:
                raise Exception("API handler not available")
            
            # Update key state
            should_publish = api_handler.key_state_manager.update_key(key_code, True)
            
            response_data = {
                'type': 'keyboard_response',
                'success': True,
                'key': key,
                'key_code': key_code,
                'is_held': is_held,
                'timestamp': timestamp,
                'published': should_publish
            }
            
            if request_id:
                response_data['request_id'] = request_id
            
            if should_publish:
                # Publish ROS2 messages based on key
                self.process_key_input(key, key_code)
                
                # Broadcast the key event to all WebSocket clients
                event_data = {
                    'type': 'key_event',
                    'key': key,
                    'key_code': key_code,
                    'is_held': is_held,
                    'timestamp': timestamp,
                    'action': 'pressed'
                }
                
                # Schedule async broadcast
                api_handler._schedule_broadcast(event_data)
                
                print(f"WebSocket keyboard input processed: {key} ({key_code})")
            
            # Send response back to requesting client
            await websocket.send(json.dumps(response_data))
                
        except Exception as e:
            error_response = {
                'type': 'keyboard_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))
            print(f"Error handling WebSocket keyboard input: {e}")

    async def handle_websocket_key_down(self, data, websocket):
        """Handle key down event received via WebSocket"""
        try:
            key = data.get('key', '')
            key_code = data.get('key_code', 0)
            timestamp = data.get('timestamp', int(time.time() * 1000))
            request_id = data.get('request_id')
            
            # Process key down
            api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
            if not api_handler:
                raise Exception("API handler not available")
            should_publish = api_handler.key_state_manager.update_key(key_code, True)
            
            response_data = {
                'type': 'key_down_response',
                'success': True,
                'key': key,
                'key_code': key_code,
                'timestamp': timestamp,
                'published': should_publish
            }
            
            if request_id:
                response_data['request_id'] = request_id
            
            if should_publish:
                self.process_key_input(key, key_code)
                
                # Broadcast key down event
                event_data = {
                    'type': 'key_down_event',
                    'key': key,
                    'key_code': key_code,
                    'timestamp': timestamp
                }
                api_handler._schedule_broadcast(event_data)
                
                print(f"WebSocket key down: {key} ({key_code})")
            
            # Send response back to requesting client
            await websocket.send(json.dumps(response_data))
                
        except Exception as e:
            error_response = {
                'type': 'key_down_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))
            print(f"Error handling WebSocket key down: {e}")

    async def handle_websocket_key_up(self, data, websocket):
        """Handle key up event received via WebSocket"""
        try:
            key = data.get('key', '')
            key_code = data.get('key_code', 0)
            timestamp = data.get('timestamp', int(time.time() * 1000))
            request_id = data.get('request_id')
            
            # Process key up (release)
            api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
            if not api_handler:
                raise Exception("API handler not available")
            should_publish = api_handler.key_state_manager.update_key(key_code, False)
            
            response_data = {
                'type': 'key_up_response',
                'success': True,
                'key': key,
                'key_code': key_code,
                'timestamp': timestamp,
                'published': should_publish
            }
            
            if request_id:
                response_data['request_id'] = request_id
            
            if should_publish:
                # Publish stop command or update movement based on remaining keys
                self.process_key_release(key, key_code)
                
                # Broadcast key up event
                event_data = {
                    'type': 'key_up_event',
                    'key': key,
                    'key_code': key_code,
                    'timestamp': timestamp
                }
                api_handler._schedule_broadcast(event_data)
                
                print(f"WebSocket key up: {key} ({key_code})")
            
            # Send response back to requesting client
            await websocket.send(json.dumps(response_data))
                
        except Exception as e:
            error_response = {
                'type': 'key_up_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))
            print(f"Error handling WebSocket key up: {e}")

    async def handle_websocket_status_request(self, data, websocket):
        """Enhanced status request handler with detailed WebSocket statistics"""
        try:
            request_id = data.get('request_id')
            api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
            if not api_handler:
                raise Exception("API handler not available")
            
            # Get enhanced WebSocket statistics
            ws_stats = api_handler.websocket_manager.get_stats()
            uptime = int(time.time() - api_handler.server_start_time)
            
            # Gather comprehensive status information
            status_data = {
                'type': 'status_response',
                'success': True,
                'data': {
                    'server_running': True,
                    'version': '1.0.0',
                    'uptime': uptime,
                    'uptime_human': f"{uptime // 3600}h {(uptime % 3600) // 60}m {uptime % 60}s",
                    'timestamp': int(time.time() * 1000),
                    'server_time': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'websocket_mode': True,
                    'websocket': {
                        'connected_clients': api_handler.websocket_manager.get_client_count(),
                        'stats': ws_stats,
                        'server_url': 'ws://0.0.0.0:8765',
                        'features': ['keyboard_control', 'status_monitoring', 'batch_commands', 'ping_pong']
                    },
                    'ros2': {
                        'twist_publisher_active': hasattr(self, 'twist_publisher') and self.twist_publisher is not None,
                        'servo_publisher_active': hasattr(self, 'servo_publisher') and self.servo_publisher is not None,
                        'led_state': self.led_state,
                        'node_name': self.get_name() if hasattr(self, 'get_name') else 'bocchi_publisher'
                    },
                    'system': {
                        'platform': 'docker_container',
                        'threads_active': threading.active_count(),
                        'websocket_thread_alive': hasattr(self, 'websocket_thread') and self.websocket_thread.is_alive()
                    }
                },
                'timestamp': int(time.time() * 1000)
            }
            
            if request_id:
                status_data['request_id'] = request_id
                
            # Use enhanced send method if available
            if hasattr(api_handler.websocket_manager, 'send_to_client'):
                await api_handler.websocket_manager.send_to_client(websocket, status_data)
            else:
                await websocket.send(json.dumps(status_data))
                
            print(f"WebSocket status request processed successfully")
            
        except Exception as e:
            error_response = {
                'type': 'status_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))

    async def handle_websocket_test_connection(self, data, websocket):
        """Enhanced connection test handler with comprehensive diagnostics"""
        try:
            request_id = data.get('request_id')
            client_timestamp = data.get('timestamp', 0)
            server_timestamp = int(time.time() * 1000)
            
            # Calculate connection metrics
            latency_ms = max(0, server_timestamp - client_timestamp) if client_timestamp > 0 else 0
            
            # Get WebSocket manager stats
            api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
            ws_stats = api_handler.websocket_manager.get_stats() if api_handler else {}
            
            response_data = {
                'type': 'test_connection_response',
                'success': True,
                'message': 'Enhanced WebSocket connection test successful',
                'server_time': server_timestamp,
                'client_time': client_timestamp,
                'latency_ms': latency_ms,
                'connection_quality': 'excellent' if latency_ms < 50 else 'good' if latency_ms < 100 else 'fair',
                'timestamp': server_timestamp,
                'diagnostics': {
                    'websocket_stats': ws_stats,
                    'server_responsive': True,
                    'docker_container': True,
                    'protocol_version': 'ws/13',
                    'compression': 'disabled',
                    'max_message_size': '1MB'
                }
            }
            
            if request_id:
                response_data['request_id'] = request_id
                
            await websocket.send(json.dumps(response_data))
            
        except Exception as e:
            error_response = {
                'type': 'test_connection_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))

    async def handle_websocket_send_key(self, data, websocket):
        """Handle single key send via WebSocket"""
        try:
            key = data.get('key', '')
            key_code = data.get('key_code', 0)
            is_held = data.get('is_held', False)
            request_id = data.get('request_id')
            
            # Use existing keyboard input handler
            await self.handle_websocket_keyboard_input(data, websocket)
            
        except Exception as e:
            error_response = {
                'type': 'send_key_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))

    async def handle_websocket_send_key_batch(self, data, websocket):
        """Handle batch key send via WebSocket"""
        try:
            keys_input = data.get('keys_input', '')
            request_id = data.get('request_id')
            
            # Parse key batch (format: key1:code1,key2:code2)
            processed_keys = []
            if keys_input:
                for key_pair in keys_input.split(','):
                    if ':' in key_pair:
                        key, code_str = key_pair.strip().split(':', 1)
                        try:
                            key_code = int(code_str)
                            processed_keys.append({'key': key, 'key_code': key_code})
                        except ValueError:
                            continue
            
            # Process each key
            for key_data in processed_keys:
                key_msg = {
                    'type': 'keyboard',
                    'key': key_data['key'],
                    'key_code': key_data['key_code'],
                    'is_held': False,
                    'timestamp': int(time.time() * 1000)
                }
                await self.handle_websocket_keyboard_input(key_msg, websocket)
            
            response_data = {
                'type': 'send_key_batch_response',
                'success': True,
                'processed_count': len(processed_keys),
                'keys': processed_keys,
                'timestamp': int(time.time() * 1000)
            }
            
            if request_id:
                response_data['request_id'] = request_id
                
            await websocket.send(json.dumps(response_data))
            
        except Exception as e:
            error_response = {
                'type': 'send_key_batch_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))

    async def handle_websocket_get_routes(self, data, websocket):
        """Handle route information request via WebSocket"""
        try:
            request_id = data.get('request_id')
            
            # Get Flask routes for compatibility
            api_handler = self.flask_app.config['api_handler']
            routes = []
            for rule in api_handler.app.url_map.iter_rules():
                routes.append({
                    'rule': rule.rule,
                    'endpoint': rule.endpoint,
                    'methods': list(rule.methods)
                })
            
            response_data = {
                'type': 'routes_response',
                'success': True,
                'routes': routes,
                'websocket_enabled': True,
                'timestamp': int(time.time() * 1000)
            }
            
            if request_id:
                response_data['request_id'] = request_id
                
            await websocket.send(json.dumps(response_data))
            
        except Exception as e:
            error_response = {
                'type': 'routes_response',
                'success': False,
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            if data.get('request_id'):
                error_response['request_id'] = data.get('request_id')
            await websocket.send(json.dumps(error_response))

    async def handle_websocket_unknown_message(self, data, websocket):
        """Enhanced unknown message handler with helpful suggestions"""
        try:
            request_id = data.get('request_id')
            message_type = data.get('type', 'unknown')
            
            # List of supported message types
            supported_types = [
                'keyboard', 'key_down', 'key_up', 'get_status', 'test_connection',
                'send_key', 'send_key_batch', 'get_routes', 'ping', 'client_connected'
            ]
            
            # Find similar message types (basic similarity check)
            suggestions = []
            if message_type and message_type != 'unknown':
                for supported in supported_types:
                    if message_type.lower() in supported.lower() or supported.lower() in message_type.lower():
                        suggestions.append(supported)
            
            response_data = {
                'type': 'unknown_message_error',
                'success': False,
                'error': f'Unknown message type: {message_type}',
                'received_type': message_type,
                'supported_types': supported_types,
                'suggestions': suggestions[:3],  # Limit to 3 suggestions
                'help': 'Use test_connection to verify connectivity or get_status for server information',
                'timestamp': int(time.time() * 1000)
            }
            
            if request_id:
                response_data['request_id'] = request_id
                
            # Use enhanced send method if available
            api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
            if api_handler and hasattr(api_handler.websocket_manager, 'send_to_client'):
                await api_handler.websocket_manager.send_to_client(websocket, response_data)
                api_handler.websocket_manager.stats['errors'] += 1
            else:
                await websocket.send(json.dumps(response_data))
                
            print(f"Unknown WebSocket message type '{message_type}' - suggested: {suggestions}")
            
        except Exception as e:
            print(f"Error handling unknown WebSocket message: {e}")

    def process_key_input(self, key, key_code):
        """Process key input and publish appropriate ROS2 messages"""
        try:
            # Movement keys
            if key_code == 87:  # W - Forward
                self.publish_twist(0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
            elif key_code == 83:  # S - Backward
                self.publish_twist(-0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
            elif key_code == 65:  # A - Turn Left
                self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.5)
            elif key_code == 68:  # D - Turn Right
                self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, -0.5)
            elif key_code == 70:  # F - Servo toggle
                api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
                if not api_handler:
                    return
                servo_pos = api_handler.key_state_manager.servo_position
                self.publish_servo(servo_pos)
            elif key_code == 76 or key_code == 108:  # L or l - LED toggle
                self.led_state = not self.led_state
                self.publish_led_toggle(self.led_state)
            else:
                # Stop movement for other keys
                self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        except Exception as e:
            print(f"Error processing key input: {e}")

    def process_key_release(self, key, key_code):
        """Process key release and update robot movement"""
        try:
            api_handler = self.api_handler or (self.flask_app.config.get('api_handler') if self.flask_app else None)
            if not api_handler:
                return
            current_keys = api_handler.key_state_manager.current_keys

            # Determine movement based on remaining pressed keys
            linear_x = 0.0
            angular_z = 0.0

            if 87 in current_keys:  # W still pressed
                linear_x = 0.5
            elif 83 in current_keys:  # S still pressed
                linear_x = -0.5

            if 65 in current_keys:  # A still pressed
                angular_z = 0.5
            elif 68 in current_keys:  # D still pressed
                angular_z = -0.5

            # Publish updated movement
            self.publish_twist(linear_x, 0.0, 0.0, 0.0, 0.0, angular_z)

        except Exception as e:
            print(f"Error processing key release: {e}")

    def publish_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Publish Twist message to cmd_vel topic"""
        try:
            msg = Twist()
            msg.linear.x = linear_x
            msg.linear.y = linear_y
            msg.linear.z = linear_z
            msg.angular.x = angular_x
            msg.angular.y = angular_y
            msg.angular.z = angular_z
            self.twist_publisher.publish(msg)
        except Exception as e:
            print(f"Error publishing Twist message: {e}")

    def publish_servo(self, position):
        """Publish servo position message"""
        try:
            msg = Int32()
            msg.data = position
            self.servo_publisher.publish(msg)
        except Exception as e:
            print(f"Error publishing servo message: {e}")

    def publish_led_toggle(self, state):
        """Publish LED toggle message"""
        try:
            msg = Bool()
            msg.data = state
            self.led_publisher.publish(msg)
            print(f"Published LED toggle: {state}")
        except Exception as e:
            print(f"Error publishing LED toggle message: {e}")

    def led_status_callback(self, msg):
        """Callback for LED status updates"""
        self.led_state = msg.data
        print(f"Received LED status: {self.led_state}")

    def timer_callback(self):
        """Optimized timer callback - only publish when key state changes"""
        twist_msg = self.key_manager.get_twist_if_changed()
        if twist_msg is not None:
            self.twist_publisher.publish(twist_msg)
            self.get_logger().info(f'Publishing cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

        servo_position = self.key_manager.get_servo_if_changed()
        if servo_position is not None:
            servo_msg = Int32()
            servo_msg.data = servo_position
            self.servo_publisher.publish(servo_msg)
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
    
    try:
        rclpy.init(args=args)
        signal.signal(signal.SIGINT, signal_handler)

        minimal_publisher = MinimalPublisher()
        
        # Give servers time to initialize before spinning
        print("Waiting for servers to initialize...")
        time.sleep(3)
        
        print("Starting ROS2 node spin...")
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
            try:
                minimal_publisher.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass
        print("ROS2 node shutdown complete")

if __name__ == '__main__':
    main()
