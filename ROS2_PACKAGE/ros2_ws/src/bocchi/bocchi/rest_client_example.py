#!/usr/bin/env python3
"""
Example REST API client for testing the ROS2 keyboard publisher.
This demonstrates how to connect to and interact with the REST API keyboard service.
Replaces the previous gRPC client with HTTP requests and SSE for real-time updates.
"""

import requests
import time
import sys
import json
import threading
from datetime import datetime
import sseclient


class KeyboardClient:
    def __init__(self, server_address=None):
        """Initialize the REST API client"""
        if server_address is None:
            # Try to detect from environment or use localhost as fallback
            server_address = self.detect_server_address()
        
        self.server_address = server_address.rstrip('/')
        self.connected = False
        self.sse_client = None
        self.sse_thread = None
    
    def detect_server_address(self):
        """Detect the server address from environment or network"""
        import socket
        
        # Try to get the local IP address
        try:
            # Connect to a dummy address to get local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            
            # Test if server is running on local IP
            test_url = f'http://{local_ip}:5000'
            try:
                import requests
                response = requests.get(f'{test_url}/api/status', timeout=2)
                if response.status_code == 200:
                    print(f"Detected server at {test_url}")
                    return test_url
            except:
                pass
        except:
            pass
        
        # Fallback to localhost
        print("Using localhost as fallback")
        return 'http://localhost:5000'
        
    def connect(self):
        """Connect to the REST API server"""
        try:
            print(f"Connecting to REST API server at {self.server_address}...")
            
            # Test the connection by getting status
            response = self.get_status()
            if response:
                self.connected = True
                print(f"✓ Connected successfully!")
                return True
            else:
                self.connected = False
                return False
                
        except Exception as e:
            print(f"✗ Connection error: {e}")
            self.connected = False
            return False
    
    def send_key(self, key, key_code, is_held=False):
        """Send a single key press"""
        if not self.connected:
            print("Not connected to server")
            return False
            
        try:
            data = {
                'key': key,
                'key_code': key_code,
                'is_held': is_held,
                'timestamp': int(time.time() * 1000)
            }
            
            response = requests.post(
                f"{self.server_address}/api/key",
                json=data,
                timeout=5
            )
            
            result = response.json()
            
            if result.get('success', False):
                held_text = " (HELD)" if is_held else ""
                print(f"✓ Key sent: '{key}' (code: {key_code}){held_text} - {result.get('message', '')}")
                return True
            else:
                print(f"✗ Key failed: {result.get('error', 'Unknown error')}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"✗ Request error sending key: {e}")
            return False
        except Exception as e:
            print(f"✗ Error sending key: {e}")
            return False

    def send_key_down(self, key, key_code):
        """Send a key down event"""
        if not self.connected:
            print("Not connected to server")
            return False
            
        try:
            data = {
                'key': key,
                'key_code': key_code,
                'timestamp': int(time.time() * 1000)
            }
            
            response = requests.post(
                f"{self.server_address}/api/key/down",
                json=data,
                timeout=5
            )
            
            result = response.json()
            
            if result.get('success', False):
                print(f"✓ Key DOWN: '{key}' (code: {key_code}) - {result.get('message', '')}")
                return True
            else:
                print(f"✗ Key DOWN failed: {result.get('error', 'Unknown error')}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"✗ Request error sending key down: {e}")
            return False
        except Exception as e:
            print(f"✗ Error sending key down: {e}")
            return False

    def send_key_up(self, key, key_code):
        """Send a key up event"""
        if not self.connected:
            print("Not connected to server")
            return False
            
        try:
            data = {
                'key': key,
                'key_code': key_code,
                'timestamp': int(time.time() * 1000)
            }
            
            response = requests.post(
                f"{self.server_address}/api/key/up",
                json=data,
                timeout=5
            )
            
            result = response.json()
            
            if result.get('success', False):
                print(f"✓ Key UP: '{key}' (code: {key_code}) - {result.get('message', '')}")
                return True
            else:
                print(f"✗ Key UP failed: {result.get('error', 'Unknown error')}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"✗ Request error sending key up: {e}")
            return False
        except Exception as e:
            print(f"✗ Error sending key up: {e}")
            return False
    
    def send_keys_batch(self, keys_data):
        """Send multiple keys at once"""
        if not self.connected:
            print("Not connected to server")
            return False
            
        try:
            data = {'keys': keys_data}
            
            response = requests.post(
                f"{self.server_address}/api/keys/batch",
                json=data,
                timeout=10
            )
            
            result = response.json()
            
            if result.get('success', False):
                processed = result.get('processed_keys', [])
                print(f"✓ Batch sent: {len(processed)} keys processed - {result.get('message', '')}")
                return True
            else:
                print(f"✗ Batch failed: {result.get('error', 'Unknown error')}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"✗ Request error sending batch: {e}")
            return False
        except Exception as e:
            print(f"✗ Error sending batch: {e}")
            return False
    
    def get_status(self):
        """Get server status"""
        try:
            response = requests.get(f"{self.server_address}/api/status", timeout=5)
            result = response.json()
            
            if result.get('success', False):
                data = result.get('data', {})
                print(f"Server Status:")
                print(f"  Running: {data.get('server_running', False)}")
                print(f"  Connected clients: {data.get('connected_clients', 0)}")
                print(f"  Version: {data.get('version', 'Unknown')}")
                print(f"  Uptime: {data.get('uptime', 0)} seconds")
                return data
            else:
                print(f"✗ Status error: {result.get('error', 'Unknown error')}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"✗ Request error getting status: {e}")
            return None
        except Exception as e:
            print(f"✗ Status error: {e}")
            return None
    
    def start_sse_listener(self):
        """Start listening to Server-Sent Events"""
        if self.sse_thread and self.sse_thread.is_alive():
            print("SSE listener already running")
            return
            
        def sse_listener():
            try:
                print("Starting SSE listener...")
                
                # Use sseclient-py for better SSE handling
                try:
                    messages = sseclient.SSEClient(f"{self.server_address}/api/events")
                except NameError:
                    # Fallback if sseclient is not available
                    print("Warning: sseclient not available, using basic requests")
                    response = requests.get(f"{self.server_address}/api/events", stream=True)
                    
                    for line in response.iter_lines():
                        if line:
                            line = line.decode('utf-8')
                            if line.startswith('data: '):
                                data = line[6:]  # Remove 'data: ' prefix
                                try:
                                    event_data = json.loads(data)
                                    self.handle_sse_event(event_data)
                                except json.JSONDecodeError:
                                    pass
                    return
                
                for msg in messages:
                    if msg.data:
                        try:
                            event_data = json.loads(msg.data)
                            self.handle_sse_event(event_data)
                        except json.JSONDecodeError:
                            print(f"Invalid SSE data: {msg.data}")
                            
            except requests.exceptions.RequestException as e:
                print(f"SSE connection error: {e}")
            except Exception as e:
                print(f"SSE listener error: {e}")
            finally:
                print("SSE listener stopped")
        
        self.sse_thread = threading.Thread(target=sse_listener, daemon=True)
        self.sse_thread.start()
        print("SSE listener started in background")
    
    def handle_sse_event(self, event_data):
        """Handle incoming SSE events"""
        event_type = event_data.get('type', 'unknown')
        timestamp = datetime.fromtimestamp(event_data.get('timestamp', 0) / 1000)
        
        if event_type == 'welcome':
            print(f"[SSE {timestamp.strftime('%H:%M:%S')}] {event_data.get('message', '')}")
        elif event_type == 'key_received':
            key = event_data.get('key', '')
            key_code = event_data.get('key_code', 0)
            is_held = event_data.get('is_held', False)
            held_text = " (HELD)" if is_held else ""
            print(f"[SSE {timestamp.strftime('%H:%M:%S')}] Server processed key: '{key}' ({key_code}){held_text}")
        elif event_type == 'key_down':
            key = event_data.get('key', '')
            key_code = event_data.get('key_code', 0)
            print(f"[SSE {timestamp.strftime('%H:%M:%S')}] Key DOWN: '{key}' ({key_code})")
        elif event_type == 'key_up':
            key = event_data.get('key', '')
            key_code = event_data.get('key_code', 0)
            print(f"[SSE {timestamp.strftime('%H:%M:%S')}] Key UP: '{key}' ({key_code})")
        elif event_type == 'keys_batch':
            keys = event_data.get('keys', [])
            print(f"[SSE {timestamp.strftime('%H:%M:%S')}] Server processed batch: {len(keys)} keys")
        elif event_type == 'ping':
            # Silent keepalive
            pass
        else:
            print(f"[SSE {timestamp.strftime('%H:%M:%S')}] {event_type}: {event_data}")
    
    def stop_sse_listener(self):
        """Stop the SSE listener"""
        if self.sse_client:
            self.sse_client.close()
        print("SSE listener stopped")
    
    def disconnect(self):
        """Disconnect from the server"""
        self.stop_sse_listener()
        self.connected = False
        print("Disconnected from server")


def install_sseclient():
    """Try to install sseclient-py if not available"""
    try:
        import sseclient
        return True
    except ImportError:
        try:
            import subprocess
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'sseclient-py'])
            import sseclient
            return True
        except Exception as e:
            print(f"Warning: Could not install sseclient-py: {e}")
            print("SSE functionality will work but with basic fallback")
            return False


def main():
    """Main function with interactive menu"""
    print("=== ROS2 REST API Keyboard Client ===")
    
    # Try to install sseclient for better SSE support
    install_sseclient()
    
    # Parse command line arguments
    server_address = None
    if len(sys.argv) > 1:
        server_address = sys.argv[1]
        if not server_address.startswith('http'):
            server_address = f'http://{server_address}'
    
    print("\nServer address detection:")
    print("- If no argument provided, will auto-detect server")
    print("- Use format: python3 rest_client_example.py [server_url]")
    print("- Examples: python3 rest_client_example.py http://192.168.1.100:5000")
    print("           python3 rest_client_example.py 192.168.1.100:5000")
    print("")
    
    client = KeyboardClient(server_address)
    print(f"Attempting to connect to: {client.server_address}")
    
    # Connect to server
    if not client.connect():
        print("Failed to connect. Make sure the REST API server is running.")
        print("Try specifying the server address manually:")
        print("python3 rest_client_example.py http://your-server-ip:5000")
        return
    
    try:
        while True:
            print("\n" + "="*50)
            print("Choose an option:")
            print("1. Send single key")
            print("2. Send key batch")
            print("3. Get server status")
            print("4. Send quick test keys")
            print("5. Test key down/up events")
            print("6. Start SSE listener")
            print("7. Stop SSE listener")
            print("8. Test connection")
            print("9. Quit")
            print("="*50)
            
            try:
                choice = input("Enter choice (1-9): ").strip()
                
                if choice == '1':
                    key = input("Enter key: ").strip()
                    if key:
                        if len(key) == 1:
                            key_code = ord(key)
                        elif key.upper() == 'SPACE':
                            key_code = 32
                        elif key.upper() == 'ENTER':
                            key_code = 13
                        elif key.upper() == 'ESC':
                            key_code = 27
                        else:
                            try:
                                key_code = int(input(f"Enter key code for '{key}': "))
                            except ValueError:
                                key_code = 0
                        
                        client.send_key(key, key_code)
                
                elif choice == '2':
                    print("Enter keys (format: key:code, e.g., 'a:97,b:98' or press Enter for default test batch)")
                    keys_input = input("Keys: ").strip()
                    
                    if not keys_input:
                        # Default test batch
                        keys_data = [
                            {'key': 'h', 'key_code': 104},
                            {'key': 'e', 'key_code': 101},
                            {'key': 'l', 'key_code': 108},
                            {'key': 'l', 'key_code': 108},
                            {'key': 'o', 'key_code': 111}
                        ]
                    else:
                        keys_data = []
                        for pair in keys_input.split(','):
                            try:
                                key, code = pair.split(':')
                                keys_data.append({'key': key.strip(), 'key_code': int(code.strip())})
                            except ValueError:
                                print(f"Invalid format: {pair}")
                                continue
                    
                    if keys_data:
                        client.send_keys_batch(keys_data)
                
                elif choice == '3':
                    client.get_status()
                
                elif choice == '4':
                    print("Sending test keys...")
                    test_keys = [
                        ('a', 97),
                        ('b', 98),
                        ('SPACE', 32),
                        ('1', 49),
                        ('2', 50)
                    ]
                    
                    for key, code in test_keys:
                        client.send_key(key, code)
                        time.sleep(0.5)
                    
                    print("Test keys sent!")
                
                elif choice == '5':
                    print("Testing key down/up events...")
                    key = input("Enter key for down/up test: ").strip()
                    if key:
                        if len(key) == 1:
                            key_code = ord(key)
                        else:
                            try:
                                key_code = int(input(f"Enter key code for '{key}': "))
                            except ValueError:
                                key_code = 0
                        
                        print(f"Sending key DOWN for '{key}'...")
                        client.send_key_down(key, key_code)
                        time.sleep(2)
                        print(f"Sending key UP for '{key}'...")
                        client.send_key_up(key, key_code)
                
                elif choice == '6':
                    client.start_sse_listener()
                
                elif choice == '7':
                    client.stop_sse_listener()
                
                elif choice == '8':
                    client.connect()
                
                elif choice == '9' or choice.lower() == 'q':
                    print("Quitting...")
                    break
                
                else:
                    print("Invalid choice. Please enter 1-9.")
                    
            except KeyboardInterrupt:
                print("\nOperation interrupted")
                break
            except Exception as e:
                print(f"Error: {e}")
    
    finally:
        client.disconnect()


if __name__ == '__main__':
    main()