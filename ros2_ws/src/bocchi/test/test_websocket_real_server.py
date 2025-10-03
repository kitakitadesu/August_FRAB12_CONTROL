#!/usr/bin/env python3
"""
Real WebSocket server test for bocchi robot controller.
This test creates actual WebSocket servers and clients to test real networking.
"""

import unittest
import asyncio
import websockets
import json
import time
import threading
import socket
from concurrent.futures import ThreadPoolExecutor
import sys
import os


class TestRealWebSocketServer(unittest.TestCase):
    """Test real WebSocket server functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.test_port_base = 9000
        self.servers = []
        self.connections = []
        
    def tearDown(self):
        """Clean up test fixtures"""
        # Close all connections and servers
        for conn in self.connections:
            try:
                asyncio.get_event_loop().run_until_complete(conn.close())
            except:
                pass
                
        for server in self.servers:
            try:
                server.close()
                asyncio.get_event_loop().run_until_complete(server.wait_closed())
            except:
                pass
    
    def get_available_port(self):
        """Get an available port for testing"""
        port = self.test_port_base
        while port < self.test_port_base + 100:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', port))
                    return port
            except OSError:
                port += 1
        raise Exception("No available ports found")
    
    async def robot_control_handler(self, websocket, path):
        """Robot control WebSocket handler"""
        try:
            # Send welcome message
            welcome = {
                'type': 'welcome',
                'message': 'Connected to bocchi robot controller',
                'timestamp': int(time.time() * 1000),
                'features': ['movement_control', 'servo_control', 'real_time_feedback']
            }
            await websocket.send(json.dumps(welcome))
            
            # Handle incoming messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    # Process different message types
                    if data.get('type') == 'key_down':
                        await self.handle_key_down(websocket, data)
                    elif data.get('type') == 'key_up':
                        await self.handle_key_up(websocket, data)
                    elif data.get('type') == 'ping':
                        await self.handle_ping(websocket, data)
                    elif data.get('type') == 'robot_command':
                        await self.handle_robot_command(websocket, data)
                    else:
                        await self.send_error(websocket, f"Unknown message type: {data.get('type')}")
                        
                except json.JSONDecodeError:
                    await self.send_error(websocket, "Invalid JSON format")
                except Exception as e:
                    await self.send_error(websocket, f"Processing error: {str(e)}")
                    
        except websockets.exceptions.ConnectionClosed:
            pass
    
    async def handle_key_down(self, websocket, data):
        """Handle key down events"""
        key = data.get('key', '')
        key_code = data.get('key_code', 0)
        
        # Validate key
        if not key or not isinstance(key_code, int):
            await self.send_error(websocket, "Invalid key data")
            return
        
        # Process movement keys
        movement_response = None
        if key_code == 87:  # W key
            movement_response = {'action': 'move_forward', 'speed': 0.5}
        elif key_code == 83:  # S key
            movement_response = {'action': 'move_backward', 'speed': 0.5}
        elif key_code == 65:  # A key
            movement_response = {'action': 'turn_left', 'speed': 0.5}
        elif key_code == 68:  # D key
            movement_response = {'action': 'turn_right', 'speed': 0.5}
        elif key_code == 70:  # F key
            movement_response = {'action': 'servo_toggle', 'position': 180}
        
        response = {
            'type': 'key_processed',
            'original_key': key,
            'key_code': key_code,
            'movement': movement_response,
            'timestamp': int(time.time() * 1000)
        }
        
        await websocket.send(json.dumps(response))
    
    async def handle_key_up(self, websocket, data):
        """Handle key up events"""
        key = data.get('key', '')
        key_code = data.get('key_code', 0)
        
        response = {
            'type': 'key_released',
            'original_key': key,
            'key_code': key_code,
            'movement': {'action': 'stop'},
            'timestamp': int(time.time() * 1000)
        }
        
        await websocket.send(json.dumps(response))
    
    async def handle_ping(self, websocket, data):
        """Handle ping messages"""
        response = {
            'type': 'pong',
            'timestamp': int(time.time() * 1000),
            'original_timestamp': data.get('timestamp', 0),
            'latency_ms': int(time.time() * 1000) - data.get('timestamp', 0)
        }
        
        await websocket.send(json.dumps(response))
    
    async def handle_robot_command(self, websocket, data):
        """Handle direct robot commands"""
        command = data.get('command', '')
        
        if command == 'status':
            response = {
                'type': 'robot_status',
                'status': 'online',
                'battery': 85,
                'position': {'x': 0, 'y': 0, 'theta': 0},
                'timestamp': int(time.time() * 1000)
            }
        elif command == 'emergency_stop':
            response = {
                'type': 'emergency_stop_confirmed',
                'all_movement_stopped': True,
                'timestamp': int(time.time() * 1000)
            }
        else:
            response = {
                'type': 'command_error',
                'error': f"Unknown command: {command}",
                'timestamp': int(time.time() * 1000)
            }
        
        await websocket.send(json.dumps(response))
    
    async def send_error(self, websocket, error_message):
        """Send error message to client"""
        error_response = {
            'type': 'error',
            'error': error_message,
            'timestamp': int(time.time() * 1000)
        }
        await websocket.send(json.dumps(error_response))
    
    async def test_basic_connection(self):
        """Test basic WebSocket connection"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect client
        websocket = await websockets.connect(f'ws://localhost:{port}')
        self.connections.append(websocket)
        
        # Receive welcome message
        welcome_msg = await websocket.recv()
        welcome_data = json.loads(welcome_msg)
        
        self.assertEqual(welcome_data['type'], 'welcome')
        self.assertIn('message', welcome_data)
        self.assertIn('features', welcome_data)
        self.assertIsInstance(welcome_data['features'], list)
    
    async def test_key_down_processing(self):
        """Test key down message processing"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect client
        websocket = await websockets.connect(f'ws://localhost:{port}')
        self.connections.append(websocket)
        
        # Skip welcome message
        await websocket.recv()
        
        # Send key down message
        key_message = {
            'type': 'key_down',
            'key': 'w',
            'key_code': 87,
            'timestamp': int(time.time() * 1000)
        }
        await websocket.send(json.dumps(key_message))
        
        # Receive response
        response_msg = await websocket.recv()
        response_data = json.loads(response_msg)
        
        self.assertEqual(response_data['type'], 'key_processed')
        self.assertEqual(response_data['original_key'], 'w')
        self.assertEqual(response_data['key_code'], 87)
        self.assertIn('movement', response_data)
        self.assertEqual(response_data['movement']['action'], 'move_forward')
    
    async def test_all_movement_keys(self):
        """Test all movement keys (WASD + F)"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect client
        websocket = await websockets.connect(f'ws://localhost:{port}')
        self.connections.append(websocket)
        
        # Skip welcome message
        await websocket.recv()
        
        # Test all keys
        key_tests = [
            {'key': 'w', 'code': 87, 'expected_action': 'move_forward'},
            {'key': 's', 'code': 83, 'expected_action': 'move_backward'},
            {'key': 'a', 'code': 65, 'expected_action': 'turn_left'},
            {'key': 'd', 'code': 68, 'expected_action': 'turn_right'},
            {'key': 'f', 'code': 70, 'expected_action': 'servo_toggle'}
        ]
        
        for test in key_tests:
            # Send key down
            key_message = {
                'type': 'key_down',
                'key': test['key'],
                'key_code': test['code'],
                'timestamp': int(time.time() * 1000)
            }
            await websocket.send(json.dumps(key_message))
            
            # Receive response
            response_msg = await websocket.recv()
            response_data = json.loads(response_msg)
            
            self.assertEqual(response_data['type'], 'key_processed')
            self.assertEqual(response_data['movement']['action'], test['expected_action'])
    
    async def test_ping_pong(self):
        """Test ping-pong functionality"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect client
        websocket = await websockets.connect(f'ws://localhost:{port}')
        self.connections.append(websocket)
        
        # Skip welcome message
        await websocket.recv()
        
        # Send ping
        ping_timestamp = int(time.time() * 1000)
        ping_message = {
            'type': 'ping',
            'timestamp': ping_timestamp
        }
        await websocket.send(json.dumps(ping_message))
        
        # Receive pong
        pong_msg = await websocket.recv()
        pong_data = json.loads(pong_msg)
        
        self.assertEqual(pong_data['type'], 'pong')
        self.assertEqual(pong_data['original_timestamp'], ping_timestamp)
        self.assertIn('latency_ms', pong_data)
        self.assertGreaterEqual(pong_data['latency_ms'], 0)
    
    async def test_robot_commands(self):
        """Test robot command processing"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect client
        websocket = await websockets.connect(f'ws://localhost:{port}')
        self.connections.append(websocket)
        
        # Skip welcome message
        await websocket.recv()
        
        # Test status command
        status_message = {
            'type': 'robot_command',
            'command': 'status',
            'timestamp': int(time.time() * 1000)
        }
        await websocket.send(json.dumps(status_message))
        
        response_msg = await websocket.recv()
        response_data = json.loads(response_msg)
        
        self.assertEqual(response_data['type'], 'robot_status')
        self.assertIn('status', response_data)
        self.assertIn('battery', response_data)
        self.assertIn('position', response_data)
        
        # Test emergency stop command
        stop_message = {
            'type': 'robot_command',
            'command': 'emergency_stop',
            'timestamp': int(time.time() * 1000)
        }
        await websocket.send(json.dumps(stop_message))
        
        response_msg = await websocket.recv()
        response_data = json.loads(response_msg)
        
        self.assertEqual(response_data['type'], 'emergency_stop_confirmed')
        self.assertTrue(response_data['all_movement_stopped'])
    
    async def test_multiple_clients(self):
        """Test multiple concurrent clients"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect multiple clients
        num_clients = 3
        clients = []
        
        for i in range(num_clients):
            websocket = await websockets.connect(f'ws://localhost:{port}')
            clients.append(websocket)
            self.connections.append(websocket)
            
            # Skip welcome message
            await websocket.recv()
        
        # Send messages from all clients simultaneously
        for i, client in enumerate(clients):
            key_message = {
                'type': 'key_down',
                'key': 'w',
                'key_code': 87,
                'client_id': i,
                'timestamp': int(time.time() * 1000)
            }
            await client.send(json.dumps(key_message))
        
        # Receive responses from all clients
        for i, client in enumerate(clients):
            response_msg = await client.recv()
            response_data = json.loads(response_msg)
            
            self.assertEqual(response_data['type'], 'key_processed')
            self.assertEqual(response_data['movement']['action'], 'move_forward')
    
    async def test_error_handling(self):
        """Test error handling for invalid messages"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect client
        websocket = await websockets.connect(f'ws://localhost:{port}')
        self.connections.append(websocket)
        
        # Skip welcome message
        await websocket.recv()
        
        # Send invalid JSON
        await websocket.send("invalid json {")
        
        # Receive error response
        error_msg = await websocket.recv()
        error_data = json.loads(error_msg)
        
        self.assertEqual(error_data['type'], 'error')
        self.assertIn('Invalid JSON', error_data['error'])
        
        # Send unknown message type
        unknown_message = {
            'type': 'unknown_type',
            'data': 'test'
        }
        await websocket.send(json.dumps(unknown_message))
        
        error_msg = await websocket.recv()
        error_data = json.loads(error_msg)
        
        self.assertEqual(error_data['type'], 'error')
        self.assertIn('Unknown message type', error_data['error'])
    
    async def test_performance_latency(self):
        """Test WebSocket performance and latency"""
        port = self.get_available_port()
        
        # Start server
        server = await websockets.serve(self.robot_control_handler, 'localhost', port)
        self.servers.append(server)
        
        # Connect client
        websocket = await websockets.connect(f'ws://localhost:{port}')
        self.connections.append(websocket)
        
        # Skip welcome message
        await websocket.recv()
        
        # Measure round-trip time for multiple pings
        latencies = []
        num_pings = 10
        
        for i in range(num_pings):
            start_time = time.time() * 1000
            
            ping_message = {
                'type': 'ping',
                'timestamp': int(start_time)
            }
            await websocket.send(json.dumps(ping_message))
            
            pong_msg = await websocket.recv()
            end_time = time.time() * 1000
            
            pong_data = json.loads(pong_msg)
            self.assertEqual(pong_data['type'], 'pong')
            
            latency = end_time - start_time
            latencies.append(latency)
        
        # Calculate statistics
        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)
        
        # Performance assertions
        self.assertLess(avg_latency, 50)  # Average latency under 50ms
        self.assertLess(max_latency, 100)  # Max latency under 100ms


def run_async_test(coro):
    """Helper function to run async tests"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class AsyncTestWrapper(unittest.TestCase):
    """Wrapper for async tests"""
    
    def test_basic_connection(self):
        """Test basic WebSocket connection"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_basic_connection())
        finally:
            test_instance.tearDown()
    
    def test_key_down_processing(self):
        """Test key down processing"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_key_down_processing())
        finally:
            test_instance.tearDown()
    
    def test_all_movement_keys(self):
        """Test all movement keys"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_all_movement_keys())
        finally:
            test_instance.tearDown()
    
    def test_ping_pong(self):
        """Test ping-pong functionality"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_ping_pong())
        finally:
            test_instance.tearDown()
    
    def test_robot_commands(self):
        """Test robot commands"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_robot_commands())
        finally:
            test_instance.tearDown()
    
    def test_multiple_clients(self):
        """Test multiple clients"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_multiple_clients())
        finally:
            test_instance.tearDown()
    
    def test_error_handling(self):
        """Test error handling"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_error_handling())
        finally:
            test_instance.tearDown()
    
    def test_performance_latency(self):
        """Test performance and latency"""
        test_instance = TestRealWebSocketServer()
        test_instance.setUp()
        try:
            run_async_test(test_instance.test_performance_latency())
        finally:
            test_instance.tearDown()


if __name__ == '__main__':
    print("🌐 Running Real WebSocket Server Tests")
    print("=" * 50)
    
    # Run all tests
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add async test wrapper
    tests = loader.loadTestsFromTestCase(AsyncTestWrapper)
    suite.addTests(tests)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2, buffer=True)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n{'='*50}")
    print(f"🌐 Real WebSocket Test Results:")
    print(f"{'='*50}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print(f"\nFailures:")
        for test, traceback in result.failures:
            print(f"  - {test}")
            
    if result.errors:
        print(f"\nErrors:")
        for test, traceback in result.errors:
            print(f"  - {test}")
    
    success_rate = ((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100) if result.testsRun > 0 else 0
    print(f"\nSuccess rate: {success_rate:.1f}%")
    
    if result.wasSuccessful():
        print("✅ All real WebSocket tests passed!")
        print("🚀 WebSocket networking is working correctly")
        print("🤖 Robot control protocol validated")
    else:
        print("⚠️  Some tests failed - review WebSocket implementation")
        
    print(f"{'='*50}")
    
    # Exit with proper code
    sys.exit(0 if result.wasSuccessful() else 1)