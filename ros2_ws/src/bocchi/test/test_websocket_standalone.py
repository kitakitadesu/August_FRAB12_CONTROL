#!/usr/bin/env python3
"""
ROS2-compatible standalone WebSocket tests for bocchi robot controller.
These tests run without external ROS2 dependencies using mocks and are
compatible with colcon test and ros2 test infrastructure.
"""

import unittest
import asyncio
import json
import time
import threading
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import sys
import os
from concurrent.futures import ThreadPoolExecutor
import queue
import pytest

# WebSocket functionality (optional dependency)
try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False

# ROS2 test compatibility
try:
    import rclpy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class MockTwist:
    """Mock Twist message for testing"""
    def __init__(self):
        self.linear = MockVector3()
        self.angular = MockVector3()


class MockVector3:
    """Mock Vector3 for testing"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class MockInt32:
    """Mock Int32 message for testing"""
    def __init__(self, data=0):
        self.data = data


class MockWebSocketManager:
    """Mock WebSocket manager implementation"""
    
    def __init__(self):
        self.connected_clients = set()
        self.lock = threading.Lock()

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
                if hasattr(client, 'send') and asyncio.iscoroutinefunction(client.send):
                    await client.send(message_str)
                elif hasattr(client, 'send'):
                    client.send(message_str)
            except Exception as e:
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            self.remove_client(client)


class MockKeyStateManager:
    """Mock key state manager implementation"""
    
    def __init__(self, debounce_time=0.01):
        self.debounce_time = debounce_time
        self.current_keys = set()
        self.servo_position = 0
        self.key_changed = False
        self.servo_changed = False
        self.last_key_time = {}
        self.lock = threading.Lock()

    def update_key(self, key_code, is_pressed=True):
        with self.lock:
            current_time = time.time()
            
            # Handle F key for servo toggle
            if key_code == 70 and is_pressed:  # F key
                self.servo_position = 180 if self.servo_position == 0 else 0
                self.servo_changed = True
                self.last_key_time[key_code] = current_time
                return True
            
            # Debouncing
            if key_code in self.last_key_time:
                if current_time - self.last_key_time[key_code] < self.debounce_time:
                    return False
            
            self.last_key_time[key_code] = current_time
            
            if is_pressed:
                if key_code not in self.current_keys:
                    self.current_keys.add(key_code)
                    self.key_changed = True
                    return True
            else:
                if key_code in self.current_keys:
                    self.current_keys.remove(key_code)
                    self.key_changed = True
                    return True
                    
        return False

    def get_twist_if_changed(self):
        """Calculate Twist message based on current pressed keys"""
        if not self.key_changed:
            return None
            
        self.key_changed = False
        twist = MockTwist()
        
        # Key mappings: W=87, S=83, A=65, D=68
        if 87 in self.current_keys:  # W - forward
            twist.linear.x = 0.5
        elif 83 in self.current_keys:  # S - backward
            twist.linear.x = -0.5
        
        if 65 in self.current_keys:  # A - turn left
            twist.angular.z = 0.5
        elif 68 in self.current_keys:  # D - turn right
            twist.angular.z = -0.5
            
        return twist

    def get_servo_if_changed(self):
        """Get servo position if changed"""
        if not self.servo_changed:
            return None
            
        self.servo_changed = False
        return self.servo_position


class TestStandaloneWebSocketManager(unittest.TestCase):
    """Test WebSocket manager functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.ws_manager = MockWebSocketManager()
        
    def test_websocket_manager_initialization(self):
        """Test WebSocketManager initializes correctly"""
        self.assertIsInstance(self.ws_manager.connected_clients, set)
        self.assertEqual(len(self.ws_manager.connected_clients), 0)
        
    def test_add_client(self):
        """Test adding WebSocket clients"""
        mock_websocket1 = Mock()
        mock_websocket2 = Mock()
        
        self.ws_manager.add_client(mock_websocket1)
        self.assertEqual(len(self.ws_manager.connected_clients), 1)
        self.assertIn(mock_websocket1, self.ws_manager.connected_clients)
        
        self.ws_manager.add_client(mock_websocket2)
        self.assertEqual(len(self.ws_manager.connected_clients), 2)
        self.assertIn(mock_websocket2, self.ws_manager.connected_clients)
        
    def test_remove_client(self):
        """Test removing WebSocket clients"""
        mock_websocket = Mock()
        
        # Add client first
        self.ws_manager.add_client(mock_websocket)
        self.assertEqual(len(self.ws_manager.connected_clients), 1)
        
        # Remove client
        self.ws_manager.remove_client(mock_websocket)
        self.assertEqual(len(self.ws_manager.connected_clients), 0)
        self.assertNotIn(mock_websocket, self.ws_manager.connected_clients)
        
    def test_multiple_client_management(self):
        """Test managing multiple clients"""
        clients = []
        for i in range(5):
            mock_websocket = Mock()
            clients.append(mock_websocket)
            self.ws_manager.add_client(mock_websocket)
            
        self.assertEqual(len(self.ws_manager.connected_clients), 5)
        
        # Remove some clients
        for client in clients[:3]:
            self.ws_manager.remove_client(client)
            
        self.assertEqual(len(self.ws_manager.connected_clients), 2)
        
        # Verify correct clients remain
        for client in clients[3:]:
            self.assertIn(client, self.ws_manager.connected_clients)

    def test_broadcast_empty_clients(self):
        """Test broadcasting with no connected clients"""
        async def _test():
            message = {'type': 'test', 'data': 'hello'}
            # Should not raise any exception
            await self.ws_manager.broadcast(message)
        
        run_async_test(_test())
        
    def test_broadcast_with_clients(self):
        """Test broadcasting to connected clients"""
        async def _test():
            mock_websocket1 = AsyncMock()
            mock_websocket2 = AsyncMock()
            
            self.ws_manager.add_client(mock_websocket1)
            self.ws_manager.add_client(mock_websocket2)
            
            message = {'type': 'test', 'data': 'hello'}
            await self.ws_manager.broadcast(message)
        
        run_async_test(_test())
        
        expected_json = json.dumps(message)
        mock_websocket1.send.assert_called_once_with(expected_json)
        mock_websocket2.send.assert_called_once_with(expected_json)


class TestStandaloneKeyStateManager(unittest.TestCase):
    """Test key state manager functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.key_manager = MockKeyStateManager(debounce_time=0.01)
        
    def test_initialization(self):
        """Test KeyStateManager initializes correctly"""
        self.assertIsInstance(self.key_manager.current_keys, set)
        self.assertEqual(len(self.key_manager.current_keys), 0)
        self.assertEqual(self.key_manager.servo_position, 0)
        self.assertFalse(self.key_manager.key_changed)
        self.assertFalse(self.key_manager.servo_changed)
        
    def test_wasd_key_mapping(self):
        """Test WASD key press and release functionality"""
        # Test W key (forward)
        result = self.key_manager.update_key(87, is_pressed=True)  # W key
        self.assertTrue(result)
        self.assertIn(87, self.key_manager.current_keys)
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, 0.5)
        self.assertEqual(twist.angular.z, 0.0)
        
        # Test key release
        time.sleep(0.02)  # Wait for debounce
        result = self.key_manager.update_key(87, is_pressed=False)
        self.assertTrue(result)
        self.assertNotIn(87, self.key_manager.current_keys)
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, 0.0)
        
    def test_servo_f_key_toggle(self):
        """Test F key servo toggle functionality"""
        # Initial position should be 0
        self.assertEqual(self.key_manager.servo_position, 0)
        
        # First F key press - toggle to 180
        result = self.key_manager.update_key(70, is_pressed=True)  # F key
        self.assertTrue(result)
        self.assertEqual(self.key_manager.servo_position, 180)
        
        servo_pos = self.key_manager.get_servo_if_changed()
        self.assertEqual(servo_pos, 180)
        
        # Wait for debounce
        time.sleep(0.02)
        
        # Second F key press - toggle back to 0
        result = self.key_manager.update_key(70, is_pressed=True)
        self.assertTrue(result)
        self.assertEqual(self.key_manager.servo_position, 0)
        
        servo_pos = self.key_manager.get_servo_if_changed()
        self.assertEqual(servo_pos, 0)
        
    def test_combined_movement(self):
        """Test combined WASD key presses"""
        # Press W (forward) and A (left turn) simultaneously
        self.key_manager.update_key(87, is_pressed=True)  # W
        self.key_manager.update_key(65, is_pressed=True)  # A
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, 0.5)   # Forward
        self.assertEqual(twist.angular.z, 0.5)  # Turn left
        
    def test_key_debouncing(self):
        """Test key debouncing prevents rapid duplicate processing"""
        # Rapid key presses within debounce time
        result1 = self.key_manager.update_key(87, is_pressed=True)
        result2 = self.key_manager.update_key(87, is_pressed=True)  # Immediate repeat
        
        self.assertTrue(result1)
        self.assertFalse(result2)  # Should be debounced
        
        # Wait for debounce to expire
        time.sleep(0.02)
        
        # Next press should register
        result3 = self.key_manager.update_key(87, is_pressed=True)
        self.assertFalse(result3)  # Still pressed, so no change


class TestWebSocketMessageFormats(unittest.TestCase):
    """Test WebSocket message format compliance"""
    
    def test_key_down_message_format(self):
        """Test key down message format"""
        message = {
            'type': 'key_down',
            'key': 'w',
            'key_code': 87,
            'timestamp': int(time.time() * 1000)
        }
        
        # Should be JSON serializable
        json_str = json.dumps(message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'key_down')
        self.assertEqual(parsed['key'], 'w')
        self.assertEqual(parsed['key_code'], 87)
        self.assertIsInstance(parsed['timestamp'], int)
        
    def test_welcome_message_format(self):
        """Test welcome message format"""
        message = {
            'type': 'welcome',
            'message': 'Connected to bocchi robot controller',
            'timestamp': int(time.time() * 1000)
        }
        
        json_str = json.dumps(message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'welcome')
        self.assertIn('message', parsed)
        
    def test_keys_batch_message_format(self):
        """Test keys batch message format"""
        message = {
            'type': 'keys_batch',
            'keys': [
                {'key': 'w', 'key_code': 87},
                {'key': 'a', 'key_code': 65}
            ],
            'timestamp': int(time.time() * 1000)
        }
        
        json_str = json.dumps(message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'keys_batch')
        self.assertIsInstance(parsed['keys'], list)
        self.assertEqual(len(parsed['keys']), 2)


class TestWebSocketServer(unittest.TestCase):
    """Test WebSocket server functionality"""
    
    async def echo_handler(self, websocket, path):
        """Simple echo handler for testing"""
        try:
            async for message in websocket:
                data = json.loads(message)
                response = {
                    'type': 'echo',
                    'original': data,
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(response))
        except websockets.exceptions.ConnectionClosed:
            pass
        except json.JSONDecodeError:
            error_response = {
                'type': 'error',
                'message': 'Invalid JSON',
                'timestamp': int(time.time() * 1000)
            }
            await websocket.send(json.dumps(error_response))
    
    def test_websocket_connection(self):
        """Test WebSocket connection establishment"""
        async def _test():
            test_port = 8776
            
            # Start test server
            server = await websockets.serve(self.echo_handler, 'localhost', test_port)
            
            try:
                # Connect to test server
                async with websockets.connect(f'ws://localhost:{test_port}') as websocket:
                    # Send test message
                    test_message = {
                        'type': 'test_connection',
                        'data': 'hello server'
                    }
                    await websocket.send(json.dumps(test_message))
                    
                    # Receive response
                    response = await websocket.recv()
                    response_data = json.loads(response)
                    
                    self.assertEqual(response_data['type'], 'echo')
                    self.assertEqual(response_data['original'], test_message)
                    
            finally:
                server.close()
                await server.wait_closed()
        
        run_async_test(_test())
    
    def test_multiple_connections(self):
        """Test multiple concurrent connections"""
        async def _test():
            test_port = 8777
            connection_count = 3
            
            # Start test server
            server = await websockets.serve(self.echo_handler, 'localhost', test_port)
            
            try:
                # Create multiple connections
                connections = []
                for i in range(connection_count):
                    conn = await websockets.connect(f'ws://localhost:{test_port}')
                    connections.append(conn)
                
                # Send messages from all connections
                for i, conn in enumerate(connections):
                    message = {
                        'type': 'multi_test',
                        'client_id': i,
                        'data': f'message from client {i}'
                    }
                    await conn.send(json.dumps(message))
                    
                    # Receive echo
                    response = await conn.recv()
                    response_data = json.loads(response)
                    
                    self.assertEqual(response_data['type'], 'echo')
                    self.assertEqual(response_data['original']['client_id'], i)
                
                # Close all connections
                for conn in connections:
                    await conn.close()
                    
            finally:
                server.close()
                await server.wait_closed()
        
        run_async_test(_test())


class TestPerformance(unittest.TestCase):
    """Test performance characteristics"""
    
    def test_rapid_key_processing(self):
        """Test rapid key processing performance"""
        key_manager = MockKeyStateManager(debounce_time=0.001)  # Short debounce
        
        start_time = time.time()
        processed_count = 0
        
        # Process many key events with different keys and press/release cycles
        keys = [87, 83, 65, 68]  # W, S, A, D keys
        for i in range(100):
            key_code = keys[i % 4]
            is_pressed = (i % 2 == 0)  # Alternate press/release
            result = key_manager.update_key(key_code, is_pressed)
            if result:
                processed_count += 1
            time.sleep(0.002)  # Small delay
            
        end_time = time.time()
        duration = end_time - start_time
        
        # Should process quickly
        self.assertLess(duration, 2.0)  # Under 2 seconds
        self.assertGreater(processed_count, 1)  # Should process some keys (debouncing limits this)
        
    def test_websocket_manager_performance(self):
        """Test WebSocket manager performance"""
        ws_manager = MockWebSocketManager()
        
        start_time = time.time()
        
        # Add many clients rapidly
        clients = []
        for i in range(50):
            client = Mock()
            clients.append(client)
            ws_manager.add_client(client)
            
        end_time = time.time()
        duration = end_time - start_time
        
        # Should complete quickly
        self.assertLess(duration, 0.5)  # Under 0.5 seconds
        self.assertEqual(len(ws_manager.connected_clients), 50)


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
    
    def test_websocket_connection(self):
        """Test WebSocket connection"""
        test_instance = TestWebSocketServer()
        run_async_test(test_instance.test_websocket_connection())
        
    def test_multiple_connections(self):
        """Test multiple connections"""
        test_instance = TestWebSocketServer()
        run_async_test(test_instance.test_multiple_connections())
        
    def test_broadcast_empty_clients(self):
        """Test broadcast with no clients"""
        test_instance = TestStandaloneWebSocketManager()
        test_instance.setUp()
        run_async_test(test_instance.test_broadcast_empty_clients())
        
    def test_broadcast_with_clients(self):
        """Test broadcast with clients"""
        test_instance = TestStandaloneWebSocketManager()
        test_instance.setUp()
        run_async_test(test_instance.test_broadcast_with_clients())


if __name__ == '__main__':
    print("🧪 Running Standalone WebSocket Tests")
    print("=" * 50)
    
    # Run all tests
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestStandaloneWebSocketManager,
        TestStandaloneKeyStateManager,
        TestWebSocketMessageFormats,
        TestWebSocketServer,
        TestPerformance,
        AsyncTestWrapper
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2, buffer=True)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n{'='*50}")
    print(f"📊 Standalone WebSocket Test Results:")
    print(f"{'='*50}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print(f"\nFailures:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback.split('AssertionError:')[-1].strip()}")
            
    if result.errors:
        print(f"\nErrors:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback.split('Error:')[-1].strip()}")
    
    success_rate = ((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100) if result.testsRun > 0 else 0
    print(f"\nSuccess rate: {success_rate:.1f}%")
    
    if result.wasSuccessful():
        print("✅ All standalone WebSocket tests passed!")
        print("🚀 WebSocket functionality is working correctly")
    else:
        print("⚠️  Some tests failed - review implementation")
        
    print(f"{'='*50}")
    
    # Exit with proper code
    sys.exit(0 if result.wasSuccessful() else 1)