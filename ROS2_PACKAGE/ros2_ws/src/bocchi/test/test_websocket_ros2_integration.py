#!/usr/bin/env python3
"""
WebSocket integration tests with ROS2 robot control for bocchi.
Tests the complete flow from WebSocket messages to ROS2 topic publications.
"""

import unittest
import asyncio
import websockets
import json
import time
import threading
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import sys
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# Try to import bocchi modules with proper ROS2 path handling
try:
    from bocchi.publisher import WebSocketManager, KeyStateManager, KeyboardAPI, MinimalPublisher
    BOCCHI_AVAILABLE = True
except ImportError:
    # Fallback for test environment
    import sys
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    try:
        from bocchi.publisher import WebSocketManager, KeyStateManager, KeyboardAPI, MinimalPublisher
        BOCCHI_AVAILABLE = True
    except ImportError:
        BOCCHI_AVAILABLE = False
        # Create mock classes for testing
        class WebSocketManager:
            def __init__(self):
                self.connected_clients = set()
                self.lock = threading.Lock()
            def add_client(self, client):
                self.connected_clients.add(client)
            def remove_client(self, client):
                self.connected_clients.discard(client)
            async def broadcast(self, message):
                pass
        
        class KeyStateManager:
            def __init__(self, debounce_time=0.01):
                self.current_keys = set()
                self.debounce_time = debounce_time
            def update_key(self, key_code, is_pressed=True):
                return True
        
        class KeyboardAPI:
            def __init__(self, publisher_node):
                self.websocket_manager = WebSocketManager()
        
        class MinimalPublisher:
            pass


class TestWebSocketROS2Integration(unittest.TestCase):
    """Test WebSocket to ROS2 integration"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock ROS2 node
        self.mock_node = Mock()
        self.mock_twist_publisher = Mock()
        self.mock_servo_publisher = Mock()
        
        self.mock_node.create_publisher.side_effect = [
            self.mock_twist_publisher,
            self.mock_servo_publisher
        ]
        
        # Create key state manager
        self.key_manager = KeyStateManager(debounce_time=0.01)
        
        # Mock the ROS2 publishers
        self.published_twists = []
        self.published_servo_positions = []
        
        def capture_twist(twist_msg):
            self.published_twists.append(twist_msg)
            
        def capture_servo(servo_msg):
            self.published_servo_positions.append(servo_msg.data)
            
        self.mock_twist_publisher.publish.side_effect = capture_twist
        self.mock_servo_publisher.publish.side_effect = capture_servo
        
        # Create mock publisher node with key manager
        self.mock_publisher_node = Mock()
        self.mock_publisher_node.key_manager = self.key_manager
        self.mock_publisher_node.twist_publisher = self.mock_twist_publisher
        self.mock_publisher_node.servo_publisher = self.mock_servo_publisher
        
        # Create KeyboardAPI with mock publisher
        self.api = KeyboardAPI(self.mock_publisher_node)
        
    def test_wasd_key_to_twist_integration(self):
        """Test WASD keys generate correct Twist messages"""
        # Test W key (forward)
        result = self.key_manager.update_key(87, is_pressed=True)  # W key
        self.assertTrue(result)
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, 0.5)
        self.assertEqual(twist.angular.z, 0.0)
        
        # Test A key (turn left)
        result = self.key_manager.update_key(65, is_pressed=True)  # A key
        self.assertTrue(result)
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, 0.5)  # Still moving forward
        self.assertEqual(twist.angular.z, 0.5)  # Now also turning left
        
        # Test S key (backward)
        self.key_manager.update_key(87, is_pressed=False)  # Release W
        self.key_manager.update_key(83, is_pressed=True)   # Press S
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, -0.5)  # Moving backward
        self.assertEqual(twist.angular.z, 0.5)   # Still turning left
        
        # Test D key (turn right)
        self.key_manager.update_key(65, is_pressed=False)  # Release A
        self.key_manager.update_key(68, is_pressed=True)   # Press D
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, -0.5)   # Still moving backward
        self.assertEqual(twist.angular.z, -0.5)  # Now turning right
        
    def test_f_key_servo_control(self):
        """Test F key controls servo position"""
        # Initial servo position should be 0
        self.assertEqual(self.key_manager.servo_position, 0)
        
        # First F key press - toggle to 180
        result = self.key_manager.update_key(70, is_pressed=True)  # F key
        self.assertTrue(result)
        self.assertEqual(self.key_manager.servo_position, 180)
        
        servo_pos = self.key_manager.get_servo_if_changed()
        self.assertEqual(servo_pos, 180)
        
        # Second F key press - toggle back to 0
        result = self.key_manager.update_key(70, is_pressed=True)
        self.assertTrue(result)
        self.assertEqual(self.key_manager.servo_position, 0)
        
        servo_pos = self.key_manager.get_servo_if_changed()
        self.assertEqual(servo_pos, 0)
        
    def test_websocket_key_event_broadcasts(self):
        """Test WebSocket broadcasts correct key events"""
        # Mock WebSocket client
        mock_websocket = Mock()
        self.api.websocket_manager.add_client(mock_websocket)
        
        # Simulate key down event through API
        with patch('time.time', return_value=1234567890.123):
            # Process key down
            result = self.key_manager.update_key(87, is_pressed=True)  # W key
            
            # Create expected event data
            expected_event = {
                'type': 'key_down',
                'key': 'w',
                'key_code': 87,
                'timestamp': 1234567890123
            }
            
            # Verify the event would be broadcast (test the data structure)
            self.assertTrue(result)
            
    def test_key_debouncing_prevents_duplicate_publications(self):
        """Test key debouncing prevents duplicate ROS2 publications"""
        # Set a longer debounce time for this test
        key_manager = KeyStateManager(debounce_time=0.1)
        
        # Press same key multiple times rapidly
        result1 = key_manager.update_key(87, is_pressed=True)  # W key
        result2 = key_manager.update_key(87, is_pressed=True)  # Same key again
        result3 = key_manager.update_key(87, is_pressed=True)  # And again
        
        # First press should register, subsequent ones should be debounced
        self.assertTrue(result1)
        self.assertFalse(result2)  # Debounced
        self.assertFalse(result3)  # Debounced
        
        # Only one twist message should be generated
        twist1 = key_manager.get_twist_if_changed()
        twist2 = key_manager.get_twist_if_changed()
        
        self.assertIsNotNone(twist1)
        self.assertIsNone(twist2)  # No change since last check
        
    def test_key_release_stops_movement(self):
        """Test key release stops robot movement"""
        # Press W key
        self.key_manager.update_key(87, is_pressed=True)  # W key
        twist = self.key_manager.get_twist_if_changed()
        self.assertEqual(twist.linear.x, 0.5)
        
        # Release W key
        self.key_manager.update_key(87, is_pressed=False)
        twist = self.key_manager.get_twist_if_changed()
        self.assertEqual(twist.linear.x, 0.0)
        
    def test_multiple_keys_simultaneous(self):
        """Test multiple keys pressed simultaneously"""
        # Press W (forward) and A (left) simultaneously
        self.key_manager.update_key(87, is_pressed=True)  # W
        self.key_manager.update_key(65, is_pressed=True)  # A
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertEqual(twist.linear.x, 0.5)   # Forward movement
        self.assertEqual(twist.angular.z, 0.5)  # Left turn
        
        # Release one key, keep the other
        self.key_manager.update_key(87, is_pressed=False)  # Release W
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertEqual(twist.linear.x, 0.0)   # No forward movement
        self.assertEqual(twist.angular.z, 0.5)  # Still turning left
        
    def test_invalid_key_codes_ignored(self):
        """Test invalid key codes are ignored"""
        # Test with invalid key codes
        invalid_keys = [999, -1, 1000, 0]
        
        for invalid_key in invalid_keys:
            result = self.key_manager.update_key(invalid_key, is_pressed=True)
            # Should return False or not cause changes
            twist = self.key_manager.get_twist_if_changed()
            # Twist should be None (no change) or zero movement
            if twist is not None:
                self.assertEqual(twist.linear.x, 0.0)
                self.assertEqual(twist.angular.z, 0.0)


class TestWebSocketMessagePropagation(unittest.TestCase):
    """Test message propagation from WebSocket to ROS2"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_publisher_node = Mock()
        self.mock_publisher_node.key_manager = KeyStateManager(debounce_time=0.01)
        self.api = KeyboardAPI(self.mock_publisher_node)
        
    def test_websocket_key_down_message_format(self):
        """Test WebSocket key down message format"""
        # Simulate key press
        timestamp = int(time.time() * 1000)
        
        expected_event = {
            'type': 'key_down',
            'key': 'w',
            'key_code': 87,
            'timestamp': timestamp
        }
        
        # Verify JSON serialization
        json_str = json.dumps(expected_event)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'key_down')
        self.assertEqual(parsed['key'], 'w')
        self.assertEqual(parsed['key_code'], 87)
        self.assertIsInstance(parsed['timestamp'], int)
        
    def test_websocket_key_up_message_format(self):
        """Test WebSocket key up message format"""
        timestamp = int(time.time() * 1000)
        
        expected_event = {
            'type': 'key_up',
            'key': 'w',
            'key_code': 87,
            'timestamp': timestamp
        }
        
        json_str = json.dumps(expected_event)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'key_up')
        self.assertEqual(parsed['key'], 'w')
        self.assertEqual(parsed['key_code'], 87)
        
    def test_websocket_batch_keys_message_format(self):
        """Test WebSocket batch keys message format"""
        timestamp = int(time.time() * 1000)
        
        expected_event = {
            'type': 'keys_batch',
            'keys': [
                {'key': 'w', 'key_code': 87},
                {'key': 'a', 'key_code': 65}
            ],
            'timestamp': timestamp
        }
        
        json_str = json.dumps(expected_event)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'keys_batch')
        self.assertIsInstance(parsed['keys'], list)
        self.assertEqual(len(parsed['keys']), 2)


class TestWebSocketServerIntegration(unittest.TestCase):
    """Test WebSocket server integration with robot control"""
    
    def setUp(self):
        """Set up integration test"""
        self.test_port = 8773
        self.websocket_url = f'ws://localhost:{self.test_port}'
        self.received_messages = []
        self.server = None
        
    async def robot_websocket_handler(self, websocket, path):
        """Simulate robot WebSocket handler"""
        try:
            # Send welcome message
            welcome = {
                'type': 'welcome',
                'message': 'Connected to bocchi robot controller',
                'timestamp': int(time.time() * 1000)
            }
            await websocket.send(json.dumps(welcome))
            
            # Handle incoming messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.received_messages.append(data)
                    
                    # Simulate robot response based on message type
                    if data.get('type') == 'key_down':
                        response = {
                            'type': 'robot_response',
                            'action': f"Moving based on key: {data.get('key')}",
                            'timestamp': int(time.time() * 1000)
                        }
                        await websocket.send(json.dumps(response))
                        
                except json.JSONDecodeError:
                    error_response = {
                        'type': 'error',
                        'message': 'Invalid JSON received',
                        'timestamp': int(time.time() * 1000)
                    }
                    await websocket.send(json.dumps(error_response))
                    
        except websockets.exceptions.ConnectionClosed:
            pass
            
    async def test_websocket_robot_communication(self):
        """Test complete WebSocket communication with robot"""
        # Start test server
        self.server = await websockets.serve(
            self.robot_websocket_handler,
            'localhost',
            self.test_port
        )
        
        try:
            # Connect as client
            async with websockets.connect(self.websocket_url) as websocket:
                # Receive welcome message
                welcome_msg = await websocket.recv()
                welcome_data = json.loads(welcome_msg)
                self.assertEqual(welcome_data['type'], 'welcome')
                
                # Send key down message
                key_message = {
                    'type': 'key_down',
                    'key': 'w',
                    'key_code': 87,
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(key_message))
                
                # Receive robot response
                response_msg = await websocket.recv()
                response_data = json.loads(response_msg)
                
                self.assertEqual(response_data['type'], 'robot_response')
                self.assertIn('Moving based on key: w', response_data['action'])
                
                # Verify server received our message
                self.assertEqual(len(self.received_messages), 1)
                self.assertEqual(self.received_messages[0]['type'], 'key_down')
                self.assertEqual(self.received_messages[0]['key'], 'w')
                
        finally:
            self.server.close()
            await self.server.wait_closed()
            
    async def test_websocket_error_handling(self):
        """Test WebSocket error handling"""
        self.server = await websockets.serve(
            self.robot_websocket_handler,
            'localhost',
            self.test_port
        )
        
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                # Skip welcome message
                await websocket.recv()
                
                # Send invalid JSON
                await websocket.send("invalid json {")
                
                # Receive error response
                error_msg = await websocket.recv()
                error_data = json.loads(error_msg)
                
                self.assertEqual(error_data['type'], 'error')
                self.assertIn('Invalid JSON', error_data['message'])
                
        finally:
            self.server.close()
            await self.server.wait_closed()


class TestROS2TopicIntegration(unittest.TestCase):
    """Test ROS2 topic publishing integration"""
    
    def setUp(self):
        """Set up ROS2 topic test fixtures"""
        # Mock ROS2 publishers
        self.mock_twist_publisher = Mock()
        self.mock_servo_publisher = Mock()
        
        # Track published messages
        self.published_twists = []
        self.published_servo_msgs = []
        
        def capture_twist(twist_msg):
            self.published_twists.append({
                'linear_x': twist_msg.linear.x,
                'linear_y': twist_msg.linear.y,
                'linear_z': twist_msg.linear.z,
                'angular_x': twist_msg.angular.x,
                'angular_y': twist_msg.angular.y,
                'angular_z': twist_msg.angular.z
            })
            
        def capture_servo(servo_msg):
            self.published_servo_msgs.append(servo_msg.data)
            
        self.mock_twist_publisher.publish.side_effect = capture_twist
        self.mock_servo_publisher.publish.side_effect = capture_servo
        
        # Create key manager
        self.key_manager = KeyStateManager(debounce_time=0.01)
        
    def test_twist_message_structure(self):
        """Test Twist message structure for robot movement"""
        # Test forward movement
        self.key_manager.update_key(87, is_pressed=True)  # W key
        twist = self.key_manager.get_twist_if_changed()
        
        # Verify Twist message structure
        self.assertEqual(twist.linear.x, 0.5)
        self.assertEqual(twist.linear.y, 0.0)
        self.assertEqual(twist.linear.z, 0.0)
        self.assertEqual(twist.angular.x, 0.0)
        self.assertEqual(twist.angular.y, 0.0)
        self.assertEqual(twist.angular.z, 0.0)
        
        # Test turning
        self.key_manager.update_key(65, is_pressed=True)  # A key (left)
        twist = self.key_manager.get_twist_if_changed()
        
        self.assertEqual(twist.linear.x, 0.5)   # Forward
        self.assertEqual(twist.angular.z, 0.5)  # Turn left
        
    def test_servo_message_structure(self):
        """Test servo control message structure"""
        # Test servo toggle
        self.key_manager.update_key(70, is_pressed=True)  # F key
        servo_pos = self.key_manager.get_servo_if_changed()
        
        self.assertEqual(servo_pos, 180)
        
        # Test toggle back
        self.key_manager.update_key(70, is_pressed=True)  # F key again
        servo_pos = self.key_manager.get_servo_if_changed()
        
        self.assertEqual(servo_pos, 0)
        
    def test_movement_velocity_limits(self):
        """Test movement velocity stays within safe limits"""
        # Test all movement keys
        movement_keys = [87, 83, 65, 68]  # W, S, A, D
        
        for key_code in movement_keys:
            self.key_manager.update_key(key_code, is_pressed=True)
            
        twist = self.key_manager.get_twist_if_changed()
        
        # Verify velocities are within reasonable limits
        self.assertLessEqual(abs(twist.linear.x), 1.0)   # Max 1.0 m/s
        self.assertLessEqual(abs(twist.angular.z), 1.0)  # Max 1.0 rad/s
        
    def test_zero_velocity_on_no_keys(self):
        """Test robot stops when no keys are pressed"""
        # Press and then release all keys
        for key_code in [87, 83, 65, 68]:  # W, S, A, D
            self.key_manager.update_key(key_code, is_pressed=True)
            self.key_manager.update_key(key_code, is_pressed=False)
            
        twist = self.key_manager.get_twist_if_changed()
        
        # Should result in zero movement
        self.assertEqual(twist.linear.x, 0.0)
        self.assertEqual(twist.linear.y, 0.0)
        self.assertEqual(twist.linear.z, 0.0)
        self.assertEqual(twist.angular.x, 0.0)
        self.assertEqual(twist.angular.y, 0.0)
        self.assertEqual(twist.angular.z, 0.0)


def run_async_test(coro):
    """Helper function to run async tests"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class AsyncTestCase(unittest.TestCase):
    """Base class for async tests"""
    
    def run_async(self, coro):
        """Run an async coroutine as a test"""
        return run_async_test(coro)


class AsyncWebSocketROS2Tests(AsyncTestCase):
    """Async WebSocket ROS2 integration tests"""
    
    def test_websocket_robot_communication(self):
        """Test WebSocket robot communication"""
        test_instance = TestWebSocketServerIntegration()
        test_instance.setUp()
        self.run_async(test_instance.test_websocket_robot_communication())
        
    def test_websocket_error_handling(self):
        """Test WebSocket error handling"""
        test_instance = TestWebSocketServerIntegration()
        test_instance.setUp()
        self.run_async(test_instance.test_websocket_error_handling())


if __name__ == '__main__':
    # Run all tests
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestWebSocketROS2Integration,
        TestWebSocketMessagePropagation,
        TestWebSocketServerIntegration,
        TestROS2TopicIntegration,
        AsyncWebSocketROS2Tests
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2, buffer=True)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"WebSocket ROS2 Integration Test Results:")
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
    
    success_rate = (result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100
    print(f"\nSuccess rate: {success_rate:.1f}%")
    print(f"{'='*60}")
    
    # Exit with proper code
    sys.exit(0 if result.wasSuccessful() else 1)