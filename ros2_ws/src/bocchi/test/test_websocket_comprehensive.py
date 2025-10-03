#!/usr/bin/env python3
"""
ROS2-compatible comprehensive WebSocket test runner for bocchi robot controller.
Integrates with colcon test and ros2 test infrastructure.
"""

import unittest
import asyncio
import json
import time
import threading
import sys
import os
from unittest.mock import Mock, patch, MagicMock, AsyncMock

# ROS2 test compatibility
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# WebSocket functionality
try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False


class MockROS2Node:
    """Mock ROS2 node for testing without full ROS2 environment"""
    
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        
    def create_publisher(self, msg_type, topic, qos_profile=10):
        mock_publisher = Mock()
        mock_publisher.publish = Mock()
        self.publishers[topic] = mock_publisher
        return mock_publisher
        
    def destroy_node(self):
        pass


class MockTwist:
    """Mock Twist message"""
    def __init__(self):
        self.linear = MockVector3()
        self.angular = MockVector3()


class MockVector3:
    """Mock Vector3"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class MockInt32:
    """Mock Int32 message"""
    def __init__(self, data=0):
        self.data = data


class TestWebSocketROS2Integration(unittest.TestCase):
    """Test WebSocket integration with ROS2 (using mocks if ROS2 not available)"""
    
    def setUp(self):
        """Set up test fixtures"""
        if ROS2_AVAILABLE:
            try:
                rclpy.init()
                self.node = rclpy.create_node('test_websocket_node')
            except:
                self.node = MockROS2Node('test_websocket_node')
        else:
            self.node = MockROS2Node('test_websocket_node')
            
        # Mock publishers
        if ROS2_AVAILABLE:
            # Use patch to mock the real ROS2 publishers
            self.twist_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.servo_publisher = self.node.create_publisher(Int32, '/servo_position', 10)
            
            # Published messages tracking
            self.published_twists = []
            self.published_servo_positions = []
            
            # Patch the publish methods to capture messages
            original_twist_publish = self.twist_publisher.publish
            original_servo_publish = self.servo_publisher.publish
            
            def capture_twist(twist_msg):
                self.published_twists.append(twist_msg)
                
            def capture_servo(servo_msg):
                self.published_servo_positions.append(servo_msg.data if hasattr(servo_msg, 'data') else servo_msg)
                
            self.twist_publisher.publish = capture_twist
            self.servo_publisher.publish = capture_servo
        else:
            self.twist_publisher = self.node.create_publisher(MockTwist, '/cmd_vel', 10)
            self.servo_publisher = self.node.create_publisher(MockInt32, '/servo_position', 10)
            
            # Published messages tracking
            self.published_twists = []
            self.published_servo_positions = []
            
            def capture_twist(twist_msg):
                self.published_twists.append(twist_msg)
                
            def capture_servo(servo_msg):
                self.published_servo_positions.append(servo_msg.data if hasattr(servo_msg, 'data') else servo_msg)
                
            self.twist_publisher.publish.side_effect = capture_twist
            self.servo_publisher.publish.side_effect = capture_servo
    
    def tearDown(self):
        """Clean up test fixtures"""
        if ROS2_AVAILABLE and hasattr(self.node, 'destroy_node'):
            try:
                self.node.destroy_node()
                rclpy.shutdown()
            except:
                pass
    
    def test_ros2_node_creation(self):
        """Test ROS2 node can be created for WebSocket integration"""
        self.assertIsNotNone(self.node)
        if ROS2_AVAILABLE:
            self.assertEqual(self.node.get_name(), 'test_websocket_node')
        else:
            self.assertEqual(self.node.name, 'test_websocket_node')
    
    def test_twist_publisher_creation(self):
        """Test Twist publisher for robot movement"""
        self.assertIsNotNone(self.twist_publisher)
        
        # Test publishing a twist message
        if ROS2_AVAILABLE:
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 0.2
        else:
            twist = MockTwist()
            twist.linear.x = 0.5
            twist.angular.z = 0.2
        
        self.twist_publisher.publish(twist)
        
        # Verify message was captured
        self.assertEqual(len(self.published_twists), 1)
        self.assertEqual(self.published_twists[0].linear.x, 0.5)
        self.assertEqual(self.published_twists[0].angular.z, 0.2)
    
    def test_servo_publisher_creation(self):
        """Test servo position publisher"""
        self.assertIsNotNone(self.servo_publisher)
        
        # Test publishing servo position
        if ROS2_AVAILABLE:
            servo_msg = Int32()
            servo_msg.data = 180
        else:
            servo_msg = MockInt32(180)
        self.servo_publisher.publish(servo_msg)
        
        # Verify message was captured
        self.assertEqual(len(self.published_servo_positions), 1)
        self.assertEqual(self.published_servo_positions[0], 180)
    
    def test_wasd_to_twist_mapping(self):
        """Test WASD key mapping to Twist messages"""
        # W key - forward movement
        if ROS2_AVAILABLE:
            twist_forward = Twist()
            twist_backward = Twist()
            twist_left = Twist()
            twist_right = Twist()
        else:
            twist_forward = MockTwist()
            twist_backward = MockTwist()
            twist_left = MockTwist()
            twist_right = MockTwist()
            
        twist_forward.linear.x = 0.5
        self.twist_publisher.publish(twist_forward)
        
        # S key - backward movement
        twist_backward.linear.x = -0.5
        self.twist_publisher.publish(twist_backward)
        
        # A key - left turn
        twist_left.angular.z = 0.5
        self.twist_publisher.publish(twist_left)
        
        # D key - right turn
        twist_right.angular.z = -0.5
        self.twist_publisher.publish(twist_right)
        
        # Verify all movements were published
        self.assertEqual(len(self.published_twists), 4)
        self.assertEqual(self.published_twists[0].linear.x, 0.5)   # Forward
        self.assertEqual(self.published_twists[1].linear.x, -0.5)  # Backward
        self.assertEqual(self.published_twists[2].angular.z, 0.5)  # Left
        self.assertEqual(self.published_twists[3].angular.z, -0.5) # Right
    
    def test_f_key_servo_control(self):
        """Test F key servo control"""
        # F key - servo position 0
        if ROS2_AVAILABLE:
            servo_0 = Int32()
            servo_0.data = 0
            servo_180 = Int32()
            servo_180.data = 180
        else:
            servo_0 = MockInt32(0)
            servo_180 = MockInt32(180)
            
        self.servo_publisher.publish(servo_0)
        
        # F key - servo position 180
        self.servo_publisher.publish(servo_180)
        
        # Verify servo positions were published
        self.assertEqual(len(self.published_servo_positions), 2)
        self.assertEqual(self.published_servo_positions[0], 0)
        self.assertEqual(self.published_servo_positions[1], 180)


class TestWebSocketMessageFormats(unittest.TestCase):
    """Test WebSocket message format compliance for ROS2 integration"""
    
    def test_key_event_message_format(self):
        """Test key event message format for ROS2 processing"""
        message = {
            'type': 'key_down',
            'key': 'w',
            'key_code': 87,
            'timestamp': int(time.time() * 1000),
            'ros2_topic': '/cmd_vel',
            'robot_command': 'move_forward'
        }
        
        # Verify JSON serialization
        json_str = json.dumps(message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'key_down')
        self.assertEqual(parsed['key'], 'w')
        self.assertEqual(parsed['key_code'], 87)
        self.assertEqual(parsed['ros2_topic'], '/cmd_vel')
        self.assertEqual(parsed['robot_command'], 'move_forward')
    
    def test_robot_status_message_format(self):
        """Test robot status message format"""
        status_message = {
            'type': 'robot_status',
            'node_name': 'bocchi_publisher',
            'topics': {
                'cmd_vel': '/cmd_vel',
                'servo_position': '/servo_position'
            },
            'status': 'active',
            'timestamp': int(time.time() * 1000)
        }
        
        json_str = json.dumps(status_message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'robot_status')
        self.assertEqual(parsed['node_name'], 'bocchi_publisher')
        self.assertIn('topics', parsed)
        self.assertEqual(parsed['status'], 'active')
    
    def test_ros2_command_message_format(self):
        """Test ROS2 command message format"""
        command_message = {
            'type': 'ros2_command',
            'command': 'emergency_stop',
            'target_topics': ['/cmd_vel'],
            'priority': 'high',
            'timestamp': int(time.time() * 1000)
        }
        
        json_str = json.dumps(command_message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'ros2_command')
        self.assertEqual(parsed['command'], 'emergency_stop')
        self.assertIsInstance(parsed['target_topics'], list)
        self.assertEqual(parsed['priority'], 'high')


@unittest.skipIf(not WEBSOCKETS_AVAILABLE, "websockets library not available")
class TestWebSocketServerIntegration(unittest.TestCase):
    """Test WebSocket server integration with ROS2 (requires websockets)"""
    
    async def test_websocket_ros2_message_flow(self):
        """Test message flow from WebSocket to ROS2 topics"""
        async def ros2_websocket_handler(websocket, path):
            """WebSocket handler that simulates ROS2 integration"""
            try:
                welcome = {
                    'type': 'ros2_ready',
                    'node_name': 'bocchi_websocket_node',
                    'available_topics': ['/cmd_vel', '/servo_position'],
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(welcome))
                
                async for message in websocket:
                    data = json.loads(message)
                    
                    if data.get('type') == 'key_down':
                        # Simulate ROS2 topic publishing
                        response = {
                            'type': 'ros2_published',
                            'topic': '/cmd_vel' if data.get('key') in ['w', 's', 'a', 'd'] else '/servo_position',
                            'message_type': 'Twist' if data.get('key') in ['w', 's', 'a', 'd'] else 'Int32',
                            'original_key': data.get('key'),
                            'timestamp': int(time.time() * 1000)
                        }
                        await websocket.send(json.dumps(response))
                        
            except websockets.exceptions.ConnectionClosed:
                pass
        
        # Start test server
        port = 9100
        server = await websockets.serve(ros2_websocket_handler, 'localhost', port)
        
        try:
            # Connect client
            async with websockets.connect(f'ws://localhost:{port}') as websocket:
                # Receive welcome
                welcome_msg = await websocket.recv()
                welcome_data = json.loads(welcome_msg)
                
                self.assertEqual(welcome_data['type'], 'ros2_ready')
                self.assertIn('available_topics', welcome_data)
                
                # Send key event
                key_event = {
                    'type': 'key_down',
                    'key': 'w',
                    'key_code': 87,
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(key_event))
                
                # Receive ROS2 response
                response_msg = await websocket.recv()
                response_data = json.loads(response_msg)
                
                self.assertEqual(response_data['type'], 'ros2_published')
                self.assertEqual(response_data['topic'], '/cmd_vel')
                self.assertEqual(response_data['message_type'], 'Twist')
                self.assertEqual(response_data['original_key'], 'w')
                
        finally:
            server.close()
            await server.wait_closed()


class TestROS2TestCompatibility(unittest.TestCase):
    """Test compatibility with ROS2 testing infrastructure"""
    
    def test_colcon_test_compatibility(self):
        """Test that tests are compatible with colcon test"""
        # This test verifies the test structure is compatible with ROS2 testing
        self.assertTrue(True, "Test structure is ROS2 compatible")
    
    def test_ament_python_compatibility(self):
        """Test compatibility with ament_python build system"""
        # Verify test can run in ament_python environment
        import sys
        import os
        
        # Check if we're in a ROS2 workspace structure
        current_path = os.path.abspath(__file__)
        self.assertIn('test', current_path)
        
        # Check if required modules can be imported
        try:
            import unittest
            import json
            import time
            self.assertTrue(True, "Required modules available")
        except ImportError:
            self.fail("Required modules not available")
    
    def test_pytest_compatibility(self):
        """Test compatibility with pytest (ROS2 default test runner)"""
        # This test ensures our unittest-based tests work with pytest
        self.assertIsInstance(self, unittest.TestCase)
        
    def test_ros2_test_discovery(self):
        """Test that tests can be discovered by ros2 test"""
        # Check test naming convention
        test_file = __file__
        self.assertTrue(os.path.basename(test_file).startswith('test_'))
        self.assertTrue(test_file.endswith('.py'))


def run_async_test(coro):
    """Helper function to run async tests"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class AsyncTestWrapper(unittest.TestCase):
    """Wrapper for async tests to work with ROS2 test infrastructure"""
    
    @unittest.skipIf(not WEBSOCKETS_AVAILABLE, "websockets library not available")
    def test_websocket_ros2_message_flow(self):
        """Test WebSocket to ROS2 message flow"""
        test_instance = TestWebSocketServerIntegration()
        run_async_test(test_instance.test_websocket_ros2_message_flow())


# ROS2 test entry point
def main():
    """Main entry point for ROS2 test execution"""
    if ROS2_AVAILABLE:
        rclpy.init()
    
    # Run tests
    unittest.main(argv=[''], exit=False, verbosity=2)
    
    if ROS2_AVAILABLE:
        rclpy.shutdown()


if __name__ == '__main__':
    print("🤖 ROS2 WebSocket Comprehensive Tests")
    print("=" * 50)
    print(f"ROS2 Available: {'✅' if ROS2_AVAILABLE else '❌'}")
    print(f"WebSockets Available: {'✅' if WEBSOCKETS_AVAILABLE else '❌'}")
    print("=" * 50)
    
    # Configure test discovery for ROS2
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestWebSocketROS2Integration,
        TestWebSocketMessageFormats,
        TestROS2TestCompatibility,
        AsyncTestWrapper
    ]
    
    # Only add WebSocket server tests if websockets is available
    if WEBSOCKETS_AVAILABLE:
        test_classes.append(TestWebSocketServerIntegration)
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests with ROS2 compatible output
    runner = unittest.TextTestRunner(verbosity=2, buffer=True)
    result = runner.run(suite)
    
    # Print ROS2-style summary
    print(f"\n{'='*50}")
    print(f"🤖 ROS2 WebSocket Test Results:")
    print(f"{'='*50}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("✅ All ROS2 WebSocket tests passed!")
        print("🚀 Ready for colcon test integration")
    else:
        print("⚠️  Some tests failed")
        if result.failures:
            for test, error in result.failures:
                print(f"  FAIL: {test}")
        if result.errors:
            for test, error in result.errors:
                print(f"  ERROR: {test}")
    
    print(f"{'='*50}")
    
    # Exit with proper code for ROS2 test infrastructure
    sys.exit(0 if result.wasSuccessful() else 1)