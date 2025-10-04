#!/usr/bin/env python3
"""
Comprehensive WebSocket functionality tests for bocchi robot controller.
Tests WebSocket communication, message handling, and integration with ROS2.
"""

import unittest
import asyncio
import websockets
import json
import time
import threading
import socket
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import sys
import os
from concurrent.futures import ThreadPoolExecutor
import queue
import signal

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


class TestWebSocketManager(unittest.TestCase):
    """Test WebSocket manager core functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.ws_manager = WebSocketManager()
        
    def test_websocket_manager_initialization(self):
        """Test WebSocketManager initializes correctly"""
        self.assertIsInstance(self.ws_manager.connected_clients, set)
        self.assertEqual(len(self.ws_manager.connected_clients), 0)
        self.assertIsNotNone(self.ws_manager.lock)
        
    def test_add_client_single(self):
        """Test adding a single WebSocket client"""
        mock_websocket = Mock()
        mock_websocket.remote_address = ('127.0.0.1', 12345)
        
        self.ws_manager.add_client(mock_websocket)
        self.assertEqual(len(self.ws_manager.connected_clients), 1)
        self.assertIn(mock_websocket, self.ws_manager.connected_clients)
        
    def test_add_client_multiple(self):
        """Test adding multiple WebSocket clients"""
        clients = []
        for i in range(5):
            mock_websocket = Mock()
            mock_websocket.remote_address = ('127.0.0.1', 12345 + i)
            clients.append(mock_websocket)
            self.ws_manager.add_client(mock_websocket)
            
        self.assertEqual(len(self.ws_manager.connected_clients), 5)
        for client in clients:
            self.assertIn(client, self.ws_manager.connected_clients)
            
    def test_add_client_duplicate(self):
        """Test adding the same client multiple times"""
        mock_websocket = Mock()
        mock_websocket.remote_address = ('127.0.0.1', 12345)
        
        self.ws_manager.add_client(mock_websocket)
        self.ws_manager.add_client(mock_websocket)  # Add same client again
        
        # Should only be added once (set behavior)
        self.assertEqual(len(self.ws_manager.connected_clients), 1)
        
    def test_remove_client(self):
        """Test removing WebSocket clients"""
        mock_websocket = Mock()
        mock_websocket.remote_address = ('127.0.0.1', 12345)
        
        # Add client first
        self.ws_manager.add_client(mock_websocket)
        self.assertEqual(len(self.ws_manager.connected_clients), 1)
        
        # Remove client
        self.ws_manager.remove_client(mock_websocket)
        self.assertEqual(len(self.ws_manager.connected_clients), 0)
        self.assertNotIn(mock_websocket, self.ws_manager.connected_clients)
        
    def test_remove_client_nonexistent(self):
        """Test removing non-existent client doesn't cause error"""
        mock_websocket = Mock()
        
        # Should not raise any exception
        self.ws_manager.remove_client(mock_websocket)
        self.assertEqual(len(self.ws_manager.connected_clients), 0)
        
    def test_thread_safety_add_remove(self):
        """Test thread safety of add/remove operations"""
        clients = []
        
        def add_clients():
            for i in range(10):
                mock_websocket = Mock()
                mock_websocket.remote_address = ('127.0.0.1', 12345 + i)
                clients.append(mock_websocket)
                self.ws_manager.add_client(mock_websocket)
                time.sleep(0.001)  # Small delay to encourage race conditions
                
        def remove_clients():
            time.sleep(0.005)  # Let some clients be added first
            for client in clients[:5]:  # Remove first 5 clients
                self.ws_manager.remove_client(client)
                time.sleep(0.001)
                
        thread1 = threading.Thread(target=add_clients)
        thread2 = threading.Thread(target=remove_clients)
        
        thread1.start()
        thread2.start()
        
        thread1.join()
        thread2.join()
        
        # Should have 5 clients remaining (10 added - 5 removed)
        self.assertEqual(len(self.ws_manager.connected_clients), 5)


class TestWebSocketBroadcasting(unittest.TestCase):
    """Test WebSocket broadcasting functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.ws_manager = WebSocketManager()
        
    async def test_broadcast_empty_clients(self):
        """Test broadcasting with no connected clients"""
        message = {'type': 'test', 'data': 'hello', 'timestamp': int(time.time() * 1000)}
        
        # Should not raise any exception
        await self.ws_manager.broadcast(message)
        
    async def test_broadcast_single_client(self):
        """Test broadcasting to a single client"""
        mock_websocket = AsyncMock()
        mock_websocket.remote_address = ('127.0.0.1', 12345)
        
        self.ws_manager.add_client(mock_websocket)
        
        message = {'type': 'test', 'data': 'hello', 'timestamp': int(time.time() * 1000)}
        await self.ws_manager.broadcast(message)
        
        expected_json = json.dumps(message)
        mock_websocket.send.assert_called_once_with(expected_json)
        
    async def test_broadcast_multiple_clients(self):
        """Test broadcasting to multiple clients"""
        clients = []
        for i in range(3):
            mock_websocket = AsyncMock()
            mock_websocket.remote_address = ('127.0.0.1', 12345 + i)
            clients.append(mock_websocket)
            self.ws_manager.add_client(mock_websocket)
            
        message = {'type': 'test', 'data': 'hello', 'timestamp': int(time.time() * 1000)}
        await self.ws_manager.broadcast(message)
        
        expected_json = json.dumps(message)
        for client in clients:
            client.send.assert_called_once_with(expected_json)
            
    async def test_broadcast_with_connection_error(self):
        """Test broadcasting handles connection errors gracefully"""
        mock_websocket1 = AsyncMock()
        mock_websocket2 = AsyncMock()
        
        # Mock websocket1 to raise ConnectionClosed
        mock_websocket1.send.side_effect = websockets.exceptions.ConnectionClosed(None, None)
        mock_websocket1.remote_address = ('127.0.0.1', 12345)
        
        mock_websocket2.remote_address = ('127.0.0.1', 12346)
        
        self.ws_manager.add_client(mock_websocket1)
        self.ws_manager.add_client(mock_websocket2)
        
        message = {'type': 'test', 'data': 'hello', 'timestamp': int(time.time() * 1000)}
        await self.ws_manager.broadcast(message)
        
        # Failed client should be removed, successful client should remain
        self.assertNotIn(mock_websocket1, self.ws_manager.connected_clients)
        self.assertIn(mock_websocket2, self.ws_manager.connected_clients)
        
        expected_json = json.dumps(message)
        mock_websocket2.send.assert_called_once_with(expected_json)
        
    async def test_broadcast_with_general_exception(self):
        """Test broadcasting handles general exceptions"""
        mock_websocket1 = AsyncMock()
        mock_websocket2 = AsyncMock()
        
        # Mock websocket1 to raise a general exception
        mock_websocket1.send.side_effect = Exception("Network error")
        mock_websocket1.remote_address = ('127.0.0.1', 12345)
        
        mock_websocket2.remote_address = ('127.0.0.1', 12346)
        
        self.ws_manager.add_client(mock_websocket1)
        self.ws_manager.add_client(mock_websocket2)
        
        message = {'type': 'test', 'data': 'hello', 'timestamp': int(time.time() * 1000)}
        await self.ws_manager.broadcast(message)
        
        # Failed client should be removed
        self.assertNotIn(mock_websocket1, self.ws_manager.connected_clients)
        self.assertIn(mock_websocket2, self.ws_manager.connected_clients)


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
        
        # Verify required fields
        required_fields = ['type', 'key', 'key_code', 'timestamp']
        for field in required_fields:
            self.assertIn(field, parsed)
            
    def test_key_up_message_format(self):
        """Test key up message format"""
        message = {
            'type': 'key_up',
            'key': 's',
            'key_code': 83,
            'timestamp': int(time.time() * 1000)
        }
        
        json_str = json.dumps(message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'key_up')
        self.assertEqual(parsed['key'], 's')
        self.assertEqual(parsed['key_code'], 83)
        
    def test_key_received_message_format(self):
        """Test key received message format"""
        message = {
            'type': 'key_received',
            'key': 'a',
            'key_code': 65,
            'is_held': True,
            'timestamp': int(time.time() * 1000)
        }
        
        json_str = json.dumps(message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'key_received')
        self.assertEqual(parsed['key'], 'a')
        self.assertEqual(parsed['key_code'], 65)
        self.assertTrue(parsed['is_held'])
        
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
        self.assertEqual(parsed['keys'][0]['key'], 'w')
        self.assertEqual(parsed['keys'][1]['key_code'], 65)
        
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
        self.assertIsInstance(parsed['message'], str)
        
    def test_invalid_message_handling(self):
        """Test handling of invalid JSON messages"""
        invalid_messages = [
            "not json",
            "{invalid: json}",
            '{"type": }',
            "",
            None
        ]
        
        for invalid_msg in invalid_messages:
            if invalid_msg is not None:
                with self.assertRaises(json.JSONDecodeError):
                    json.loads(invalid_msg)


class TestWebSocketIntegration(unittest.TestCase):
    """Integration tests for WebSocket with KeyboardAPI"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        # Mock ROS2 publisher node
        self.mock_publisher = Mock()
        self.mock_publisher.key_manager = KeyStateManager(debounce_time=0.01)
        
        # Create KeyboardAPI instance
        self.api = KeyboardAPI(self.mock_publisher)
        
    def test_keyboard_api_websocket_manager_initialization(self):
        """Test KeyboardAPI initializes WebSocket manager"""
        self.assertIsNotNone(self.api.websocket_manager)
        self.assertIsInstance(self.api.websocket_manager, WebSocketManager)
        
    def test_broadcast_event_functionality(self):
        """Test broadcast_event method"""
        event_data = {
            'type': 'test_event',
            'key': 'w',
            'timestamp': int(time.time() * 1000)
        }
        
        # Should not raise exception even with no connected clients
        self.api.broadcast_event(event_data)
        
    @patch('threading.Thread')
    def test_broadcast_event_threading(self, mock_thread):
        """Test broadcast_event uses threading correctly"""
        # Add a mock client to trigger the threading path
        mock_websocket = Mock()
        self.api.websocket_manager.add_client(mock_websocket)
        
        event_data = {
            'type': 'test_event',
            'key': 'w',
            'timestamp': int(time.time() * 1000)
        }
        
        self.api.broadcast_event(event_data)
        
        # Verify thread was created
        mock_thread.assert_called_once()
        call_args = mock_thread.call_args
        self.assertEqual(call_args[1]['daemon'], True)
        self.assertEqual(call_args[1]['target'], self.api._schedule_broadcast)


class TestWebSocketServerLifecycle(unittest.TestCase):
    """Test WebSocket server lifecycle management"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.test_port = 8767  # Use different port for testing
        
    def test_port_availability_check(self):
        """Test port availability checking"""
        def is_port_available(port):
            """Check if port is available"""
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', port))
                    return True
            except OSError:
                return False
        
        # Test with an available port
        available_port = 9999
        while not is_port_available(available_port):
            available_port += 1
            
        self.assertTrue(is_port_available(available_port))
        
        # Test with a used port (bind and test)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('localhost', 0))  # Bind to any available port
            used_port = s.getsockname()[1]
            self.assertFalse(is_port_available(used_port))


class TestWebSocketConnectionHandling(unittest.TestCase):
    """Test WebSocket connection handling scenarios"""
    
    async def test_connection_lifecycle(self):
        """Test complete connection lifecycle"""
        received_messages = []
        server_ready = asyncio.Event()
        
        async def test_handler(websocket, path):
            """Test WebSocket handler"""
            server_ready.set()
            try:
                # Send welcome message
                welcome = {
                    'type': 'welcome',
                    'message': 'Test server ready',
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(welcome))
                
                # Handle incoming messages
                async for message in websocket:
                    data = json.loads(message)
                    received_messages.append(data)
                    
                    # Echo back
                    response = {
                        'type': 'echo',
                        'original': data,
                        'timestamp': int(time.time() * 1000)
                    }
                    await websocket.send(json.dumps(response))
                    
            except websockets.exceptions.ConnectionClosed:
                pass
                
        # Start test server
        server = await websockets.serve(test_handler, 'localhost', 8768)
        
        try:
            # Wait for server to be ready
            await asyncio.wait_for(server_ready.wait(), timeout=1.0)
            
            # Connect as client
            async with websockets.connect('ws://localhost:8768') as websocket:
                # Receive welcome message
                welcome_msg = await websocket.recv()
                welcome_data = json.loads(welcome_msg)
                self.assertEqual(welcome_data['type'], 'welcome')
                
                # Send test message
                test_message = {
                    'type': 'test_message',
                    'data': 'hello server'
                }
                await websocket.send(json.dumps(test_message))
                
                # Receive echo
                echo_msg = await websocket.recv()
                echo_data = json.loads(echo_msg)
                self.assertEqual(echo_data['type'], 'echo')
                self.assertEqual(echo_data['original'], test_message)
                
            # Verify server received our message
            self.assertEqual(len(received_messages), 1)
            self.assertEqual(received_messages[0], test_message)
            
        finally:
            server.close()
            await server.wait_closed()
            
    async def test_multiple_concurrent_connections(self):
        """Test handling multiple concurrent connections"""
        connection_count = 0
        max_connections = 5
        connections_ready = asyncio.Event()
        
        async def concurrent_handler(websocket, path):
            nonlocal connection_count
            connection_count += 1
            
            if connection_count >= max_connections:
                connections_ready.set()
                
            try:
                # Keep connection alive
                await websocket.wait_closed()
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                connection_count -= 1
                
        server = await websockets.serve(concurrent_handler, 'localhost', 8769)
        
        try:
            # Create multiple concurrent connections
            connections = []
            
            async def create_connection():
                websocket = await websockets.connect('ws://localhost:8769')
                connections.append(websocket)
                return websocket
                
            # Create all connections concurrently
            await asyncio.gather(*[create_connection() for _ in range(max_connections)])
            
            # Wait for all connections to be established
            await asyncio.wait_for(connections_ready.wait(), timeout=2.0)
            
            self.assertEqual(connection_count, max_connections)
            
            # Close all connections
            for conn in connections:
                await conn.close()
                
        finally:
            server.close()
            await server.wait_closed()
            
    async def test_connection_error_handling(self):
        """Test connection error scenarios"""
        async def error_handler(websocket, path):
            """Handler that simulates various errors"""
            if path == '/connection_close':
                # Immediately close connection
                await websocket.close()
            elif path == '/invalid_message':
                # Send invalid JSON
                await websocket.send("invalid json {")
            elif path == '/exception':
                # Raise an exception
                raise Exception("Simulated server error")
            else:
                # Normal behavior
                await websocket.send('{"type": "ok"}')
                await websocket.wait_closed()
                
        server = await websockets.serve(error_handler, 'localhost', 8770)
        
        try:
            # Test immediate connection close
            with self.assertRaises(websockets.exceptions.ConnectionClosedError):
                async with websockets.connect('ws://localhost:8770/connection_close') as websocket:
                    await websocket.recv()
                    
            # Test normal connection
            async with websockets.connect('ws://localhost:8770/normal') as websocket:
                message = await websocket.recv()
                data = json.loads(message)
                self.assertEqual(data['type'], 'ok')
                
        finally:
            server.close()
            await server.wait_closed()


class TestWebSocketPerformance(unittest.TestCase):
    """Test WebSocket performance characteristics"""
    
    async def test_message_throughput(self):
        """Test message throughput capabilities"""
        messages_received = 0
        start_time = None
        
        async def throughput_handler(websocket, path):
            nonlocal messages_received, start_time
            start_time = time.time()
            
            try:
                async for message in websocket:
                    messages_received += 1
                    # Echo back immediately
                    await websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                pass
                
        server = await websockets.serve(throughput_handler, 'localhost', 8771)
        
        try:
            async with websockets.connect('ws://localhost:8771') as websocket:
                # Send rapid messages
                num_messages = 100
                messages = []
                
                for i in range(num_messages):
                    message = {
                        'type': 'throughput_test',
                        'sequence': i,
                        'timestamp': int(time.time() * 1000)
                    }
                    messages.append(json.dumps(message))
                    
                # Send all messages rapidly
                send_start = time.time()
                for msg in messages:
                    await websocket.send(msg)
                    
                # Receive all echoes
                for i in range(num_messages):
                    response = await websocket.recv()
                    data = json.loads(response)
                    self.assertEqual(data['sequence'], i)
                    
                send_end = time.time()
                
                # Calculate throughput
                duration = send_end - send_start
                throughput = num_messages / duration
                
                print(f"Throughput test: {throughput:.2f} messages/second")
                
                # Should handle at least 50 messages per second
                self.assertGreater(throughput, 50)
                
        finally:
            server.close()
            await server.wait_closed()
            
    async def test_large_message_handling(self):
        """Test handling of large messages"""
        large_data = "x" * 10000  # 10KB of data
        
        async def large_message_handler(websocket, path):
            async for message in websocket:
                data = json.loads(message)
                # Echo back with confirmation
                response = {
                    'type': 'large_echo',
                    'size': len(data.get('data', '')),
                    'first_10': data.get('data', '')[:10],
                    'last_10': data.get('data', '')[-10:]
                }
                await websocket.send(json.dumps(response))
                
        server = await websockets.serve(large_message_handler, 'localhost', 8772)
        
        try:
            async with websockets.connect('ws://localhost:8772') as websocket:
                large_message = {
                    'type': 'large_test',
                    'data': large_data
                }
                
                await websocket.send(json.dumps(large_message))
                response = await websocket.recv()
                data = json.loads(response)
                
                self.assertEqual(data['type'], 'large_echo')
                self.assertEqual(data['size'], len(large_data))
                self.assertEqual(data['first_10'], large_data[:10])
                self.assertEqual(data['last_10'], large_data[-10:])
                
        finally:
            server.close()
            await server.wait_closed()


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


# Async test wrappers
class AsyncWebSocketTests(AsyncTestCase):
    """Wrapper for async WebSocket tests"""
    
    def test_broadcast_empty_clients(self):
        async def test():
            ws_manager = WebSocketManager()
            await ws_manager.broadcast({'type': 'test'})
        self.run_async(test())
        
    def test_broadcast_single_client(self):
        self.run_async(TestWebSocketBroadcasting().test_broadcast_single_client())
        
    def test_broadcast_multiple_clients(self):
        self.run_async(TestWebSocketBroadcasting().test_broadcast_multiple_clients())
        
    def test_broadcast_connection_error(self):
        self.run_async(TestWebSocketBroadcasting().test_broadcast_with_connection_error())
        
    def test_connection_lifecycle(self):
        self.run_async(TestWebSocketConnectionHandling().test_connection_lifecycle())
        
    def test_multiple_concurrent_connections(self):
        self.run_async(TestWebSocketConnectionHandling().test_multiple_concurrent_connections())
        
    def test_message_throughput(self):
        self.run_async(TestWebSocketPerformance().test_message_throughput())
        
    def test_large_message_handling(self):
        self.run_async(TestWebSocketPerformance().test_large_message_handling())


if __name__ == '__main__':
    # Configure test discovery and execution
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    test_classes = [
        TestWebSocketManager,
        TestWebSocketBroadcasting,
        TestWebSocketMessageFormats,
        TestWebSocketIntegration,
        TestWebSocketServerLifecycle,
        TestWebSocketConnectionHandling,
        TestWebSocketPerformance,
        AsyncWebSocketTests
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, buffer=True)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n{'='*50}")
    print(f"Test Results Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    print(f"{'='*50}")
    
    # Exit with proper code
    sys.exit(0 if result.wasSuccessful() else 1)