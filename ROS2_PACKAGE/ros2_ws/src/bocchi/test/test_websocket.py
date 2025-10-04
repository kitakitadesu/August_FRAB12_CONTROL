#!/usr/bin/env python3

import unittest
import asyncio
import websockets
import json
import time
import threading
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the bocchi package to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bocchi.publisher import WebSocketManager, KeyStateManager

class TestWebSocketManager(unittest.TestCase):
    """Test WebSocket manager functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.ws_manager = WebSocketManager()
        
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
        
        # Removing non-existent client should not cause error
        self.ws_manager.remove_client(mock_websocket)
        self.assertEqual(len(self.ws_manager.connected_clients), 0)
        
    async def test_broadcast_empty_clients(self):
        """Test broadcasting with no connected clients"""
        message = {'type': 'test', 'data': 'hello'}
        
        # Should not raise any exception
        await self.ws_manager.broadcast(message)
        
    async def test_broadcast_with_clients(self):
        """Test broadcasting to connected clients"""
        mock_websocket1 = Mock()
        mock_websocket2 = Mock()
        
        # Mock the send method to be async
        async def mock_send(data):
            pass
        
        mock_websocket1.send = Mock(side_effect=mock_send)
        mock_websocket2.send = Mock(side_effect=mock_send)
        
        self.ws_manager.add_client(mock_websocket1)
        self.ws_manager.add_client(mock_websocket2)
        
        message = {'type': 'test', 'data': 'hello'}
        await self.ws_manager.broadcast(message)
        
        expected_json = json.dumps(message)
        mock_websocket1.send.assert_called_once_with(expected_json)
        mock_websocket2.send.assert_called_once_with(expected_json)
        
    async def test_broadcast_with_disconnected_client(self):
        """Test broadcasting handles disconnected clients"""
        mock_websocket1 = Mock()
        mock_websocket2 = Mock()
        
        # Mock websocket1 to raise ConnectionClosed
        async def mock_send_error(data):
            raise websockets.exceptions.ConnectionClosed(None, None)
        
        async def mock_send_success(data):
            pass
        
        mock_websocket1.send = Mock(side_effect=mock_send_error)
        mock_websocket2.send = Mock(side_effect=mock_send_success)
        
        self.ws_manager.add_client(mock_websocket1)
        self.ws_manager.add_client(mock_websocket2)
        
        message = {'type': 'test', 'data': 'hello'}
        await self.ws_manager.broadcast(message)
        
        # Disconnected client should be removed
        self.assertNotIn(mock_websocket1, self.ws_manager.connected_clients)
        self.assertIn(mock_websocket2, self.ws_manager.connected_clients)


class TestKeyStateManagerWithWebSocket(unittest.TestCase):
    """Test KeyStateManager functionality for WebSocket integration"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.key_manager = KeyStateManager(debounce_time=0.01)
        
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
        
    def test_key_release_partial(self):
        """Test releasing one key while others are still pressed"""
        # Press W and A
        self.key_manager.update_key(87, is_pressed=True)  # W
        self.key_manager.update_key(65, is_pressed=True)  # A
        
        # Release W, keep A
        self.key_manager.update_key(87, is_pressed=False)  # Release W
        
        twist = self.key_manager.get_twist_if_changed()
        self.assertIsNotNone(twist)
        self.assertEqual(twist.linear.x, 0.0)   # No forward movement
        self.assertEqual(twist.angular.z, 0.5)  # Still turning left


class TestWebSocketIntegration(unittest.TestCase):
    """Integration tests for WebSocket functionality"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        self.server_host = 'localhost'
        self.server_port = 8766  # Use different port for testing
        self.websocket_url = f'ws://{self.server_host}:{self.server_port}'
        self.server = None
        self.received_messages = []
        
    async def mock_websocket_handler(self, websocket, path):
        """Mock WebSocket handler for testing"""
        try:
            async for message in websocket:
                data = json.loads(message)
                self.received_messages.append(data)
                
                # Echo back the message
                response = {
                    'type': 'echo',
                    'original': data,
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(response))
        except websockets.exceptions.ConnectionClosed:
            pass
            
    async def start_test_server(self):
        """Start test WebSocket server"""
        self.server = await websockets.serve(
            self.mock_websocket_handler,
            self.server_host,
            self.server_port
        )
        
    async def stop_test_server(self):
        """Stop test WebSocket server"""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            
    async def test_websocket_connection(self):
        """Test WebSocket connection establishment"""
        await self.start_test_server()
        
        try:
            # Connect to test server
            async with websockets.connect(self.websocket_url) as websocket:
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
            await self.stop_test_server()
            
    async def test_key_event_messaging(self):
        """Test key event message structure"""
        await self.start_test_server()
        
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                # Send key down event
                key_down_message = {
                    'type': 'key_down',
                    'key': 'w',
                    'key_code': 87,
                    'timestamp': int(time.time() * 1000)
                }
                await websocket.send(json.dumps(key_down_message))
                
                # Receive echo
                response = await websocket.recv()
                response_data = json.loads(response)
                
                self.assertEqual(response_data['original']['type'], 'key_down')
                self.assertEqual(response_data['original']['key'], 'w')
                self.assertEqual(response_data['original']['key_code'], 87)
                
        finally:
            await self.stop_test_server()


class TestWebSocketProtocol(unittest.TestCase):
    """Test WebSocket protocol compliance"""
    
    def test_message_format_key_down(self):
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
        
    def test_message_format_key_up(self):
        """Test key up message format"""
        message = {
            'type': 'key_up',
            'key': 'w',
            'key_code': 87,
            'timestamp': int(time.time() * 1000)
        }
        
        json_str = json.dumps(message)
        parsed = json.loads(json_str)
        
        self.assertEqual(parsed['type'], 'key_up')
        
    def test_message_format_welcome(self):
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


def run_async_test(coro):
    """Helper function to run async tests"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


# Async test runner wrapper
class AsyncTestCase(unittest.TestCase):
    """Base class for async tests"""
    
    def run_async(self, coro):
        """Run an async coroutine as a test"""
        return run_async_test(coro)


class TestAsyncWebSocketFunctionality(AsyncTestCase):
    """Async WebSocket functionality tests"""
    
    def test_websocket_manager_broadcast(self):
        """Test WebSocket manager broadcast functionality"""
        async def async_test():
            ws_manager = WebSocketManager()
            await ws_manager.broadcast({'type': 'test'})
            # Should complete without error even with no clients
            
        self.run_async(async_test())
        
    def test_websocket_connection_lifecycle(self):
        """Test WebSocket connection lifecycle"""
        async def async_test():
            # Start a simple echo server
            async def echo_handler(websocket, path):
                async for message in websocket:
                    await websocket.send(f"Echo: {message}")
                    
            server = await websockets.serve(echo_handler, 'localhost', 8767)
            
            try:
                # Connect and test
                async with websockets.connect('ws://localhost:8767') as websocket:
                    await websocket.send("Hello")
                    response = await websocket.recv()
                    self.assertEqual(response, "Echo: Hello")
            finally:
                server.close()
                await server.wait_closed()
                
        self.run_async(async_test())


if __name__ == '__main__':
    # Run all tests
    unittest.main(verbosity=2)