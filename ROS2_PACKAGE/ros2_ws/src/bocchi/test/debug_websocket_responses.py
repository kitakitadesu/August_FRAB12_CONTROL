#!/usr/bin/env python3
"""
Debug script to inspect WebSocket responses from bocchi robot controller.
This helps understand the exact response format for different message types.
"""

import asyncio
import json
import time

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("ERROR: websockets library required. Install with: pip install websockets")
    exit(1)


async def debug_websocket_responses():
    """Debug WebSocket responses by sending different message types"""
    
    if not WEBSOCKETS_AVAILABLE:
        print("WebSocket library not available")
        return
    
    print("=" * 60)
    print("DEBUG: WebSocket Response Inspector")
    print("Connecting to ws://localhost:8765")
    print("=" * 60)
    
    try:
        # Connect to WebSocket server
        websocket = await websockets.connect('ws://localhost:8765')
        print("✓ Connected successfully")
        
        # Receive welcome message
        welcome = await websocket.recv()
        welcome_data = json.loads(welcome)
        print(f"\n1. WELCOME MESSAGE:")
        print(json.dumps(welcome_data, indent=2))
        
        # Test different message types
        test_messages = [
            {
                'name': 'KEYBOARD MESSAGE',
                'message': {
                    'type': 'keyboard',
                    'key': 'w',
                    'key_code': 87,
                    'is_held': False,
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'KEY_DOWN MESSAGE',
                'message': {
                    'type': 'key_down',
                    'key': 'a',
                    'key_code': 65,
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'KEY_UP MESSAGE',
                'message': {
                    'type': 'key_up',
                    'key': 'a',
                    'key_code': 65,
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'SEND_KEY MESSAGE',
                'message': {
                    'type': 'send_key',
                    'key': 'd',
                    'key_code': 68,
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'GET_STATUS MESSAGE',
                'message': {
                    'type': 'get_status',
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'TEST_CONNECTION MESSAGE',
                'message': {
                    'type': 'test_connection',
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'SEND_KEY_BATCH MESSAGE',
                'message': {
                    'type': 'send_key_batch',
                    'keys': [
                        {'key': 'w', 'key_code': 87},
                        {'key': 's', 'key_code': 83}
                    ],
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'GET_ROUTES MESSAGE',
                'message': {
                    'type': 'get_routes',
                    'timestamp': int(time.time() * 1000)
                }
            },
            {
                'name': 'UNKNOWN MESSAGE TYPE',
                'message': {
                    'type': 'unknown_command',
                    'data': 'test',
                    'timestamp': int(time.time() * 1000)
                }
            }
        ]
        
        for i, test_case in enumerate(test_messages, 2):
            print(f"\n{i}. {test_case['name']}:")
            print(f"   SENT: {json.dumps(test_case['message'], indent=8)}")
            
            # Send message
            await websocket.send(json.dumps(test_case['message']))
            
            # Try to receive response with timeout
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                response_data = json.loads(response)
                print(f"   RECEIVED: {json.dumps(response_data, indent=8)}")
                
                # Analyze response
                response_type = response_data.get('type', 'NO_TYPE')
                success = response_data.get('success')
                error = response_data.get('error')
                
                print(f"   ANALYSIS:")
                print(f"     - Response type: {response_type}")
                print(f"     - Success field: {success}")
                print(f"     - Error field: {error}")
                
                if success is False and error:
                    print(f"     - Error details: {error}")
                
            except asyncio.TimeoutError:
                print("   RECEIVED: [TIMEOUT - No response within 2 seconds]")
            except json.JSONDecodeError as e:
                print(f"   RECEIVED: [JSON DECODE ERROR: {e}]")
            except Exception as e:
                print(f"   RECEIVED: [ERROR: {e}]")
            
            # Small delay between messages
            await asyncio.sleep(0.1)
        
        # Test invalid JSON
        print(f"\n{len(test_messages) + 2}. INVALID JSON TEST:")
        print("   SENT: invalid json {")
        try:
            await websocket.send("invalid json {")
            response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
            response_data = json.loads(response)
            print(f"   RECEIVED: {json.dumps(response_data, indent=8)}")
        except Exception as e:
            print(f"   RECEIVED: [ERROR: {e}]")
        
        await websocket.close()
        print("\n✓ Connection closed")
        
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False
    
    print("\n" + "=" * 60)
    print("DEBUG: WebSocket Response Inspection Complete")
    print("=" * 60)
    return True


async def test_specific_keyboard_message():
    """Test the specific keyboard message that's failing"""
    print("\n" + "=" * 60)
    print("SPECIFIC TEST: Keyboard Message Response")
    print("=" * 60)
    
    try:
        websocket = await websockets.connect('ws://localhost:8765')
        
        # Consume welcome message
        await websocket.recv()
        
        # Send the exact message from the failing test
        keyboard_msg = {
            'type': 'keyboard',
            'key': 'w',
            'key_code': 87,
            'is_held': False,
            'timestamp': int(time.time() * 1000)
        }
        
        print("Sending keyboard message:")
        print(json.dumps(keyboard_msg, indent=2))
        
        await websocket.send(json.dumps(keyboard_msg))
        
        # Receive response
        response = await asyncio.wait_for(websocket.recv(), timeout=3.0)
        response_data = json.loads(response)
        
        print("\nReceived response:")
        print(json.dumps(response_data, indent=2))
        
        # Check specific fields that the test is looking for
        print(f"\nField analysis:")
        print(f"- type: {response_data.get('type')}")
        print(f"- success: {response_data.get('success')}")
        print(f"- key: {response_data.get('key')}")
        print(f"- published: {response_data.get('published')}")
        
        # Test expectation analysis
        expected_type = 'keyboard_response'
        actual_type = response_data.get('type')
        expected_success = True
        actual_success = response_data.get('success', False)
        
        print(f"\nTest expectation analysis:")
        print(f"- Expected type '{expected_type}', got '{actual_type}': {'✓' if actual_type == expected_type else '✗'}")
        print(f"- Expected success True, got {actual_success}: {'✓' if actual_success else '✗'}")
        
        await websocket.close()
        
    except Exception as e:
        print(f"Error in specific test: {e}")


def main():
    """Main debug function"""
    print("WebSocket Response Debug Tool")
    print("This will connect to the live bocchi server and inspect responses")
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        # Run general debug
        loop.run_until_complete(debug_websocket_responses())
        
        # Run specific test
        loop.run_until_complete(test_specific_keyboard_message())
        
    except KeyboardInterrupt:
        print("\nDebug interrupted by user")
    except Exception as e:
        print(f"Debug error: {e}")
    finally:
        loop.close()


if __name__ == '__main__':
    main()