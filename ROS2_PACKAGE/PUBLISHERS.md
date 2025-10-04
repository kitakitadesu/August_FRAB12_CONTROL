# ROS2 Publishers

## MinimalPublisher

This node publishes to two topics for robot control:

### Movement Control

**Topic:** `/cmd_vel`
**Datatype:** `geometry_msgs/Twist`
**Publishing Rate:** 20 Hz (0.05s timer)
**Key Mappings:**
- `W`: Move Forward (`linear.x = 0.5`)
- `S`: Move Backward (`linear.x = -0.5`)
- `A`: Turn Left (`angular.z = 0.5`)
- `D`: Turn Right (`angular.z = -0.5`)
- Other keys: Stop (`linear.x = 0.0, angular.z = 0.0`)

**Data Explanation:**
- `linear.x`: Forward/backward velocity (0.5 for forward, -0.5 for backward, 0.0 for stop)
- `angular.z`: Rotational velocity (0.5 for left turn, -0.5 for right turn, 0.0 for straight)
- Other fields (linear.y, linear.z, angular.x, angular.y) are always 0.0

### Servo Control

**Topic:** `/servo_position`
**Datatype:** `std_msgs/Int32`
**Publishing Rate:** On key press event
**Key Mappings:**
- `F`: Toggle servo position between 0° and 180°

**Data Explanation:**
- `data`: Servo angle in degrees (0 or 180)
- Toggle behavior: 0° → 180° → 0° → 180° ...

## Usage

1. Start the publisher: `ros2 run bocchi publisher`
2. Open web interface: `http://localhost:5000`
3. Press WASD keys for movement, F key for servo control
4. Monitor topics:
   - `ros2 topic echo /cmd_vel`
   - `ros2 topic echo /servo_position`

## WebSocket Keyboard Control

The system now supports real-time keyboard control via WebSocket for improved responsiveness:

### Features
- **Real-time control**: Lower latency than REST API
- **Automatic fallback**: Falls back to REST API if WebSocket unavailable  
- **Connection status**: Visual indicator shows current input mode
- **Simultaneous keys**: Support for multiple key combinations
- **Debouncing**: Prevents duplicate commands from rapid key presses

### WebSocket Endpoints
- **WebSocket Server**: `ws://localhost:8765` (or `ws://<your-ip>:8765`)
- **Message Types**:
  - `key_down`: Key press event
  - `key_up`: Key release event
  - `keyboard`: Legacy key event with held state

### Message Format
```json
{
  "type": "key_down",
  "key": "W",
  "key_code": 87,
  "timestamp": 1640995200000
}
```

### Testing WebSocket Control
Use the included test script:
```bash
# Run automated tests
python3 test_websocket_keyboard.py

# Interactive mode
python3 test_websocket_keyboard.py --interactive
```