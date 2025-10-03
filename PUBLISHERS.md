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
2. Open web interface: `http://localhost:8000`
3. Press WASD keys for movement, F key for servo control
4. Monitor topics:
   - `ros2 topic echo /cmd_vel`
   - `ros2 topic echo /servo_position`