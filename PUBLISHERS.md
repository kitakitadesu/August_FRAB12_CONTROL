# ROS2 Publishers

## MinimalPublisher

**Topic:** `/cmd_vel`
**Datatype:** `geometry_msgs/Twist`
**Data Explanation:**
- `linear.x`: Forward/backward velocity (0.5 for forward, -0.5 for backward, 0.0 for stop or other keys)
- `angular.z`: Rotational velocity (0.5 for left turn, -0.5 for right turn, 0.0 for straight or other keys)
- Other fields (linear.y, linear.z, angular.x, angular.y) are always 0.0