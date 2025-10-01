# AGENTS.md

## Setup commands
- Activate ROS2 environment: `source /opt/ros/humble/setup.bash`
- Activate workspace: `source /workdir/ros2_ws/install/setup.bash`
- Install dependencies: `rosdep install --from-paths /workdir/ros2_ws/src/bocchi/ --ignore-src -r -y`
- Build workspace: `colcon build --symlink-install --packages-select bocchi`
- Run node: `ros2 run bocchi <node_name>`

## Code style
- Use dependency injection patterns where possible
