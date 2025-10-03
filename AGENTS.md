# AGENTS.md

## Setup commands
- Activate ROS2 environment: `source /opt/ros/humble/setup.bash`
- Activate workspace: `source /workdir/ros2_ws/install/setup.bash`
- Install dependencies: `./scripts/setup.sh` or `rosdep install --from-paths /workdir/ros2_ws/src/bocchi/ --ignore-src -r -y`
- Build workspace: `./scripts/build.sh` or `colcon build --symlink-install --packages-select bocchi`
- Run tests: `./scripts/test.sh`
- Run node: `ros2 run bocchi <node_name>`

## Code style
- Use dependency injection patterns where possible
- Write tests for all features in /workdir/ros2_ws/src/bocchi/test
- Write convenient scripts in /workdir/ros2_ws/scripts
- Follow ROS2 best practices for node design and communication
- Document all public APIs and complex logic in code comments
- Use logging instead of print statements for runtime information
- Follow PEP 8 style guide for Python code
- Don't create summary files, keep documentation in inlined comments
