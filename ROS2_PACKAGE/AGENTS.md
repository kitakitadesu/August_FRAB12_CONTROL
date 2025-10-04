# AGENTS.md

## Docker Development Environment

### Quick Start
1. **Start Docker container**: `./docker.sh run`
2. **Build ROS2 workspace**: `./ros2-docker.sh build`
3. **Run a node**: `./ros2-docker.sh run bocchi publisher`

### Docker Management Commands
- `./docker.sh build` - Build the Docker image (default: ros-base)
- `./docker.sh run --desktop` - Run with ghcr.io/tiryoh/ros2-desktop-vnc:humble (pre-built VNC desktop image)
- `./docker.sh build --platform linux/amd64` - Build for specific platform (cross-platform support)
- `./docker.sh run` - Build and run new container (default: ros-base)
- `./docker.sh run --desktop --platform linux/amd64` - Run desktop image for specific platform
- `./docker.sh run --platform linux/amd64` - Run with specific platform
- `./docker.sh start` - Start existing container
- `./docker.sh stop` - Stop container
- `./docker.sh shell` - Open shell in container (manual access)
- `./docker.sh ssh` - SSH into container (root:password)
- `./docker.sh clean` - Remove container and image

### ROS2 Development Commands
Use `./ros2-docker.sh` for all ROS2 operations with automatic environment setup:

- `./ros2-docker.sh status` - Check container and ROS2 environment status
- `./ros2-docker.sh shell` - Open interactive shell with ROS2 environment
- `./ros2-docker.sh build` - Build the ROS2 workspace
- `./ros2-docker.sh setup` - Install dependencies
- `./ros2-docker.sh test` - Run tests
- `./ros2-docker.sh run <package> <node>` - Run a ROS2 node
- `./ros2-docker.sh launch <package> <launch_file>` - Launch a ROS2 launch file
- `./ros2-docker.sh monitor-nodes [args]` - Monitor ROS2 nodes
- `./ros2-docker.sh monitor-topics [args]` - Monitor ROS2 topics
- `./ros2-docker.sh exec "<command>"` - Execute arbitrary commands

### Examples
```bash
# Check status
./ros2-docker.sh status

# Build workspace
./ros2-docker.sh build

# Run publisher node
./ros2-docker.sh run bocchi publisher

# Run subscriber node (in another terminal)
./ros2-docker.sh run bocchi subscriber

# Execute custom ROS2 commands
./ros2-docker.sh exec "ros2 node list"
./ros2-docker.sh exec "ros2 topic list"

# Monitor ROS2 nodes and topics
./ros2-docker.sh monitor-nodes
./ros2-docker.sh monitor-nodes --continuous
./ros2-docker.sh monitor-topics
./ros2-docker.sh monitor-topics --report

# Open shell for interactive development
./ros2-docker.sh shell

# Run with desktop VNC image (includes GUI tools and VNC)
# Security options are automatically configured for Ubuntu Jammy compatibility
./docker.sh run --desktop

# Cross-platform examples (useful for Apple Silicon Macs)
./docker.sh build --platform linux/amd64
./docker.sh run --platform linux/amd64 --desktop
```

## Native Setup (Alternative)
If you prefer to run without Docker:
- Activate ROS2 environment: `source /opt/ros/humble/setup.bash`
- Activate workspace: `source /workdir/ros2_ws/install/setup.bash`
- Install dependencies: `./scripts/setup.sh` or `rosdep install --from-paths /workdir/ros2_ws/src/bocchi/ --ignore-src -r -y`
- Build workspace: `./scripts/build.sh` or `colcon build --symlink-install --packages-select bocchi`
- Run tests: `./scripts/test.sh`
- Run node: `ros2 run bocchi <node_name>`

## SSH Access
The Docker container runs Dropbear SSH server:
- **Host**: `localhost`
- **Port**: `2222`
- **Username**: `root`
- **Password**: `password`
- **Command**: `ssh root@localhost -p 2222`

## Container Details
- **Base Image**: `ros:humble-ros-base` (default, lightweight)
- **Alternative**: `ghcr.io/tiryoh/ros2-desktop-vnc:humble` (with `--desktop` flag, includes GUI packages and VNC)
- **Working Directory**: `/workdir` (mounted from host)
- **ROS2 Environment**: Automatically sourced
- **Privileges**: Full system access for hardware interaction

### Image Options
- **Default (ros-base)**: Minimal ROS2 installation, smaller image size, faster builds
- **Desktop (--desktop)**: Full ROS2 desktop with VNC support, includes GUI tools like RViz, Gazebo, rqt
  - **Note**: Requires `--security-opt seccomp=unconfined` flag (Ubuntu Jammy based image)
- **Platform (--platform)**: Cross-platform support (e.g., `linux/amd64` for Apple Silicon compatibility)

## Code style
- Use dependency injection patterns where possible
- Write tests for all features in /workdir/ros2_ws/src/bocchi/test
- Write convenient scripts in /workdir/ros2_ws/scripts
- Follow ROS2 best practices for node design and communication
- Document all public APIs and complex logic in code comments
- Use logging instead of print statements for runtime information
- Follow PEP 8 style guide for Python code
- Don't create summary files, keep documentation in inlined comments
- Commit changes with consistent style from old commit
- Commit individually focused changes

## Troubleshooting

### Container Issues
- If container won't start: `./docker.sh clean && ./docker.sh run`
- If SSH connection refused: Check container status with `./ros2-docker.sh status`
- If build fails: Try `./ros2-docker.sh setup` first to install dependencies

### Environment Issues
- If ROS2 commands not found: Use `./ros2-docker.sh` instead of direct commands
- If workspace not found: Ensure container is running and try `./ros2-docker.sh status`
- If packages not visible: Run `./ros2-docker.sh build` to rebuild workspace
