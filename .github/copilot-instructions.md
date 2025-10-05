# August FRAB12 Control - AI Agent Instructions

## Project Architecture

This is a dual-layer robotics control system:

**HIGH-LEVEL (ROS2_PACKAGE/)**: ROS2 Python package `bocchi` providing web-based keyboard control
- **Publisher Node**: Flask web server + WebSocket server for real-time keyboard input
- **Subscriber Node**: Processes movement commands and servo controls
- **Cross-platform**: Runs in Docker with VNC desktop support

**LOW-LEVEL (LOW_LEVEL/)**: PlatformIO C++ firmware for Raspberry Pi Pico
- **micro-ROS**: Bridges Arduino framework with ROS2 ecosystem
- **Hardware Control**: 4x DC motors with encoders, servo motor, LED status
- **Real-time**: Subscribes to `/cmd_vel` and `/servo_position` topics

## Development Workflow

### Docker-First Development
Always use Docker scripts - never run ROS2 commands directly:

```bash
# Start development environment
./docker.sh run                    # or ./docker.sh run --desktop for GUI tools
./ros2-docker.sh build            # Build ROS2 workspace
./ros2-docker.sh run bocchi publisher    # Run nodes

# Development cycle
./ros2-docker.sh shell             # Interactive development
./ros2-docker.sh test              # Run all tests
./ros2-docker.sh monitor-topics    # Debug communication
```

### PlatformIO Workflow
LOW_LEVEL/ uses PlatformIO with custom Raspberry Pi Pico platform:
- Board: `pico` with `earlephilhower` core
- Framework: Arduino with micro-ROS integration
- Libraries: Custom motor/encoder/servo classes in `lib/`

## Code Patterns & Conventions

### ROS2 Python Patterns
- **Dependency Injection**: Pass hardware interfaces to node constructors
- **WebSocket + Flask**: Real-time keyboard control via `publisher.py` (2100+ lines)
- **Async Communication**: WebSocket for low-latency, REST API fallback
- **Testing**: Comprehensive WebSocket test suite in `test/test_websocket*.py`

### C++ Embedded Patterns  
- **RAII Hardware**: DCMotor, ServoMotor, IncrementalMotorEncoder classes
- **Node Composition**: Separate publisher/subscriber nodes in `src/nodes/`
- **Error Handling**: `RCCHECK`/`RCSOFTCHECK` macros for micro-ROS operations

### File Organization
```
ROS2_PACKAGE/bocchi/
├── publisher.py          # Main web server + ROS2 publisher (Flask + WebSocket)
├── subscriber.py         # Command processor
├── templates/           # Mako templates for web UI
└── static/             # CSS/JS for keyboard interface

LOW_LEVEL/src/
├── main.cpp            # micro-ROS setup + hardware initialization  
├── nodes/              # ROS2 node implementations
└── lib/                # Custom hardware abstraction classes
```

## Key Integration Points

### Topic Communication
- `/cmd_vel` (geometry_msgs/Twist): WASD keyboard → motor control (20Hz)
- `/servo_position` (std_msgs/Int32): F key → servo toggle (0°/180°)
- `/encoder_data`: Feedback from hardware to ROS2 system

### WebSocket Protocol
Real-time keyboard events via `ws://localhost:8765`:
```json
{"type": "key_down", "key": "W", "timestamp": 1640995200000}
```

### Docker Network Bridge
- SSH: `ssh root@localhost -p 2222` (password: `password`)
- Web UI: `http://localhost:5000`
- WebSocket: `ws://localhost:8765`

## Testing Strategy

Run comprehensive test suite covering WebSocket functionality, ROS2 integration, and security:
```bash
./ros2-docker.sh test                    # All tests
python3 test_websocket_comprehensive.py # WebSocket-specific
```

Tests validate: real-time communication, fallback mechanisms, concurrent connections, and message ordering.

## Debugging Commands

```bash
# Container status and health
./ros2-docker.sh status

# ROS2 system inspection  
./ros2-docker.sh exec "ros2 node list"
./ros2-docker.sh exec "ros2 topic list"
./ros2-docker.sh exec "ros2 topic echo /cmd_vel"

# WebSocket debugging
python3 test/debug_websocket_responses.py
```

## Cross-Platform Notes
- Use `--platform linux/amd64` flag for Apple Silicon compatibility
- Desktop image requires `--security-opt seccomp=unconfined` for Ubuntu Jammy
- WebSocket server binds to `0.0.0.0` for Docker networking

When editing this codebase:
1. Use `./ros2-docker.sh` wrapper for all ROS2 operations
2. Test both WebSocket and REST API fallback paths  
3. Validate hardware integration with micro-ROS subscriber nodes
4. Follow existing patterns: dependency injection, comprehensive testing, Docker-first development