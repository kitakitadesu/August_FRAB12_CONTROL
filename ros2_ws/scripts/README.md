# Bocchi Robot Controller - Scripts Directory

This directory contains all the convenience scripts for building, testing, launching, and managing the bocchi robot controller system.

## Overview

The scripts provide a comprehensive development and deployment workflow for the ROS2-based bocchi robot controller with WebSocket communication.

## Quick Start

```bash
# From the ROS2 workspace root (ros2_ws/)
cd ros2_ws

# First-time setup
./scripts/run.sh deps     # Install dependencies
./scripts/run.sh build    # Build the package

# Launch the system
./scripts/run.sh start    # Start the robot controller

# Development workflow
./scripts/run.sh dev-watch # Development server with auto-reload
```

## Script Files

### 🎯 Main Dispatcher

#### `run.sh` - Central Command Dispatcher
The main entry point for all bocchi operations. Provides a unified interface to all other scripts.

**Usage:**
```bash
./scripts/run.sh <command> [options]
```

**Key Commands:**
- `build` - Build the ROS2 package
- `test` - Run comprehensive test suite
- `start` - Launch the bocchi system
- `dev` - Development tools
- `monitor` - System monitoring
- `help` - Show all available commands

### 🏗️ Build & Setup

#### `build.sh` - ROS2 Package Builder
Builds the bocchi ROS2 package with proper environment setup.

**Features:**
- ✅ ROS2 environment validation
- ✅ Dependency checking with rosdep
- ✅ Release and Debug build modes
- ✅ Build verification
- ✅ Clean build option (`--clean`)

**Usage:**
```bash
./scripts/build.sh [--clean]
```

**Example:**
```bash
./scripts/build.sh --clean  # Clean build from scratch
```

### 🧪 Testing

#### `test.sh` - Comprehensive Test Runner
Runs the complete WebSocket test suite with detailed reporting.

**Features:**
- ✅ Multiple test categories (unit, integration, security, performance)
- ✅ ROS2-compatible test execution
- ✅ Coverage reporting (optional)
- ✅ Verbose and quiet modes
- ✅ Individual test category selection

**Usage:**
```bash
./scripts/test.sh [options]
```

**Options:**
- `--unit` / `-u` - Run unit tests only
- `--integration` / `-i` - Run integration tests only
- `--security` / `-s` - Run security tests only
- `--performance` / `-p` - Run performance tests only
- `--websocket` / `-w` - Run WebSocket-specific tests only
- `--verbose` / `-v` - Enable verbose output
- `--coverage` / `-c` - Generate test coverage report

**Examples:**
```bash
./scripts/test.sh                    # Run all tests
./scripts/test.sh --unit            # Unit tests only
./scripts/test.sh --coverage        # All tests with coverage
./scripts/test.sh --integration -v  # Integration tests, verbose
```

### 🚀 Launch & Execution

#### `launch.sh` - System Launcher
Starts the complete bocchi robot controller system with all services.

**Features:**
- ✅ Port availability checking
- ✅ Service health monitoring
- ✅ Automatic browser opening
- ✅ Demo mode with test commands
- ✅ Network information display
- ✅ Graceful shutdown handling

**Services Started:**
- ROS2 publisher node (robot control)
- WebSocket server (port 8765)
- REST API server (port 5000)
- Web interface (port 8000)

**Usage:**
```bash
./scripts/launch.sh [options]
```

**Options:**
- `--demo` / `-d` - Enable demo mode with test data
- `--verbose` / `-v` - Enable verbose logging
- `--headless` / `-h` - Run without opening web browser
- `--dev` - Enable development mode

**Examples:**
```bash
./scripts/launch.sh              # Standard launch
./scripts/launch.sh --demo       # Launch with demo commands
./scripts/launch.sh --headless   # No browser opening
./scripts/launch.sh --dev -v     # Development mode, verbose
```

### 🛠️ Development

#### `dev.sh` - Development Tools
Provides development workflow tools including building, testing, linting, and code formatting.

**Features:**
- ✅ Development builds (debug mode)
- ✅ File watching with auto-reload
- ✅ Code linting (flake8, ament tools)
- ✅ Code formatting (black, isort)
- ✅ Dependency management
- ✅ Debug session support
- ✅ Performance profiling

**Usage:**
```bash
./scripts/dev.sh <command>
```

**Commands:**
- `build` / `b` - Build in debug mode
- `test` / `t` - Run test suite with verbose output
- `lint` / `l` - Run code linting
- `format` / `f` - Format code
- `clean` / `c` - Clean build artifacts
- `deps` - Install development dependencies
- `watch` / `w` - Start development server with auto-reload
- `debug` / `d` - Start debug session
- `status` / `s` - Show development environment status

**Examples:**
```bash
./scripts/dev.sh build          # Debug build
./scripts/dev.sh watch          # Development server
./scripts/dev.sh format         # Format all code
./scripts/dev.sh lint           # Run linting
```

### 📊 Monitoring

#### `monitor.sh` - System Health Monitor
Comprehensive system monitoring with health checks and performance metrics.

**Features:**
- ✅ Real-time system resource monitoring
- ✅ ROS2 process and topic monitoring
- ✅ Service endpoint health checks
- ✅ WebSocket connection testing
- ✅ Performance metrics collection
- ✅ Health report generation
- ✅ Continuous and one-shot monitoring

**Usage:**
```bash
./scripts/monitor.sh [options]
```

**Options:**
- `--once` / `-o` - Run monitoring checks once and exit
- `--report` / `-r` - Generate health report and exit
- `--interval` / `-i` - Set monitoring interval in seconds

**Examples:**
```bash
./scripts/monitor.sh            # Continuous monitoring
./scripts/monitor.sh --once     # Single health check
./scripts/monitor.sh --report   # Generate health report
./scripts/monitor.sh -i 10      # Monitor every 10 seconds
```

**Monitoring Categories:**
- 🖥️ System resources (CPU, memory, disk)
- 🌐 Network services and ports
- 🤖 ROS2 processes and topics
- 🔗 Service endpoints (REST API, WebSocket)
- 📊 Performance metrics
- 📋 Log file analysis

## System Architecture

### Ports Used
- **5000** - REST API server
- **8000** - Web interface
- **8765** - WebSocket server

### ROS2 Topics
- `/cmd_vel` - Robot movement commands (geometry_msgs/Twist)
- `/servo_position` - Servo control commands (std_msgs/Int32)

### Key Components
- **Publisher Node** - Main ROS2 node for robot control
- **WebSocket Server** - Real-time bidirectional communication
- **REST API** - HTTP endpoints for web interface
- **Web Interface** - User control interface

## Development Workflow

### First-Time Setup
```bash
# From the ROS2 workspace root (ros2_ws/)
cd ros2_ws

# 1. Install dependencies
./scripts/run.sh deps

# 2. Build the package
./scripts/run.sh build

# 3. Run tests to verify setup
./scripts/run.sh test

# 4. Launch the system
./scripts/run.sh start
```

### Daily Development
```bash
# From the ROS2 workspace root (ros2_ws/)
cd ros2_ws

# Start development server with auto-reload
./scripts/run.sh dev-watch

# In another terminal - run tests
./scripts/run.sh test

# Format code before commits
./scripts/run.sh dev-format
```

### Testing Workflow
```bash
# From the ROS2 workspace root (ros2_ws/)
cd ros2_ws

# Run all tests
./scripts/run.sh test

# Run specific test categories
./scripts/run.sh test-unit
./scripts/run.sh test-integration
./scripts/run.sh test-security

# Generate coverage report
./scripts/test.sh --coverage
```

### Production Deployment
```bash
# From the ROS2 workspace root (ros2_ws/)
cd ros2_ws

# Build for production
./scripts/run.sh build

# Run full test suite
./scripts/run.sh test

# Launch system
./scripts/run.sh start

# Monitor system health
./scripts/run.sh monitor
```

## Troubleshooting

### Common Issues

#### Port Already in Use
```bash
# Check what's using the ports
./scripts/run.sh ports

# Stop all bocchi processes
./scripts/run.sh stop

# Or manually kill processes
sudo lsof -ti:5000 | xargs kill -9
```

#### Build Issues
```bash
# Clean build
./scripts/run.sh clean
./scripts/run.sh build

# Check dependencies
./scripts/run.sh deps
```

#### ROS2 Environment Issues
```bash
# Check environment status
./scripts/run.sh info

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### Test Failures
```bash
# Run tests with verbose output
./scripts/test.sh --verbose

# Run specific test category
./scripts/test.sh --unit

# Check system status
./scripts/run.sh status
```

### Log Locations
- **ROS2 Logs**: `~/.ros/log/`
- **Application Logs**: `bocchi.log`
- **Monitor Logs**: `monitor.log`
- **Test Coverage**: `htmlcov/`

## Script Dependencies

### System Requirements
- Ubuntu 22.04 (recommended)
- ROS2 Humble
- Python 3.8+
- Node.js (for some development tools)

### Python Dependencies
- `rclpy` - ROS2 Python client
- `websockets` - WebSocket server/client
- `flask` - Web framework
- `pytest` - Testing framework
- `black` - Code formatter (dev)
- `flake8` - Code linter (dev)

### System Dependencies
- `curl` - HTTP testing
- `netstat` - Network monitoring
- `lsof` - Process monitoring
- `inotify-tools` - File watching (dev)

## Environment Variables

### ROS2 Environment
- `ROS_DISTRO` - ROS2 distribution (should be "humble")
- `ROS_WORKSPACE` - Workspace path

### Development Settings
- `BOCCHI_DEBUG` - Enable debug mode
- `BOCCHI_LOG_LEVEL` - Set logging level
- `BOCCHI_DEV_MODE` - Enable development features

## Integration with IDE

### VS Code
```json
{
    "tasks": [
        {
            "label": "Build bocchi",
            "type": "shell",
            "command": "./scripts/run.sh build",
            "group": "build"
        },
        {
            "label": "Test bocchi",
            "type": "shell",
            "command": "./scripts/run.sh test",
            "group": "test"
        },
        {
            "label": "Launch bocchi",
            "type": "shell",
            "command": "./scripts/run.sh start",
            "group": "build"
        }
    ]
}
```

### Launch Configuration
```json
{
    "name": "Debug bocchi",
    "type": "python",
    "request": "launch",
    "module": "bocchi.publisher",
    "console": "integratedTerminal"
}
```

## Contributing

When adding new scripts:

1. **Follow naming convention**: `action.sh`
2. **Add to `run.sh`**: Include in main dispatcher
3. **Include help text**: `--help` option
4. **Error handling**: Use `set -e` and proper error messages
5. **Logging**: Use colored output functions
6. **Documentation**: Update this README

### Script Template
```bash
#!/bin/bash
# Script description
set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# Main logic here...
```

## License

This script collection is part of the bocchi robot controller project and follows the same license terms.

---

**For more information:**
- 📖 Main documentation: `../README.md`
- 🧪 Test documentation: `../test/README_TESTS.md`
- 🔧 Development guide: `DEVELOPMENT.md` (generated by `dev.sh docs`)