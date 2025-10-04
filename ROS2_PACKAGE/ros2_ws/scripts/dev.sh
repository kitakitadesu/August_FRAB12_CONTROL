#!/bin/bash
# ROS2 Development Script for bocchi robot controller
# This script provides convenient development workflow commands

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_dev() {
    echo -e "${PURPLE}[DEV]${NC} $1"
}

print_cmd() {
    echo -e "${CYAN}[CMD]${NC} $1"
}

# Check if we're in the correct directory
if [ ! -f "src/bocchi/package.xml" ]; then
    print_error "Must be run from ROS2 workspace root (ros2_ws)"
    print_error "Current directory: $(pwd)"
    print_error "Expected to find: src/bocchi/package.xml"
    exit 1
fi

print_dev "🛠️  bocchi Robot Controller Development Tools"
print_dev "=============================================="

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
else
    print_error "ROS2 Humble not found"
    exit 1
fi

# Development functions
dev_build() {
    print_dev "Building package in development mode..."
    colcon build --packages-select bocchi --cmake-args -DCMAKE_BUILD_TYPE=Debug
    source install/setup.bash
    print_success "Development build completed"
}

dev_test() {
    print_dev "Running tests in watch mode..."
    ./scripts/test.sh --verbose
}

dev_lint() {
    print_dev "Running code linting..."
    
    # Python linting
    if command -v flake8 >/dev/null 2>&1; then
        print_status "Running flake8..."
        flake8 src/bocchi/bocchi/ --max-line-length=120 --ignore=E501,W503
    else
        print_warning "flake8 not found, skipping Python linting"
    fi
    
    # ROS2 linting
    if command -v ament_flake8 >/dev/null 2>&1; then
        print_status "Running ament_flake8..."
        ament_flake8 src/bocchi/bocchi/
    fi
    
    if command -v ament_pep257 >/dev/null 2>&1; then
        print_status "Running ament_pep257..."
        ament_pep257 src/bocchi/bocchi/
    fi
    
    print_success "Linting completed"
}

dev_format() {
    print_dev "Formatting code..."
    
    if command -v black >/dev/null 2>&1; then
        print_status "Running black formatter..."
        black src/bocchi/bocchi/ --line-length=120
    else
        print_warning "black not found, install with: pip install black"
    fi
    
    if command -v isort >/dev/null 2>&1; then
        print_status "Running isort..."
        isort src/bocchi/bocchi/
    else
        print_warning "isort not found, install with: pip install isort"
    fi
    
    print_success "Code formatting completed"
}

dev_clean() {
    print_dev "Cleaning development environment..."
    
    # Clean build artifacts
    rm -rf build/ install/ log/
    
    # Clean Python cache
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    find . -type f -name "*.pyc" -delete 2>/dev/null || true
    
    # Clean test artifacts
    rm -rf .pytest_cache/ htmlcov/ .coverage
    
    print_success "Environment cleaned"
}

dev_deps() {
    print_dev "Installing development dependencies..."
    
    # Python development packages
    pip3 install --user black isort flake8 pytest-cov pytest-asyncio pytest-mock
    
    # ROS2 dependencies
    if command -v rosdep >/dev/null 2>&1; then
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
    fi
    
    print_success "Development dependencies installed"
}

dev_watch() {
    print_dev "Starting development server with file watching..."
    
    # Kill any existing processes
    pkill -f "bocchi" 2>/dev/null || true
    
    # Build first
    dev_build
    
    print_status "Starting bocchi in development mode..."
    print_status "File changes will trigger auto-reload"
    print_status "Press Ctrl+C to stop"
    
    # Start with file watching using inotify
    if command -v inotifywait >/dev/null 2>&1; then
        ros2 run bocchi publisher &
        PUBLISHER_PID=$!
        
        while true; do
            inotifywait -r -e modify,create,delete src/bocchi/bocchi/ 2>/dev/null
            print_status "File change detected, restarting..."
            
            kill $PUBLISHER_PID 2>/dev/null || true
            sleep 1
            
            dev_build
            ros2 run bocchi publisher &
            PUBLISHER_PID=$!
        done
    else
        print_warning "inotifywait not found, running without file watching"
        print_status "Install with: sudo apt install inotify-tools"
        ros2 run bocchi publisher
    fi
}

dev_debug() {
    print_dev "Starting debug session..."
    
    # Check if gdb is available
    if command -v gdb >/dev/null 2>&1; then
        print_status "Starting with GDB debugger..."
        gdb --args python3 -m bocchi.publisher
    else
        print_status "Starting with Python debugger..."
        python3 -m pdb -m bocchi.publisher
    fi
}

dev_profile() {
    print_dev "Running performance profiling..."
    
    if command -v py-spy >/dev/null 2>&1; then
        print_status "Using py-spy for profiling..."
        py-spy record -o profile.svg -- python3 -m bocchi.publisher &
        sleep 30
        pkill py-spy
        print_success "Profile saved to profile.svg"
    else
        print_warning "py-spy not found, install with: pip install py-spy"
        print_status "Running basic profiling..."
        python3 -m cProfile -o profile.prof -m bocchi.publisher
    fi
}

dev_logs() {
    print_dev "Viewing development logs..."
    
    # ROS2 logs
    if [ -d "$HOME/.ros/log" ]; then
        print_status "ROS2 logs:"
        tail -f "$HOME/.ros/log/latest/rosout.log" 2>/dev/null || echo "No ROS2 logs found"
    fi
    
    # Application logs
    if [ -f "bocchi.log" ]; then
        print_status "Application logs:"
        tail -f bocchi.log
    else
        print_status "No application logs found"
    fi
}

dev_docs() {
    print_dev "Generating documentation..."
    
    if command -v sphinx-build >/dev/null 2>&1; then
        print_status "Building Sphinx documentation..."
        # Create docs directory if it doesn't exist
        mkdir -p docs
        sphinx-quickstart docs --quiet --project="Bocchi Robot Controller" \
                       --author="Developer" --release="1.0" --language="en"
        sphinx-build -b html docs docs/_build
        print_success "Documentation built in docs/_build/"
    else
        print_warning "Sphinx not found, generating simple docs..."
        
        # Generate simple markdown docs
        cat > DEVELOPMENT.md << EOF
# Bocchi Robot Controller - Development Guide

## Quick Start
\`\`\`bash
./scripts/dev.sh build    # Build in debug mode
./scripts/dev.sh test     # Run tests
./scripts/dev.sh watch    # Development server with auto-reload
\`\`\`

## Architecture
- **Publisher Node**: Main ROS2 node for robot control
- **WebSocket Server**: Real-time communication (port 8765)
- **REST API**: HTTP endpoints (port 5000)
- **Web Interface**: User interface (port 8000)

## Key Components
- \`src/bocchi/bocchi/publisher.py\` - Main application
- \`src/bocchi/test/\` - Test suite
- \`src/bocchi/templates/\` - Web templates
- \`src/bocchi/static/\` - Static assets

## Development Workflow
1. Make changes to source code
2. Run tests: \`./scripts/dev.sh test\`
3. Build: \`./scripts/dev.sh build\`
4. Launch: \`./scripts/launch.sh\`

EOF
        print_success "Development guide created: DEVELOPMENT.md"
    fi
}

dev_status() {
    print_dev "Development environment status:"
    echo ""
    
    # ROS2 environment
    if [ ! -z "$ROS_DISTRO" ]; then
        print_success "✅ ROS2 $ROS_DISTRO environment active"
    else
        print_error "❌ ROS2 environment not active"
    fi
    
    # Python environment
    print_status "🐍 Python: $(python3 --version)"
    
    # Required packages
    echo "📦 Dependencies:"
    for pkg in flask websockets rclpy; do
        if python3 -c "import $pkg" 2>/dev/null; then
            echo "  ✅ $pkg"
        else
            echo "  ❌ $pkg (missing)"
        fi
    done
    
    # Development tools
    echo "🛠️  Development tools:"
    for tool in black flake8 pytest; do
        if command -v $tool >/dev/null 2>&1; then
            echo "  ✅ $tool"
        else
            echo "  ❌ $tool (not installed)"
        fi
    done
    
    # Port availability
    echo "🌐 Ports:"
    for port in 5000 8000 8765; do
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            echo "  🔴 $port (in use)"
        else
            echo "  🟢 $port (available)"
        fi
    done
    
    # Build status
    if [ -f "install/bocchi/lib/bocchi/publisher" ]; then
        print_success "✅ Package built and installed"
    else
        print_warning "⚠️  Package not built"
    fi
}

# Parse command line arguments
case "${1:-help}" in
    build|b)
        dev_build
        ;;
    test|t)
        dev_test
        ;;
    lint|l)
        dev_lint
        ;;
    format|fmt|f)
        dev_format
        ;;
    clean|c)
        dev_clean
        ;;
    deps|dependencies)
        dev_deps
        ;;
    watch|w)
        dev_watch
        ;;
    debug|d)
        dev_debug
        ;;
    profile|p)
        dev_profile
        ;;
    logs)
        dev_logs
        ;;
    docs)
        dev_docs
        ;;
    status|s)
        dev_status
        ;;
    help|h|--help)
        echo "bocchi Robot Controller Development Tools"
        echo ""
        echo "Usage: $0 <command>"
        echo ""
        echo "Commands:"
        echo "  build, b          Build package in debug mode"
        echo "  test, t           Run test suite with verbose output"
        echo "  lint, l           Run code linting (flake8, ament)"
        echo "  format, fmt, f    Format code (black, isort)"
        echo "  clean, c          Clean build artifacts and cache"
        echo "  deps              Install development dependencies"
        echo "  watch, w          Start development server with auto-reload"
        echo "  debug, d          Start debug session"
        echo "  profile, p        Run performance profiling"
        echo "  logs              View development logs"
        echo "  docs              Generate documentation"
        echo "  status, s         Show development environment status"
        echo "  help, h           Show this help message"
        echo ""
        echo "Examples:"
        echo "  $0 build         # Build in debug mode"
        echo "  $0 test          # Run all tests"
        echo "  $0 watch         # Start with auto-reload"
        echo "  $0 format        # Format all code"
        echo ""
        echo "Quick development workflow:"
        echo "  1. $0 deps       # Install dependencies (first time)"
        echo "  2. $0 build      # Build the package"
        echo "  3. $0 test       # Run tests"
        echo "  4. $0 watch      # Start development server"
        ;;
    *)
        print_error "Unknown command: $1"
        print_status "Use '$0 help' for usage information"
        exit 1
        ;;
esac