#!/bin/bash
# ROS2 Launch Script for bocchi robot controller
# This script launches the complete bocchi system with WebSocket server

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

print_launch() {
    echo -e "${PURPLE}[LAUNCH]${NC} $1"
}

print_system() {
    echo -e "${CYAN}[SYSTEM]${NC} $1"
}

# Cleanup function
cleanup() {
    print_status "Shutting down bocchi system..."
    
    # Kill background processes
    if [ ! -z "$PUBLISHER_PID" ]; then
        kill $PUBLISHER_PID 2>/dev/null || true
        print_status "Publisher process stopped"
    fi
    
    if [ ! -z "$WEB_PID" ]; then
        kill $WEB_PID 2>/dev/null || true
        print_status "Web server stopped"
    fi
    
    # Kill any remaining processes
    pkill -f "bocchi" 2>/dev/null || true
    pkill -f "python.*publisher" 2>/dev/null || true
    
    print_success "System shutdown complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Check if we're in the correct directory
if [ ! -f "src/bocchi/package.xml" ]; then
    print_error "Must be run from ROS2 workspace root (ros2_ws)"
    print_error "Current directory: $(pwd)"
    print_error "Expected to find: src/bocchi/package.xml"
    exit 1
fi

print_launch "🚀 Starting bocchi robot controller system..."

# Source ROS2 environment
print_status "Sourcing ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    print_success "ROS2 Humble environment sourced"
else
    print_error "ROS2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Source built workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    print_success "Built workspace sourced"
else
    print_warning "Built workspace not found, building first..."
    ./scripts/build.sh
    source install/setup.bash
fi

# Check for required ports
check_port() {
    local port=$1
    local service=$2
    
    if netstat -tuln 2>/dev/null | grep -q ":$port "; then
        print_warning "Port $port is already in use (required for $service)"
        print_status "Attempting to free port $port..."
        
        # Try to kill processes using the port
        local pids=$(lsof -ti:$port 2>/dev/null || true)
        if [ ! -z "$pids" ]; then
            echo $pids | xargs kill -9 2>/dev/null || true
            sleep 2
            
            if netstat -tuln 2>/dev/null | grep -q ":$port "; then
                print_error "Could not free port $port"
                return 1
            else
                print_success "Port $port freed successfully"
            fi
        fi
    fi
    return 0
}

print_status "Checking required ports..."
check_port 5000 "REST API"
check_port 8000 "Web Interface"
check_port 8765 "WebSocket Server"

# Parse command line arguments
DEMO_MODE=false
VERBOSE=false
HEADLESS=false
DEV_MODE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --demo|-d)
            DEMO_MODE=true
            shift
            ;;
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        --headless|-h)
            HEADLESS=true
            shift
            ;;
        --dev)
            DEV_MODE=true
            shift
            ;;
        --help)
            echo "bocchi Robot Controller Launch Script"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -d, --demo      Enable demo mode with test data"
            echo "  -v, --verbose   Enable verbose logging"
            echo "  -h, --headless  Run without opening web browser"
            echo "  --dev           Enable development mode"
            echo "  --help          Show this help message"
            echo ""
            echo "The system will start:"
            echo "  • ROS2 publisher node (robot control)"
            echo "  • WebSocket server (port 8765)"
            echo "  • REST API server (port 5000)"
            echo "  • Web interface (port 8000)"
            echo ""
            echo "Press Ctrl+C to stop all services"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            print_status "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Display system information
print_system "================================================================="
print_system "🤖 BOCCHI ROBOT CONTROLLER SYSTEM"
print_system "================================================================="
echo "  🏗️  Build Type: $([ "$DEV_MODE" = true ] && echo "Development" || echo "Production")"
echo "  📍 Workspace: $(pwd)"
echo "  🌐 Web Interface: http://localhost:8000"
echo "  📡 REST API: http://localhost:5000"
echo "  🔗 WebSocket: ws://localhost:8765"
echo "  📊 Demo Mode: $([ "$DEMO_MODE" = true ] && echo "Enabled" || echo "Disabled")"
echo "  🔍 Verbose: $([ "$VERBOSE" = true ] && echo "Enabled" || echo "Disabled")"
print_system "================================================================="

# Start ROS2 publisher node
print_launch "Starting ROS2 publisher node..."
if [ "$VERBOSE" = true ]; then
    ros2 run bocchi publisher &
else
    ros2 run bocchi publisher > /dev/null 2>&1 &
fi
PUBLISHER_PID=$!

# Wait for publisher to start
sleep 3

# Check if publisher is running
if ! kill -0 $PUBLISHER_PID 2>/dev/null; then
    print_error "Failed to start publisher node"
    exit 1
fi

print_success "✅ Publisher node started (PID: $PUBLISHER_PID)"

# Wait for services to be available
print_status "Waiting for services to become available..."

# Check WebSocket server
for i in {1..10}; do
    if curl -s http://localhost:5000/api/websocket-info >/dev/null 2>&1; then
        print_success "✅ REST API server is ready"
        break
    fi
    if [ $i -eq 10 ]; then
        print_error "REST API server failed to start"
        cleanup
        exit 1
    fi
    sleep 1
done

# Check WebSocket server
for i in {1..10}; do
    if nc -z localhost 8765 2>/dev/null; then
        print_success "✅ WebSocket server is ready"
        break
    fi
    if [ $i -eq 10 ]; then
        print_error "WebSocket server failed to start"
        cleanup
        exit 1
    fi
    sleep 1
done

# Check web interface
for i in {1..10}; do
    if curl -s http://localhost:8000 >/dev/null 2>&1; then
        print_success "✅ Web interface is ready"
        break
    fi
    if [ $i -eq 10 ]; then
        print_warning "Web interface may not be fully ready"
        break
    fi
    sleep 1
done

# Display network information
print_status "📡 Network Information:"
if command -v ip >/dev/null 2>&1; then
    LOCAL_IPS=$(ip route get 8.8.8.8 2>/dev/null | awk '{print $7; exit}' 2>/dev/null || echo "localhost")
    if [ "$LOCAL_IPS" != "localhost" ]; then
        echo "  🌍 External Access:"
        echo "    • Web Interface: http://$LOCAL_IPS:8000"
        echo "    • REST API: http://$LOCAL_IPS:5000"
        echo "    • WebSocket: ws://$LOCAL_IPS:8765"
    fi
fi

echo "  🏠 Local Access:"
echo "    • Web Interface: http://localhost:8000"
echo "    • REST API: http://localhost:5000"
echo "    • WebSocket: ws://localhost:8765"

# Start web browser if not headless
if [ "$HEADLESS" = false ]; then
    print_status "Opening web interface in browser..."
    
    # Try different browsers
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open http://localhost:8000 >/dev/null 2>&1 &
    elif command -v open >/dev/null 2>&1; then
        open http://localhost:8000 >/dev/null 2>&1 &
    elif command -v firefox >/dev/null 2>&1; then
        firefox http://localhost:8000 >/dev/null 2>&1 &
    elif command -v chromium-browser >/dev/null 2>&1; then
        chromium-browser http://localhost:8000 >/dev/null 2>&1 &
    else
        print_warning "No web browser found. Please open http://localhost:8000 manually"
    fi
fi

# System status monitoring function
monitor_system() {
    while true; do
        sleep 10
        
        # Check if publisher is still running
        if ! kill -0 $PUBLISHER_PID 2>/dev/null; then
            print_error "Publisher process died unexpectedly"
            cleanup
            exit 1
        fi
        
        # Check if services are still responsive
        if ! curl -s http://localhost:5000/api/status >/dev/null 2>&1; then
            print_warning "REST API not responding"
        fi
        
        if ! nc -z localhost 8765 2>/dev/null; then
            print_warning "WebSocket server not responding"
        fi
    done
}

# Display system ready message
print_success "================================================================="
print_success "🎉 BOCCHI SYSTEM IS READY!"
print_success "================================================================="
echo ""
echo "  🎮 Control Methods:"
echo "    • Web Interface: Open http://localhost:8000 in your browser"
echo "    • WASD Keys: W=Forward, S=Backward, A=Left, D=Right"
echo "    • F Key: Toggle servo position (0° ↔ 180°)"
echo ""
echo "  📊 System Status:"
echo "    • ROS2 Publisher: ✅ Running (PID: $PUBLISHER_PID)"
echo "    • WebSocket Server: ✅ Active on port 8765"
echo "    • REST API: ✅ Active on port 5000"
echo "    • Web Interface: ✅ Active on port 8000"
echo ""
echo "  🔧 Available Commands:"
echo "    • Check status: curl http://localhost:5000/api/status"
echo "    • WebSocket info: curl http://localhost:5000/api/websocket-info"
echo "    • Send test key: curl -X POST http://localhost:5000/api/key -d '{\"key\":\"w\"}' -H 'Content-Type: application/json'"
echo ""
echo "  🛑 To stop the system: Press Ctrl+C"
echo ""

# Start system monitoring in the background if verbose mode
if [ "$VERBOSE" = true ]; then
    print_status "Starting system monitoring..."
    monitor_system &
    MONITOR_PID=$!
fi

# Demo mode
if [ "$DEMO_MODE" = true ]; then
    print_launch "🎯 Demo Mode: Sending test commands..."
    sleep 2
    
    # Send demo commands
    curl -s -X POST http://localhost:5000/api/key \
         -H "Content-Type: application/json" \
         -d '{"key": "w", "key_code": 87}' >/dev/null
    print_status "Demo: Sent forward command"
    
    sleep 2
    curl -s -X POST http://localhost:5000/api/key/up \
         -H "Content-Type: application/json" \
         -d '{"key": "w", "key_code": 87}' >/dev/null
    print_status "Demo: Sent stop command"
    
    sleep 1
    curl -s -X POST http://localhost:5000/api/key \
         -H "Content-Type: application/json" \
         -d '{"key": "f", "key_code": 70}' >/dev/null
    print_status "Demo: Sent servo toggle command"
    
    print_success "Demo sequence completed!"
fi

print_status "System is running. Press Ctrl+C to stop..."

# Wait for user interrupt
wait $PUBLISHER_PID