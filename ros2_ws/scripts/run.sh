#!/bin/bash
# ROS2 bocchi Robot Controller - Main Script Dispatcher
# This is the central entry point for all bocchi operations

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
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

print_header() {
    echo -e "${WHITE}$1${NC}"
}

print_cmd() {
    echo -e "${CYAN}[CMD]${NC} $1"
}

# ASCII Art Banner
show_banner() {
    echo -e "${PURPLE}"
    cat << "EOF"
    ____                  __    _ 
   / __ )____  __________/ /_  (_)
  / __  / __ \/ ___/ ___/ __ \/ / 
 / /_/ / /_/ / /__/ /__/ / / / /  
/_____/\____/\___/\___/_/ /_/_/   
                                 
🤖 Robot Controller System
EOF
    echo -e "${NC}"
}

# Check if we're in the correct directory
check_workspace() {
    if [ ! -f "src/bocchi/package.xml" ]; then
        print_error "Must be run from ROS2 workspace root (ros2_ws)"
        print_error "Current directory: $(pwd)"
        print_error "Expected to find: src/bocchi/package.xml"
        return 1
    fi
    return 0
}

# Check if scripts exist and are executable
check_scripts() {
    local scripts=("build.sh" "test.sh" "launch.sh" "dev.sh" "monitor.sh")
    local missing_scripts=()
    
    for script in "${scripts[@]}"; do
        if [ ! -f "scripts/$script" ]; then
            missing_scripts+=("$script")
        elif [ ! -x "scripts/$script" ]; then
            chmod +x "scripts/$script"
        fi
    done
    
    if [ ${#missing_scripts[@]} -ne 0 ]; then
        print_error "Missing scripts: ${missing_scripts[*]}"
        return 1
    fi
    return 0
}

# Display help information
show_help() {
    show_banner
    echo ""
    print_header "BOCCHI Robot Controller - Command Dispatcher"
    print_header "============================================="
    echo ""
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "📦 BUILD & SETUP:"
    echo "  build              Build the ROS2 package"
    echo "  clean              Clean build artifacts"
    echo "  deps               Install dependencies"
    echo ""
    echo "🧪 TESTING:"
    echo "  test               Run all tests"
    echo "  test-unit          Run unit tests only"
    echo "  test-integration   Run integration tests only"
    echo "  test-security      Run security tests only"
    echo "  test-websocket     Run WebSocket tests only"
    echo ""
    echo "🚀 EXECUTION:"
    echo "  start              Start the bocchi system"
    echo "  launch             Launch with full interface"
    echo "  demo               Start in demo mode"
    echo "  stop               Stop all bocchi processes"
    echo ""
    echo "🛠️  DEVELOPMENT:"
    echo "  dev                Development tools menu"
    echo "  dev-build          Build in debug mode"
    echo "  dev-watch          Development server with auto-reload"
    echo "  dev-test           Run tests in development mode"
    echo "  dev-lint           Run code linting"
    echo "  dev-format         Format source code"
    echo ""
    echo "📊 MONITORING:"
    echo "  monitor            Start continuous system monitoring"
    echo "  status             Show current system status"
    echo "  health             Generate health report"
    echo "  logs               View system logs"
    echo ""
    echo "ℹ️  INFORMATION:"
    echo "  info               Show system information"
    echo "  ports              Show port usage"
    echo "  topics             List ROS2 topics"
    echo "  version            Show version information"
    echo ""
    echo "Examples:"
    echo "  $0 build           # Build the package"
    echo "  $0 test            # Run all tests"
    echo "  $0 start           # Start the system"
    echo "  $0 dev-watch       # Start development server"
    echo "  $0 monitor         # Monitor system health"
    echo ""
    echo "🔧 Environment Setup:"
    echo "  • First time: $0 deps && $0 build"
    echo "  • Development: $0 dev-build && $0 dev-watch"
    echo "  • Production: $0 build && $0 start"
    echo "  • Testing: $0 test"
}

# Show system information
show_info() {
    print_header "🤖 BOCCHI SYSTEM INFORMATION"
    print_header "============================"
    echo ""
    
    # ROS2 Environment
    if [ ! -z "$ROS_DISTRO" ]; then
        print_success "ROS2 Distribution: $ROS_DISTRO"
    else
        print_warning "ROS2 Environment: Not active"
    fi
    
    # System info
    echo "🖥️  System:"
    echo "  OS: $(uname -o)"
    echo "  Kernel: $(uname -r)"
    echo "  Architecture: $(uname -m)"
    echo "  Hostname: $(hostname)"
    echo ""
    
    # Python info
    echo "🐍 Python:"
    echo "  Version: $(python3 --version 2>/dev/null || echo "Not found")"
    echo "  Location: $(which python3 2>/dev/null || echo "Not found")"
    echo ""
    
    # Package info
    echo "📦 Package Status:"
    if [ -f "install/bocchi/lib/bocchi/publisher" ]; then
        print_success "  Built: Yes"
        echo "  Install Path: $(pwd)/install/bocchi"
    else
        print_warning "  Built: No"
    fi
    
    # Workspace info
    echo ""
    echo "📁 Workspace:"
    echo "  Root: $(pwd)"
    echo "  Source: $(pwd)/src/bocchi"
    echo "  Build: $(pwd)/build/bocchi"
    echo "  Install: $(pwd)/install/bocchi"
    
    # Dependencies
    echo ""
    echo "🔗 Dependencies:"
    local deps=("flask" "websockets" "rclpy" "pytest")
    for dep in "${deps[@]}"; do
        if python3 -c "import $dep" 2>/dev/null; then
            echo "  ✅ $dep"
        else
            echo "  ❌ $dep (missing)"
        fi
    done
}

# Show port information
show_ports() {
    print_header "🌐 PORT INFORMATION"
    print_header "==================="
    echo ""
    
    local ports=(5000 8000 8765)
    local services=("REST API" "Web Interface" "WebSocket Server")
    
    for i in "${!ports[@]}"; do
        local port=${ports[$i]}
        local service=${services[$i]}
        
        echo "Port $port ($service):"
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            local pid=$(lsof -ti:$port 2>/dev/null | head -1)
            local process=$(ps -p $pid -o comm= 2>/dev/null || echo "unknown")
            print_success "  Status: LISTENING (PID: $pid, Process: $process)"
        else
            print_warning "  Status: NOT LISTENING"
        fi
        echo ""
    done
}

# Show ROS2 topics
show_topics() {
    print_header "📡 ROS2 TOPICS"
    print_header "=============="
    echo ""
    
    # Source ROS2 environment
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
    fi
    
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash >/dev/null 2>&1
    fi
    
    if command -v ros2 >/dev/null 2>&1; then
        print_status "Available topics:"
        timeout 5 ros2 topic list 2>/dev/null || print_warning "No topics found or ROS2 not running"
        
        echo ""
        print_status "Key bocchi topics:"
        local key_topics=("/cmd_vel" "/servo_position")
        for topic in "${key_topics[@]}"; do
            if timeout 2 ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
                print_success "  $topic: EXISTS"
            else
                print_warning "  $topic: NOT FOUND"
            fi
        done
    else
        print_error "ROS2 not available"
    fi
}

# Show version information
show_version() {
    print_header "📋 VERSION INFORMATION"
    print_header "======================"
    echo ""
    
    # Package version from package.xml
    if [ -f "src/bocchi/package.xml" ]; then
        local pkg_version=$(grep -o '<version>[^<]*' src/bocchi/package.xml | cut -d'>' -f2)
        echo "Package Version: ${pkg_version:-Unknown}"
    fi
    
    # Git information if available
    if [ -d ".git" ]; then
        echo "Git Branch: $(git branch --show-current 2>/dev/null || echo "Unknown")"
        echo "Git Commit: $(git rev-parse --short HEAD 2>/dev/null || echo "Unknown")"
        echo "Git Status: $(git status --porcelain 2>/dev/null | wc -l) files changed"
    fi
    
    # Build timestamp
    if [ -f "install/bocchi/lib/bocchi/publisher" ]; then
        echo "Build Date: $(stat -c %y install/bocchi/lib/bocchi/publisher 2>/dev/null | cut -d' ' -f1-2)"
    fi
    
    echo ""
    echo "Component Versions:"
    echo "  ROS2: ${ROS_DISTRO:-Not Set}"
    echo "  Python: $(python3 --version 2>/dev/null | cut -d' ' -f2 || echo "Unknown")"
    echo "  OS: $(uname -o) $(uname -r)"
}

# Stop all bocchi processes
stop_system() {
    print_status "Stopping bocchi system..."
    
    # Kill bocchi processes
    if pgrep -f "bocchi" >/dev/null; then
        pkill -f "bocchi"
        print_success "Stopped bocchi processes"
    fi
    
    # Kill python publisher processes
    if pgrep -f "python.*publisher" >/dev/null; then
        pkill -f "python.*publisher"
        print_success "Stopped publisher processes"
    fi
    
    # Free ports if needed
    local ports=(5000 8000 8765)
    for port in "${ports[@]}"; do
        local pids=$(lsof -ti:$port 2>/dev/null || true)
        if [ ! -z "$pids" ]; then
            echo $pids | xargs kill -9 2>/dev/null || true
            print_status "Freed port $port"
        fi
    done
    
    print_success "System stopped"
}

# Main command dispatcher
main() {
    # Check workspace first
    if ! check_workspace; then
        exit 1
    fi
    
    # Make scripts executable
    if ! check_scripts; then
        print_warning "Some scripts are missing or not executable"
    fi
    
    # Parse command
    case "${1:-help}" in
        # Build & Setup
        build)
            print_cmd "Running build script..."
            exec scripts/build.sh "${@:2}"
            ;;
        clean)
            print_cmd "Cleaning build artifacts..."
            rm -rf build/ install/ log/
            print_success "Build artifacts cleaned"
            ;;
        deps)
            print_cmd "Installing dependencies..."
            exec scripts/dev.sh deps
            ;;
            
        # Testing
        test)
            print_cmd "Running all tests..."
            exec scripts/test.sh "${@:2}"
            ;;
        test-unit)
            print_cmd "Running unit tests..."
            exec scripts/test.sh --unit "${@:2}"
            ;;
        test-integration)
            print_cmd "Running integration tests..."
            exec scripts/test.sh --integration "${@:2}"
            ;;
        test-security)
            print_cmd "Running security tests..."
            exec scripts/test.sh --security "${@:2}"
            ;;
        test-websocket)
            print_cmd "Running WebSocket tests..."
            exec scripts/test.sh --websocket "${@:2}"
            ;;
            
        # Execution
        start|launch)
            print_cmd "Starting bocchi system..."
            exec scripts/launch.sh "${@:2}"
            ;;
        demo)
            print_cmd "Starting demo mode..."
            exec scripts/launch.sh --demo "${@:2}"
            ;;
        stop)
            stop_system
            ;;
            
        # Development
        dev)
            print_cmd "Opening development tools..."
            exec scripts/dev.sh "${@:2}"
            ;;
        dev-build)
            print_cmd "Development build..."
            exec scripts/dev.sh build
            ;;
        dev-watch)
            print_cmd "Starting development server..."
            exec scripts/dev.sh watch
            ;;
        dev-test)
            print_cmd "Development testing..."
            exec scripts/dev.sh test
            ;;
        dev-lint)
            print_cmd "Running linting..."
            exec scripts/dev.sh lint
            ;;
        dev-format)
            print_cmd "Formatting code..."
            exec scripts/dev.sh format
            ;;
            
        # Monitoring
        monitor)
            print_cmd "Starting system monitor..."
            exec scripts/monitor.sh "${@:2}"
            ;;
        status)
            print_cmd "Checking system status..."
            exec scripts/monitor.sh --once
            ;;
        health)
            print_cmd "Generating health report..."
            exec scripts/monitor.sh --report
            ;;
        logs)
            print_cmd "Viewing logs..."
            exec scripts/monitor.sh logs
            ;;
            
        # Information
        info)
            show_info
            ;;
        ports)
            show_ports
            ;;
        topics)
            show_topics
            ;;
        version)
            show_version
            ;;
            
        # Help
        help|--help|-h)
            show_help
            ;;
            
        # Unknown command
        *)
            show_banner
            print_error "Unknown command: $1"
            echo ""
            print_status "Available commands:"
            echo "  build, test, start, dev, monitor, info, help"
            echo ""
            print_status "Use '$0 help' for detailed usage information"
            exit 1
            ;;
    esac
}

# Execute main function with all arguments
main "$@"