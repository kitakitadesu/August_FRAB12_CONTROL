#!/bin/bash
# ROS2 System Monitoring Script for bocchi robot controller
# This script monitors system health, performance, and service status

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
    echo -e "${GREEN}[OK]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_monitor() {
    echo -e "${PURPLE}[MONITOR]${NC} $1"
}

print_metric() {
    echo -e "${CYAN}[METRIC]${NC} $1"
}

print_header() {
    echo -e "${WHITE}$1${NC}"
}

# Configuration
MONITOR_INTERVAL=5
LOG_FILE="monitor.log"
ALERT_THRESHOLD_CPU=80
ALERT_THRESHOLD_MEMORY=85
ALERT_THRESHOLD_DISK=90

# Check if we're in the correct directory
if [ ! -f "src/bocchi/package.xml" ]; then
    print_error "Must be run from ROS2 workspace root (ros2_ws)"
    print_error "Current directory: $(pwd)"
    print_error "Expected to find: src/bocchi/package.xml"
    exit 1
fi

# Initialize monitoring
print_monitor "🔍 Starting bocchi system monitoring..."
print_monitor "Press Ctrl+C to stop monitoring"

# Create log file
echo "$(date): Monitoring started" > "$LOG_FILE"

# Monitoring functions
check_ros2_environment() {
    if [ ! -z "$ROS_DISTRO" ]; then
        print_success "ROS2 $ROS_DISTRO environment active"
        return 0
    else
        print_error "ROS2 environment not active"
        return 1
    fi
}

check_system_resources() {
    print_header "=== SYSTEM RESOURCES ==="
    
    # CPU Usage (simplified without bc dependency)
    local cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
    print_success "CPU Usage: ${cpu_usage}%"
    
    # Memory Usage
    local memory_info=$(free | grep Mem)
    local total_mem=$(echo $memory_info | awk '{print $2}')
    local used_mem=$(echo $memory_info | awk '{print $3}')
    local memory_percent=$((used_mem * 100 / total_mem))
    
    if [ $memory_percent -gt $ALERT_THRESHOLD_MEMORY ]; then
        print_warning "Memory Usage: ${memory_percent}% (HIGH)"
    else
        print_success "Memory Usage: ${memory_percent}%"
    fi
    
    # Disk Usage
    local disk_usage=$(df / | tail -1 | awk '{print $5}' | cut -d'%' -f1)
    if [ $disk_usage -gt $ALERT_THRESHOLD_DISK ]; then
        print_warning "Disk Usage: ${disk_usage}% (HIGH)"
    else
        print_success "Disk Usage: ${disk_usage}%"
    fi
    
    # Load Average
    local load_avg=$(uptime | awk -F'load average:' '{ print $2 }')
    print_metric "Load Average:$load_avg"
}

check_network_ports() {
    print_header "=== NETWORK SERVICES ==="
    
    # Required ports for bocchi system
    local ports=(5000 8000 8765)
    local services=("REST API" "Web Interface" "WebSocket Server")
    
    for i in "${!ports[@]}"; do
        local port=${ports[$i]}
        local service=${services[$i]}
        
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            print_success "$service (port $port): LISTENING"
        else
            print_error "$service (port $port): NOT LISTENING"
        fi
    done
}

check_ros2_processes() {
    print_header "=== ROS2 PROCESSES ==="
    
    # Check for bocchi publisher process
    if pgrep -f "bocchi.*publisher" >/dev/null; then
        local pid=$(pgrep -f "bocchi.*publisher")
        local cpu_usage=$(ps -p $pid -o %cpu --no-headers 2>/dev/null || echo "0")
        local mem_usage=$(ps -p $pid -o %mem --no-headers 2>/dev/null || echo "0")
        print_success "Publisher Process: RUNNING (PID: $pid, CPU: ${cpu_usage}%, MEM: ${mem_usage}%)"
    else
        print_error "Publisher Process: NOT RUNNING"
    fi
    
    # Check ROS2 daemon
    if pgrep -f "_ros2_daemon" >/dev/null; then
        print_success "ROS2 Daemon: RUNNING"
    else
        print_warning "ROS2 Daemon: NOT RUNNING"
    fi
}

check_service_endpoints() {
    print_header "=== SERVICE ENDPOINTS ==="
    
    # Check REST API health
    if curl -s -f http://localhost:5000/api/status >/dev/null 2>&1; then
        local response=$(curl -s http://localhost:5000/api/status | jq -r '.status' 2>/dev/null || echo "unknown")
        print_success "REST API: HEALTHY (status: $response)"
    else
        print_error "REST API: UNHEALTHY or UNREACHABLE"
    fi
    
    # Check WebSocket info endpoint
    if curl -s -f http://localhost:5000/api/websocket-info >/dev/null 2>&1; then
        local ws_clients=$(curl -s http://localhost:5000/api/websocket-info | jq -r '.connected_clients' 2>/dev/null || echo "0")
        print_success "WebSocket Server: ACTIVE ($ws_clients connected clients)"
    else
        print_error "WebSocket Server: INACTIVE or UNREACHABLE"
    fi
    
    # Check web interface
    if curl -s -f http://localhost:5000 >/dev/null 2>&1; then
        print_success "Web Interface: ACCESSIBLE"
    else
        print_error "Web Interface: INACCESSIBLE"
    fi
}

check_ros2_topics() {
    print_header "=== ROS2 TOPICS ==="
    
    # Source ROS2 environment if needed
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
    fi
    
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash >/dev/null 2>&1
    fi
    
    # Check key topics
    local topics=("/cmd_vel" "/servo_position")
    
    for topic in "${topics[@]}"; do
        if timeout 2 ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
            local hz=$(timeout 2 ros2 topic hz $topic --window 10 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
            if [ "$hz" != "0" ]; then
                print_success "Topic $topic: ACTIVE (${hz} Hz)"
            else
                print_warning "Topic $topic: EXISTS but NO DATA"
            fi
        else
            print_error "Topic $topic: NOT FOUND"
        fi
    done
}

check_log_files() {
    print_header "=== LOG FILES ==="
    
    # Check ROS2 logs
    local ros_log_dir="$HOME/.ros/log"
    if [ -d "$ros_log_dir" ]; then
        local latest_log=$(ls -t "$ros_log_dir" | head -1)
        if [ ! -z "$latest_log" ]; then
            local log_size=$(du -sh "$ros_log_dir/$latest_log" 2>/dev/null | awk '{print $1}')
            print_success "ROS2 Logs: AVAILABLE ($log_size in $latest_log)"
            
            # Check for recent errors
            local error_count=$(find "$ros_log_dir/$latest_log" -name "*.log" -exec grep -l "ERROR\|FATAL" {} \; 2>/dev/null | wc -l)
            if [ $error_count -gt 0 ]; then
                print_warning "Found $error_count log files with errors"
            fi
        fi
    else
        print_warning "ROS2 Logs: NOT FOUND"
    fi
    
    # Check application logs
    if [ -f "bocchi.log" ]; then
        local app_log_size=$(du -sh bocchi.log | awk '{print $1}')
        print_success "Application Logs: AVAILABLE ($app_log_size)"
    else
        print_metric "Application Logs: NONE"
    fi
}

check_websocket_connections() {
    print_header "=== WEBSOCKET MONITORING ==="
    
    # Test WebSocket connection
    if command -v wscat >/dev/null 2>&1; then
        local ws_test=$(timeout 3 wscat -c ws://localhost:8765 -x '{"type":"ping"}' 2>/dev/null || echo "failed")
        if [ "$ws_test" != "failed" ]; then
            print_success "WebSocket Test: CONNECTION OK"
        else
            print_error "WebSocket Test: CONNECTION FAILED"
        fi
    else
        # Alternative test using netcat
        if nc -z localhost 8765 2>/dev/null; then
            print_success "WebSocket Port: LISTENING"
        else
            print_error "WebSocket Port: NOT LISTENING"
        fi
    fi
}

performance_metrics() {
    print_header "=== PERFORMANCE METRICS ==="
    
    # Network connections
    local tcp_connections=$(netstat -tan 2>/dev/null | grep ESTABLISHED | wc -l)
    print_metric "TCP Connections: $tcp_connections"
    
    # File descriptors
    local open_files=$(lsof 2>/dev/null | wc -l)
    print_metric "Open Files: $open_files"
    
    # System uptime
    local uptime_info=$(uptime | awk -F'up ' '{print $2}' | awk -F',' '{print $1}')
    print_metric "System Uptime: $uptime_info"
    
    # Temperature (if available)
    if [ -f "/sys/class/thermal/thermal_zone0/temp" ]; then
        local temp=$(($(cat /sys/class/thermal/thermal_zone0/temp) / 1000))
        if [ $temp -gt 70 ]; then
            print_warning "CPU Temperature: ${temp}°C (HIGH)"
        else
            print_success "CPU Temperature: ${temp}°C"
        fi
    fi
}

generate_health_report() {
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")
    local report_file="health_report_$(date +%Y%m%d_%H%M%S).txt"
    
    print_monitor "Generating health report: $report_file"
    
    {
        echo "Bocchi Robot Controller Health Report"
        echo "Generated: $timestamp"
        echo "========================================"
        echo ""
        
        # System overview
        echo "SYSTEM OVERVIEW:"
        echo "OS: $(uname -o) $(uname -r)"
        echo "Architecture: $(uname -m)"
        echo "Hostname: $(hostname)"
        echo "User: $(whoami)"
        echo ""
        
        # Resource summary
        echo "RESOURCE SUMMARY:"
        echo "CPU: $(nproc) cores"
        echo "Memory: $(free -h | grep Mem | awk '{print $2}') total"
        echo "Disk: $(df -h / | tail -1 | awk '{print $2}') total"
        echo ""
        
        # Service status
        echo "SERVICE STATUS:"
        echo "ROS2 Distro: ${ROS_DISTRO:-Not Set}"
        echo "Publisher: $(pgrep -f "bocchi.*publisher" >/dev/null && echo "Running" || echo "Stopped")"
        echo "REST API: $(curl -s -f http://localhost:5000/api/status >/dev/null 2>&1 && echo "Active" || echo "Inactive")"
        echo "WebSocket: $(nc -z localhost 8765 2>/dev/null && echo "Listening" || echo "Not Listening")"
        echo "Web UI: $(curl -s -f http://localhost:5000 >/dev/null 2>&1 && echo "Accessible" || echo "Inaccessible")"
        
    } > "$report_file"
    
    print_success "Health report saved to: $report_file"
}

# Signal handlers
cleanup() {
    print_monitor "Stopping monitoring..."
    echo "$(date): Monitoring stopped" >> "$LOG_FILE"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Parse command line arguments
CONTINUOUS=true
REPORT_ONLY=false
INTERVAL=$MONITOR_INTERVAL

while [[ $# -gt 0 ]]; do
    case $1 in
        --once|-o)
            CONTINUOUS=false
            shift
            ;;
        --report|-r)
            REPORT_ONLY=true
            shift
            ;;
        --interval|-i)
            INTERVAL="$2"
            shift 2
            ;;
        --help|-h)
            echo "bocchi Robot Controller System Monitor"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -o, --once      Run monitoring checks once and exit"
            echo "  -r, --report    Generate health report and exit"
            echo "  -i, --interval  Set monitoring interval in seconds (default: 5)"
            echo "  -h, --help      Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0              # Continuous monitoring"
            echo "  $0 --once       # Single check"
            echo "  $0 --report     # Generate health report"
            echo "  $0 -i 10        # Monitor every 10 seconds"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            print_status "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Report only mode
if [ "$REPORT_ONLY" = true ]; then
    generate_health_report
    exit 0
fi

# Main monitoring loop
monitor_cycle() {
    clear
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")
    
    print_header "🔍 BOCCHI ROBOT CONTROLLER SYSTEM MONITOR"
    print_header "Timestamp: $timestamp"
    print_header "=============================================="
    echo ""
    
    # Run all checks
    check_ros2_environment
    echo ""
    check_system_resources
    echo ""
    check_network_ports
    echo ""
    check_ros2_processes
    echo ""
    check_service_endpoints
    echo ""
    check_ros2_topics
    echo ""
    check_websocket_connections
    echo ""
    performance_metrics
    echo ""
    check_log_files
    echo ""
    
    # Log monitoring cycle
    echo "$(date): Monitor cycle completed" >> "$LOG_FILE"
    
    if [ "$CONTINUOUS" = true ]; then
        print_monitor "Next check in ${INTERVAL} seconds... (Press Ctrl+C to stop)"
        echo ""
        print_status "Quick Actions:"
        echo "  • View logs: tail -f $LOG_FILE"
        echo "  • Generate report: $0 --report"
        echo "  • Single check: $0 --once"
    fi
}

# Run monitoring
if [ "$CONTINUOUS" = true ]; then
    while true; do
        monitor_cycle
        sleep "$INTERVAL"
    done
else
    monitor_cycle
fi