#!/bin/bash

# ROS2 Docker Environment Script
# This script provides easy access to the ROS2 Docker container with proper environment setup

set -e

CONTAINER_NAME="ros2-bocchi-dev"
IMAGE_NAME="ros2-bocchi"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running in TTY
function has_tty() {
    [ -t 0 ] && [ -t 1 ]
}

function log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

function log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

function log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

function log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

function show_help() {
    echo "Usage: $0 [COMMAND] [ARGS...]"
    echo ""
    echo "Commands:"
    echo "  exec [cmd]   Execute command in container with ROS2 environment"
    echo "  shell        Open interactive bash shell in container"
    echo "  build        Build the ROS2 workspace"
    echo "  setup        Install dependencies"
    echo "  test         Run tests"
    echo "  run [node]   Run a ROS2 node"
    echo "  launch [pkg] [launch_file]  Launch a ROS2 launch file"
    echo "  status       Check container status"
    echo "  help         Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 shell                    # Open interactive shell"
    echo "  $0 build                    # Build the workspace"
    echo "  $0 exec 'colcon list'       # Execute custom command"
    echo "  $0 run bocchi control_node  # Run a specific node"
    echo "  $0 launch bocchi main.launch.py  # Launch a launch file"
}

function check_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        log_error "Container '$CONTAINER_NAME' is not running."
        log_info "Please start the container first with: ./docker.sh run"
        exit 1
    fi
}

function setup_env_command() {
    echo "cd /workdir/ros2_ws && source /opt/ros/humble/setup.bash && if [ -f install/setup.bash ]; then source install/setup.bash; fi"
}

function exec_in_container() {
    local cmd="$1"
    local env_setup=$(setup_env_command)
    
    if [ -z "$cmd" ]; then
        log_error "No command provided"
        exit 1
    fi
    
    log_info "Executing in container: $cmd"
    if has_tty; then
        docker exec -it "$CONTAINER_NAME" bash -c "$env_setup && $cmd"
    else
        docker exec "$CONTAINER_NAME" bash -c "$env_setup && $cmd"
    fi
}

function shell_in_container() {
    local env_setup=$(setup_env_command)
    log_info "Opening interactive shell in container..."
    log_info "ROS2 environment will be sourced automatically"
    docker exec -it "$CONTAINER_NAME" bash -c "$env_setup && exec bash"
}

function build_workspace() {
    log_info "Building ROS2 workspace..."
    if has_tty; then
        docker exec -it "$CONTAINER_NAME" bash -c "$(setup_env_command) && ./scripts/build.sh"
    else
        docker exec "$CONTAINER_NAME" bash -c "$(setup_env_command) && ./scripts/build.sh"
    fi
}

function setup_dependencies() {
    log_info "Installing dependencies..."
    if has_tty; then
        docker exec -it "$CONTAINER_NAME" bash -c "$(setup_env_command) && ./scripts/setup.sh"
    else
        docker exec "$CONTAINER_NAME" bash -c "$(setup_env_command) && ./scripts/setup.sh"
    fi
}

function run_tests() {
    log_info "Running tests..."
    if has_tty; then
        docker exec -it "$CONTAINER_NAME" bash -c "$(setup_env_command) && ./scripts/test.sh"
    else
        docker exec "$CONTAINER_NAME" bash -c "$(setup_env_command) && ./scripts/test.sh"
    fi
}

function run_node() {
    local package="$1"
    local node="$2"
    
    if [ -z "$package" ] || [ -z "$node" ]; then
        log_error "Usage: $0 run <package> <node>"
        exit 1
    fi
    
    log_info "Running node: $package/$node"
    if has_tty; then
        docker exec -it "$CONTAINER_NAME" bash -c "$(setup_env_command) && ros2 run $package $node"
    else
        docker exec "$CONTAINER_NAME" bash -c "$(setup_env_command) && ros2 run $package $node"
    fi
}

function launch_file() {
    local package="$1"
    local launch_file="$2"
    
    if [ -z "$package" ] || [ -z "$launch_file" ]; then
        log_error "Usage: $0 launch <package> <launch_file>"
        exit 1
    fi
    
    log_info "Launching: $package/$launch_file"
    if has_tty; then
        docker exec -it "$CONTAINER_NAME" bash -c "$(setup_env_command) && ros2 launch $package $launch_file"
    else
        docker exec "$CONTAINER_NAME" bash -c "$(setup_env_command) && ros2 launch $package $launch_file"
    fi
}

function check_status() {
    log_info "Checking container status..."
    
    if docker ps | grep -q "$CONTAINER_NAME"; then
        log_success "Container '$CONTAINER_NAME' is running"
        
        # Show container info
        echo ""
        echo "Container Details:"
        docker ps --filter "name=$CONTAINER_NAME" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
        
        # Check ROS2 environment
        echo ""
        log_info "Testing ROS2 environment..."
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 --help" >/dev/null 2>&1; then
            log_success "ROS2 environment is working"
        else
            log_warning "ROS2 environment may have issues"
        fi
        
        # Check workspace
        echo ""
        log_info "Workspace status:"
        docker exec "$CONTAINER_NAME" bash -c "cd /workdir/ros2_ws && ls -la src/ 2>/dev/null || echo 'No src directory found'"
        
    else
        log_warning "Container '$CONTAINER_NAME' is not running"
        log_info "Start it with: ./docker.sh run"
    fi
}

# Main command handling
case "$1" in
    exec)
        check_container
        shift
        exec_in_container "$*"
        ;;
    shell)
        check_container
        shell_in_container
        ;;
    build)
        check_container
        build_workspace
        ;;
    setup)
        check_container
        setup_dependencies
        ;;
    test)
        check_container
        run_tests
        ;;
    run)
        check_container
        shift
        run_node "$1" "$2"
        ;;
    launch)
        check_container
        shift
        launch_file "$1" "$2"
        ;;
    status)
        check_status
        ;;
    help|--help|-h)
        show_help
        ;;
    "")
        log_warning "No command provided"
        show_help
        exit 1
        ;;
    *)
        log_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac