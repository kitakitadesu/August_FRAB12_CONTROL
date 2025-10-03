#!/bin/bash

# Docker management script for ROS2 development environment

set -e

CONTAINER_NAME="ros2-bocchi-dev"
IMAGE_NAME="ros2-bocchi"
DOCKERFILE="Dockerfile"  # Default to base image
PLATFORM=""  # Default to native platform

function show_help() {
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  build [--desktop] [--platform]    Build the Docker image (default: ros-base, --desktop: ros-desktop)"
    echo "  run [--desktop] [--platform]      Run the container (creates new container)"
    echo "  start                Start existing container"
    echo "  stop                 Stop the container"
    echo "  shell                Open a shell in the running container"
    echo "  ssh                  SSH into the container (password: password)"
    echo "  logs                 Show container logs"
    echo "  clean                Remove container and image"
    echo "  help                 Show this help message"
    echo ""
    echo "Options:"
    echo "  --desktop            Use ros:humble-desktop image (Dockerfile.desktop)"
    echo "                       Default: ros:humble-ros-base image (Dockerfile)"
    echo "  --platform           Build for specific platform (e.g., --platform linux/amd64)"
    echo "                       Default: native platform"
    echo ""
    echo "ROS2 Development:"
    echo "  Use ./ros2-docker.sh for ROS2 commands (build, run, shell, etc.)"
    echo ""
    echo "SSH Access:"
    echo "  ssh root@localhost -p 2222"
    echo "  Password: password"
}

function build_image() {
    echo "Building Docker image using $DOCKERFILE..."
    if [ -n "$PLATFORM" ]; then
        echo "Building for platform: $PLATFORM"
        docker build --platform $PLATFORM -f $DOCKERFILE -t $IMAGE_NAME .
    else
        docker build -f $DOCKERFILE -t $IMAGE_NAME .
    fi
}

function run_container() {
    echo "Running new container..."
    local run_args=""
    if [ -n "$PLATFORM" ]; then
        run_args="--platform $PLATFORM"
    fi
    docker run -d \
        --name $CONTAINER_NAME \
        -v "$(pwd):/workdir" \
        -p 2222:22 \
        -e DISPLAY=$DISPLAY \
        --privileged \
        --network host \
        $run_args \
        $IMAGE_NAME
    echo "Container started. SSH access: ssh root@localhost -p 2222"
}

function start_container() {
    echo "Starting existing container..."
    docker start $CONTAINER_NAME
}

function stop_container() {
    echo "Stopping container..."
    docker stop $CONTAINER_NAME
}

function shell_container() {
    echo "Opening shell in container..."
    docker exec -it $CONTAINER_NAME bash
}

function ssh_container() {
    echo "Connecting via SSH (password: password)..."
    ssh root@localhost -p 2222
}

function show_logs() {
    echo "Showing container logs..."
    docker logs $CONTAINER_NAME
}

function clean_all() {
    echo "Cleaning up container and image..."
    docker stop $CONTAINER_NAME 2>/dev/null || true
    docker rm $CONTAINER_NAME 2>/dev/null || true
    docker rmi $IMAGE_NAME 2>/dev/null || true
    echo "Cleanup complete."
}

# Parse options
for arg in "$@"; do
    case $arg in
        --desktop)
            DOCKERFILE="Dockerfile.desktop"
            IMAGE_NAME="ros2-bocchi-desktop"
            CONTAINER_NAME="ros2-bocchi-dev-desktop"
            ;;
        --platform)
            # Get the next argument as platform value
            shift
            PLATFORM="$1"
            ;;
    esac
done

case "$1" in
    build)
        build_image
        ;;
    run)
        build_image
        run_container
        ;;
    start)
        start_container
        ;;
    stop)
        stop_container
        ;;
    shell)
        shell_container
        ;;
    ssh)
        ssh_container
        ;;
    logs)
        show_logs
        ;;
    clean)
        clean_all
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo "Unknown command: $1"
        show_help
        exit 1
        ;;
esac