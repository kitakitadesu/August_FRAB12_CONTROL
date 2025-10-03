#!/bin/bash

# Docker management script for ROS2 development environment

set -e

CONTAINER_NAME="ros2-bocchi-dev"
IMAGE_NAME="ros2-bocchi"

function show_help() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build    Build the Docker image"
    echo "  run      Run the container (creates new container)"
    echo "  start    Start existing container"
    echo "  stop     Stop the container"
    echo "  shell    Open a shell in the running container"
    echo "  ssh      SSH into the container (password: password)"
    echo "  logs     Show container logs"
    echo "  clean    Remove container and image"
    echo "  help     Show this help message"
    echo ""
    echo "ROS2 Development:"
    echo "  Use ./ros2-docker.sh for ROS2 commands (build, run, shell, etc.)"
    echo ""
    echo "SSH Access:"
    echo "  ssh root@localhost -p 2222"
    echo "  Password: password"
}

function build_image() {
    echo "Building Docker image..."
    docker build -t $IMAGE_NAME .
}

function run_container() {
    echo "Running new container..."
    docker run -d \
        --name $CONTAINER_NAME \
        -v "$(pwd):/workdir" \
        -p 2222:22 \
        -e DISPLAY=$DISPLAY \
        --privileged \
        --network host \
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