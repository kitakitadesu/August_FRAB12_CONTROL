#!/bin/bash
# ROS2 Build Script for bocchi robot controller
# This script builds the bocchi package with proper ROS2 environment setup

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

# Check if we're in the correct directory
if [ ! -f "src/bocchi/package.xml" ]; then
    print_error "Must be run from ROS2 workspace root (ros2_ws)"
    print_error "Current directory: $(pwd)"
    print_error "Expected to find: src/bocchi/package.xml"
    exit 1
fi

print_status "Starting ROS2 build for bocchi package..."

# Source ROS2 environment
print_status "Sourcing ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    print_success "ROS2 Humble environment sourced"
else
    print_error "ROS2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Check for rosdep
print_status "Checking dependencies with rosdep..."
if command -v rosdep &> /dev/null; then
    if rosdep check --from-paths src --ignore-src -r &> /dev/null; then
        print_success "All dependencies satisfied"
    else
        print_warning "Some dependencies may be missing"
        print_status "Installing dependencies..."
        rosdep install --from-paths src --ignore-src -r -y
    fi
else
    print_warning "rosdep not found, skipping dependency check"
fi

# Clean previous build (optional)
if [ "$1" = "--clean" ] || [ "$1" = "-c" ]; then
    print_status "Cleaning previous build..."
    rm -rf build install log
    print_success "Build directories cleaned"
fi

# Build the package
print_status "Building bocchi package..."
if colcon build --packages-select bocchi --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    print_success "Build completed successfully!"
else
    print_error "Build failed!"
    exit 1
fi

# Source the built workspace
print_status "Sourcing built workspace..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    print_success "Workspace sourced successfully"
else
    print_warning "install/setup.bash not found"
fi

# Verify the build
print_status "Verifying build..."
if [ -f "install/bocchi/lib/bocchi/publisher" ]; then
    print_success "Publisher executable found"
else
    print_warning "Publisher executable not found"
fi

if [ -f "install/bocchi/lib/bocchi/subscriber" ]; then
    print_success "Subscriber executable found"
else
    print_warning "Subscriber executable not found"
fi

# Display build summary
print_status "Build Summary:"
echo "  📦 Package: bocchi"
echo "  🏗️  Build Type: Release"
echo "  📍 Install Path: $(pwd)/install/bocchi"
echo "  🎯 Executables:"
echo "    - publisher"
echo "    - subscriber"

print_status "Build completed! You can now run:"
echo "  🚀 Start publisher: ros2 run bocchi publisher"
echo "  📡 Start subscriber: ros2 run bocchi subscriber"
echo "  🧪 Run tests: ./scripts/test.sh"
echo "  🌐 Launch full system: ./scripts/launch.sh"

print_success "✅ Build process finished successfully!"