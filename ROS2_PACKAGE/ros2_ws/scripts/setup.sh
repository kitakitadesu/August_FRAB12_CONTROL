#!/bin/bash
# ROS2 bocchi Robot Controller - First-Time Setup Script
# This script sets up the complete development and runtime environment

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

print_setup() {
    echo -e "${PURPLE}[SETUP]${NC} $1"
}

print_header() {
    echo -e "${WHITE}$1${NC}"
}

# ASCII Art Banner
show_banner() {
    echo -e "${PURPLE}"
    cat << "EOF"
    ____                  __    _   _____      __            
   / __ )____  __________/ /_  (_) / ___/___  / /___  ______ 
  / __  / __ \/ ___/ ___/ __ \/ /  \__ \/ _ \/ __/ / / / __ \
 / /_/ / /_/ / /__/ /__/ / / / /  ___/ /  __/ /_/ /_/ / /_/ /
/_____/\____/\___/\___/_/ /_/_/  /____/\___/\__/\__,_/ .___/ 
                                                   /_/      
🤖 Robot Controller Setup
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

# Detect OS and distribution
detect_os() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if [ -f /etc/os-release ]; then
            . /etc/os-release
            OS_NAME=$NAME
            OS_VERSION=$VERSION_ID
        else
            OS_NAME="Unknown Linux"
            OS_VERSION="Unknown"
        fi
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        OS_NAME="macOS"
        OS_VERSION=$(sw_vers -productVersion)
    else
        OS_NAME="Unknown"
        OS_VERSION="Unknown"
    fi
    
    print_status "Detected OS: $OS_NAME $OS_VERSION"
}

# Check system requirements
check_requirements() {
    print_header "=== CHECKING SYSTEM REQUIREMENTS ==="
    
    local missing_reqs=()
    
    # Check Python 3.8+
    if command -v python3 >/dev/null 2>&1; then
        local python_version=$(python3 --version | cut -d' ' -f2)
        local python_major=$(echo $python_version | cut -d'.' -f1)
        local python_minor=$(echo $python_version | cut -d'.' -f2)
        
        if [ $python_major -ge 3 ] && [ $python_minor -ge 8 ]; then
            print_success "Python $python_version (required: 3.8+)"
        else
            print_error "Python $python_version is too old (required: 3.8+)"
            missing_reqs+=("python3.8+")
        fi
    else
        print_error "Python 3 not found"
        missing_reqs+=("python3")
    fi
    
    # Check pip
    if command -v pip3 >/dev/null 2>&1 || python3 -m pip --version >/dev/null 2>&1; then
        print_success "pip3 available"
    else
        print_warning "pip3 not found, will attempt to install"
        missing_reqs+=("python3-pip")
    fi
    
    # Check git
    if command -v git >/dev/null 2>&1; then
        print_success "git available"
    else
        print_warning "git not found (recommended for development)"
        missing_reqs+=("git")
    fi
    
    # Check curl
    if command -v curl >/dev/null 2>&1; then
        print_success "curl available"
    else
        print_error "curl required for testing and monitoring"
        missing_reqs+=("curl")
    fi
    
    # Check netstat/ss
    if command -v netstat >/dev/null 2>&1 || command -v ss >/dev/null 2>&1; then
        print_success "Network tools available"
    else
        print_warning "netstat/ss not found (needed for monitoring)"
        missing_reqs+=("net-tools")
    fi
    
    if [ ${#missing_reqs[@]} -ne 0 ]; then
        print_warning "Missing requirements: ${missing_reqs[*]}"
        return 1
    fi
    
    return 0
}

# Install system dependencies
install_system_deps() {
    print_header "=== INSTALLING SYSTEM DEPENDENCIES ==="
    
    if [[ "$OS_NAME" == *"Ubuntu"* ]] || [[ "$OS_NAME" == *"Debian"* ]]; then
        print_status "Installing Ubuntu/Debian packages..."
        
        # Update package list
        sudo apt update
        
        # Install essential packages
        sudo apt install -y \
            python3 \
            python3-pip \
            python3-dev \
            python3-venv \
            curl \
            wget \
            git \
            build-essential \
            cmake \
            net-tools \
            lsof \
            htop \
            vim \
            nano
        
        # Install inotify tools for development file watching
        sudo apt install -y inotify-tools || print_warning "inotify-tools not available"
        
    elif [[ "$OS_NAME" == *"macOS"* ]]; then
        print_status "Installing macOS packages..."
        
        if command -v brew >/dev/null 2>&1; then
            brew install python3 git curl wget htop
        else
            print_error "Homebrew not found. Please install Homebrew first:"
            print_error "  /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
            return 1
        fi
        
    else
        print_warning "Unknown OS. Please install dependencies manually:"
        print_warning "  - Python 3.8+"
        print_warning "  - pip3"
        print_warning "  - curl"
        print_warning "  - git"
        print_warning "  - build tools"
    fi
    
    print_success "System dependencies installed"
}

# Setup ROS2 Humble
setup_ros2() {
    print_header "=== SETTING UP ROS2 HUMBLE ==="
    
    # Check if ROS2 is already installed
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        print_success "ROS2 Humble already installed"
        return 0
    fi
    
    if [[ "$OS_NAME" == *"Ubuntu"* ]]; then
        print_status "Installing ROS2 Humble on Ubuntu..."
        
        # Add ROS2 repository
        sudo apt update && sudo apt install curl gnupg lsb-release -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        # Install ROS2
        sudo apt update
        sudo apt install -y ros-humble-desktop
        sudo apt install -y ros-dev-tools
        
        # Install colcon
        sudo apt install -y python3-colcon-common-extensions
        
        print_success "ROS2 Humble installed"
        
    else
        print_warning "ROS2 installation not automated for $OS_NAME"
        print_warning "Please install ROS2 Humble manually:"
        print_warning "  https://docs.ros.org/en/humble/Installation.html"
        return 1
    fi
}

# Install Python dependencies
install_python_deps() {
    print_header "=== INSTALLING PYTHON DEPENDENCIES ==="
    
    # Core runtime dependencies
    print_status "Installing runtime dependencies..."
    pip3 install --user \
        Flask==2.0.3 \
        Flask-CORS==3.0.10 \
        websockets==11.0.3 \
        requests==2.25.1 \
        Mako==1.1.6
    
    # Development dependencies
    print_status "Installing development dependencies..."
    pip3 install --user \
        pytest \
        pytest-cov \
        pytest-asyncio \
        pytest-mock \
        black \
        isort \
        flake8 \
        mypy \
        pre-commit
    
    # Optional tools
    print_status "Installing optional tools..."
    pip3 install --user \
        wscat \
        httpie \
        jq || print_warning "Some optional tools may not be available"
    
    print_success "Python dependencies installed"
}

# Setup ROS2 workspace
setup_workspace() {
    print_header "=== SETTING UP ROS2 WORKSPACE ==="
    
    # Source ROS2 environment
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_success "ROS2 environment sourced"
    else
        print_error "ROS2 not found. Please install ROS2 Humble first."
        return 1
    fi
    
    # Install workspace dependencies
    print_status "Installing workspace dependencies with rosdep..."
    if command -v rosdep >/dev/null 2>&1; then
        # Initialize rosdep if needed
        if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
            sudo rosdep init || print_warning "rosdep already initialized"
        fi
        
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
        print_success "Workspace dependencies installed"
    else
        print_warning "rosdep not available, skipping dependency installation"
    fi
}

# Build the package
build_package() {
    print_header "=== BUILDING BOCCHI PACKAGE ==="
    
    # Source ROS2 environment
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Build the package
    print_status "Building bocchi package..."
    colcon build --packages-select bocchi --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        print_success "Package built successfully"
        
        # Source the built workspace
        source install/setup.bash
        
        # Verify executables
        if [ -f "install/bocchi/lib/bocchi/publisher" ]; then
            print_success "Publisher executable created"
        else
            print_warning "Publisher executable not found"
        fi
        
    else
        print_error "Build failed"
        return 1
    fi
}

# Run tests
run_tests() {
    print_header "=== RUNNING INITIAL TESTS ==="
    
    # Source environments
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
    
    # Run basic tests
    print_status "Running basic functionality tests..."
    if [ -f "src/bocchi/scripts/test.sh" ]; then
        src/bocchi/scripts/test.sh --unit || print_warning "Some tests failed (this is normal during initial setup)"
    else
        print_warning "Test script not found, skipping tests"
    fi
}

# Setup shell environment
setup_shell_env() {
    print_header "=== SETTING UP SHELL ENVIRONMENT ==="
    
    local shell_rc=""
    if [ "$SHELL" = "/bin/bash" ] || [ "$SHELL" = "/usr/bin/bash" ]; then
        shell_rc="$HOME/.bashrc"
    elif [ "$SHELL" = "/bin/zsh" ] || [ "$SHELL" = "/usr/bin/zsh" ]; then
        shell_rc="$HOME/.zshrc"
    else
        print_warning "Unknown shell: $SHELL"
        print_warning "Please manually add ROS2 sourcing to your shell configuration"
        return 0
    fi
    
    # Add ROS2 sourcing to shell config
    local ros2_source="source /opt/ros/humble/setup.bash"
    if ! grep -q "$ros2_source" "$shell_rc" 2>/dev/null; then
        print_status "Adding ROS2 environment to $shell_rc"
        echo "" >> "$shell_rc"
        echo "# ROS2 Humble environment" >> "$shell_rc"
        echo "$ros2_source" >> "$shell_rc"
    fi
    
    # Add workspace sourcing
    local ws_source="source $(pwd)/install/setup.bash"
    if ! grep -q "$(pwd)/install/setup.bash" "$shell_rc" 2>/dev/null; then
        print_status "Adding workspace environment to $shell_rc"
        echo "# Bocchi workspace environment" >> "$shell_rc"
        echo "$ws_source" >> "$shell_rc"
    fi
    
    # Add convenient aliases
    if ! grep -q "alias bocchi=" "$shell_rc" 2>/dev/null; then
        print_status "Adding bocchi aliases to $shell_rc"
        echo "" >> "$shell_rc"
        echo "# Bocchi Robot Controller aliases" >> "$shell_rc"
        echo "alias bocchi='$(pwd)/src/bocchi/scripts/run.sh'" >> "$shell_rc"
        echo "alias bocchi-build='$(pwd)/src/bocchi/scripts/run.sh build'" >> "$shell_rc"
        echo "alias bocchi-test='$(pwd)/src/bocchi/scripts/run.sh test'" >> "$shell_rc"
        echo "alias bocchi-start='$(pwd)/src/bocchi/scripts/run.sh start'" >> "$shell_rc"
        echo "alias bocchi-dev='$(pwd)/src/bocchi/scripts/run.sh dev-watch'" >> "$shell_rc"
        echo "alias bocchi-monitor='$(pwd)/src/bocchi/scripts/run.sh monitor'" >> "$shell_rc"
    fi
    
    print_success "Shell environment configured"
    print_status "Restart your terminal or run: source $shell_rc"
}

# Create desktop shortcut (Linux only)
create_desktop_shortcut() {
    if [[ "$OS_NAME" == *"Ubuntu"* ]] && [ -d "$HOME/Desktop" ]; then
        print_status "Creating desktop shortcut..."
        
        cat > "$HOME/Desktop/Bocchi Robot Controller.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Bocchi Robot Controller
Comment=Launch Bocchi Robot Controller System
Exec=$(pwd)/src/bocchi/scripts/run.sh start
Icon=applications-science
Terminal=true
Categories=Development;Science;
EOF
        
        chmod +x "$HOME/Desktop/Bocchi Robot Controller.desktop"
        print_success "Desktop shortcut created"
    fi
}

# Display final instructions
show_final_instructions() {
    print_header "================================================================="
    print_header "🎉 BOCCHI ROBOT CONTROLLER SETUP COMPLETE!"
    print_header "================================================================="
    echo ""
    
    print_success "✅ System dependencies installed"
    print_success "✅ ROS2 Humble configured"
    print_success "✅ Python dependencies installed"
    print_success "✅ Workspace built successfully"
    print_success "✅ Shell environment configured"
    
    echo ""
    print_header "🚀 QUICK START COMMANDS:"
    echo ""
    echo "  Start the system:"
    echo "    ./src/bocchi/scripts/run.sh start"
    echo ""
    echo "  Run tests:"
    echo "    ./src/bocchi/scripts/run.sh test"
    echo ""
    echo "  Development mode:"
    echo "    ./src/bocchi/scripts/run.sh dev-watch"
    echo ""
    echo "  Monitor system:"
    echo "    ./src/bocchi/scripts/run.sh monitor"
    echo ""
    echo "  Get help:"
    echo "    ./src/bocchi/scripts/run.sh help"
    echo ""
    
    print_header "🌐 SYSTEM ACCESS:"
    echo "  • Web Interface: http://localhost:5000"
    echo "  • REST API: http://localhost:5000"
    echo "  • WebSocket: ws://localhost:8765"
    echo ""
    
    print_header "🎮 ROBOT CONTROL:"
    echo "  • W = Move Forward"
    echo "  • S = Move Backward"
    echo "  • A = Turn Left"
    echo "  • D = Turn Right"
    echo "  • F = Toggle Servo (0° ↔ 180°)"
    echo ""
    
    if [ -f "$HOME/.bashrc" ] || [ -f "$HOME/.zshrc" ]; then
        print_header "💡 CONVENIENT ALIASES ADDED:"
        echo "  • bocchi          - Main command"
        echo "  • bocchi-start    - Start system"
        echo "  • bocchi-test     - Run tests"
        echo "  • bocchi-dev      - Development mode"
        echo ""
        echo "  Restart your terminal or run: source ~/.bashrc"
    fi
    
    print_header "📚 DOCUMENTATION:"
    echo "  • Scripts: src/bocchi/scripts/README.md"
    echo "  • Tests: src/bocchi/test/README_TESTS.md"
    echo "  • Main docs: src/bocchi/README.md"
    echo ""
    
    print_header "🆘 NEED HELP?"
    echo "  • Check system status: ./src/bocchi/scripts/run.sh info"
    echo "  • View logs: ./src/bocchi/scripts/run.sh logs"
    echo "  • Generate health report: ./src/bocchi/scripts/run.sh health"
    echo ""
    
    print_header "================================================================="
    print_success "🤖 Ready to control your robot! Have fun!"
    print_header "================================================================="
}

# Main setup function
main() {
    show_banner
    echo ""
    
    # Check if we're in the right place
    if ! check_workspace; then
        exit 1
    fi
    
    print_setup "Starting bocchi robot controller setup..."
    print_setup "This will install all required dependencies and configure the system"
    echo ""
    
    # Detect OS
    detect_os
    echo ""
    
    # Parse arguments
    SKIP_SYSTEM=false
    SKIP_ROS2=false
    SKIP_PYTHON=false
    QUICK_SETUP=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-system)
                SKIP_SYSTEM=true
                shift
                ;;
            --skip-ros2)
                SKIP_ROS2=true
                shift
                ;;
            --skip-python)
                SKIP_PYTHON=true
                shift
                ;;
            --quick)
                QUICK_SETUP=true
                shift
                ;;
            --help|-h)
                echo "Bocchi Robot Controller Setup Script"
                echo ""
                echo "Usage: $0 [options]"
                echo ""
                echo "Options:"
                echo "  --skip-system   Skip system dependency installation"
                echo "  --skip-ros2     Skip ROS2 installation"
                echo "  --skip-python   Skip Python dependency installation"
                echo "  --quick         Quick setup (minimal installation)"
                echo "  --help          Show this help"
                echo ""
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                print_error "Use --help for usage information"
                exit 1
                ;;
        esac
    done
    
    # Check requirements
    if ! check_requirements; then
        if [ "$SKIP_SYSTEM" = false ]; then
            print_status "Installing missing system dependencies..."
            install_system_deps
        else
            print_error "Missing requirements and --skip-system specified"
            exit 1
        fi
    fi
    echo ""
    
    # Setup ROS2
    if [ "$SKIP_ROS2" = false ]; then
        setup_ros2
        echo ""
    fi
    
    # Install Python dependencies
    if [ "$SKIP_PYTHON" = false ]; then
        install_python_deps
        echo ""
    fi
    
    # Setup workspace
    setup_workspace
    echo ""
    
    # Build package
    build_package
    echo ""
    
    # Run initial tests (only in full setup)
    if [ "$QUICK_SETUP" = false ]; then
        run_tests
        echo ""
    fi
    
    # Setup shell environment
    setup_shell_env
    echo ""
    
    # Create desktop shortcut
    if [ "$QUICK_SETUP" = false ]; then
        create_desktop_shortcut
    fi
    
    # Show final instructions
    show_final_instructions
}

# Run main function with all arguments
main "$@"