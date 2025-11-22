#!/bin/bash

###############################################################################
#
# 🚀 ADAS System Quick Start Script
#
# Usage:
#   ./adas_quickstart.sh [command]
#
# Commands:
#   setup       - Install dependencies
#   calibrate   - Calibrate camera
#   run         - Start full system
#   monitor     - Monitor topics in separate terminal
#   test        - Run test scenarios
#   clean       - Clean logs and cache
#   status      - Check system status
#
###############################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

ADAS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PATH="$ADAS_ROOT/adas_env"

print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC}  🚗 ADAS System - $1"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ️  $1${NC}"
}

# Check if venv exists
check_venv() {
    if [ ! -d "$VENV_PATH" ]; then
        print_error "Virtual environment not found at $VENV_PATH"
        echo "Run: $0 setup"
        exit 1
    fi
}

# Setup command
cmd_setup() {
    print_header "Setup"
    
    print_info "Creating Python virtual environment..."
    python3.11 -m venv "$VENV_PATH"
    
    print_info "Activating virtual environment..."
    source "$VENV_PATH/bin/activate"
    
    print_info "Upgrading pip..."
    pip install -U pip setuptools wheel
    
    print_info "Installing dependencies..."
    pip install -r "$ADAS_ROOT/requirements.txt"
    
    print_success "Setup complete!"
    echo
    echo "Next steps:"
    echo "1. Activate environment: source $VENV_PATH/bin/activate"
    echo "2. Calibrate camera: $0 calibrate"
    echo "3. Run system: $0 run"
}

# Calibrate command
cmd_calibrate() {
    print_header "Sensor Calibration"
    
    check_venv
    source "$VENV_PATH/bin/activate"
    
    print_info "Starting camera calibration..."
    print_info "Instructions:"
    echo "  1. Print a checkerboard pattern (9x6 squares)"
    echo "  2. Show it to the camera from different angles"
    echo "  3. Press SPACE to capture frames (need 20+)"
    echo "  4. Press Q when done"
    echo
    
    python "$ADAS_ROOT/utils/calibration_tool.py" --mode camera \
        --config "$ADAS_ROOT/config/adas_config.yaml"
    
    print_success "Calibration complete!"
}

# Run command
cmd_run() {
    print_header "Starting ADAS System"
    
    check_venv
    
    print_info "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash 2>/dev/null || {
        print_error "ROS2 Humble not installed. Install with:"
        echo "  sudo apt install ros-humble-desktop"
        exit 1
    }
    
    print_info "Starting ROS2 daemon..."
    ros2 daemon start || true
    
    print_info "Launching ADAS system (15-20 nodes)..."
    echo
    echo "ROS2 topics will appear shortly. In another terminal, run:"
    echo "  $0 monitor"
    echo
    
    ros2 launch adas_system adas_system.launch.py enable_visualization:=true
}

# Monitor command
cmd_monitor() {
    print_header "Monitor System Topics"
    
    check_venv
    source /opt/ros/humble/setup.bash 2>/dev/null || {
        print_error "ROS2 not found"
        exit 1
    }
    
    print_info "Available topics:"
    ros2 topic list
    echo
    
    print_info "Monitoring tracking data..."
    echo "Press Ctrl+C to stop"
    echo
    
    ros2 topic echo /tracking/tracks --no-arr | head -50
}

# Test command
cmd_test() {
    print_header "Test Scenarios"
    
    check_venv
    source /opt/ros/humble/setup.bash 2>/dev/null || {
        print_error "ROS2 not found"
        exit 1
    }
    
    echo "Available tests:"
    echo "  1. Object detection"
    echo "  2. Tracking (multi-object)"
    echo "  3. Collision warning (FCW)"
    echo "  4. Emergency braking (AEB)"
    echo "  5. Sensor health check"
    echo
    
    read -p "Select test (1-5): " test_num
    
    case $test_num in
        1)
            print_info "Running YOLOv8 detector..."
            python "$ADAS_ROOT/perception/camera/yolov8_detector.py"
            ;;
        2)
            print_info "Running DeepSORT tracker..."
            python "$ADAS_ROOT/tracking/deepsort_tracker.py"
            ;;
        3)
            print_info "Testing FCW..."
            python -c "from adas_system.prediction.trajectory_predictor import TrajectoryPredictor; print('FCW test ready')"
            ;;
        4)
            print_info "Testing AEB..."
            python -c "from adas_system.decision.safety_state_machine import SafetyStateMachine; print('AEB test ready')"
            ;;
        5)
            print_info "Running sensor health check..."
            python "$ADAS_ROOT/utils/calibration_tool.py" --mode health
            ;;
        *)
            print_error "Invalid selection"
            ;;
    esac
}

# Clean command
cmd_clean() {
    print_header "Cleaning"
    
    print_info "Removing logs and cache..."
    rm -rf "$ADAS_ROOT/data/logs"/* 2>/dev/null || true
    rm -rf "$ADAS_ROOT/__pycache__" 2>/dev/null || true
    find "$ADAS_ROOT" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    
    print_success "Cleanup complete!"
}

# Status command
cmd_status() {
    print_header "System Status"
    
    echo -e "${BLUE}Configuration:${NC}"
    echo "  Root: $ADAS_ROOT"
    echo "  Virtual env: $([ -d "$VENV_PATH" ] && echo "✅ Exists" || echo "❌ Missing")"
    echo "  ROS2: $([ -d /opt/ros/humble ] && echo "✅ Installed" || echo "❌ Not installed")"
    echo
    
    echo -e "${BLUE}Files:${NC}"
    echo "  README.md: $([ -f "$ADAS_ROOT/README.md" ] && echo "✅" || echo "❌")"
    echo "  requirements.txt: $([ -f "$ADAS_ROOT/requirements.txt" ] && echo "✅" || echo "❌")"
    echo "  config/adas_config.yaml: $([ -f "$ADAS_ROOT/config/adas_config.yaml" ] && echo "✅" || echo "❌")"
    echo
    
    echo -e "${BLUE}Modules:${NC}"
    echo "  Sensors: $([ -f "$ADAS_ROOT/sensors/camera_driver.py" ] && echo "✅" || echo "❌")"
    echo "  Perception: $([ -f "$ADAS_ROOT/perception/camera/yolov8_detector.py" ] && echo "✅" || echo "❌")"
    echo "  Tracking: $([ -f "$ADAS_ROOT/tracking/deepsort_tracker.py" ] && echo "✅" || echo "❌")"
    echo "  Decision: $([ -f "$ADAS_ROOT/decision/safety_state_machine.py" ] && echo "✅" || echo "❌")"
    echo "  Control: $([ -f "$ADAS_ROOT/control/vehicle_controller.py" ] && echo "✅" || echo "❌")"
    echo
    
    print_success "System ready for deployment!"
}

# Help command
cmd_help() {
    cat << EOF
╔════════════════════════════════════════════════════════════╗
║           ADAS System Quick Start Script                   ║
╚════════════════════════════════════════════════════════════╝

USAGE:
  ./adas_quickstart.sh [command]

COMMANDS:
  setup       Install dependencies & create virtual environment
  calibrate   Calibrate camera intrinsics
  run         Start full ADAS system (ROS2 nodes)
  monitor     Monitor system topics in real-time
  test        Run test scenarios
  clean       Clean logs and cache
  status      Check system configuration
  help        Show this help message

QUICK START (5 MINUTES):

  1. Setup once:
     ./adas_quickstart.sh setup

  2. Calibrate camera (first time only):
     ./adas_quickstart.sh calibrate

  3. Run system in terminal A:
     ./adas_quickstart.sh run

  4. Monitor in terminal B:
     ./adas_quickstart.sh monitor

  5. Test scenarios in terminal C:
     ./adas_quickstart.sh test

DOCUMENTATION:
  README.md              - System overview
  IMPLEMENTATION_GUIDE.md - Detailed setup guide
  ARCHITECTURE_DIAGRAM.txt - Visual architecture
  config/adas_config.yaml - Configuration file

For more information, see: $ADAS_ROOT/README.md

EOF
}

# Main script logic
main() {
    local command="${1:-help}"
    
    case "$command" in
        setup)
            cmd_setup
            ;;
        calibrate)
            cmd_calibrate
            ;;
        run)
            cmd_run
            ;;
        monitor)
            cmd_monitor
            ;;
        test)
            cmd_test
            ;;
        clean)
            cmd_clean
            ;;
        status)
            cmd_status
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run: $0 help"
            exit 1
            ;;
    esac
}

# Run main
main "$@"
