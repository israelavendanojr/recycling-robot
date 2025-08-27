#!/bin/bash

# Quick Pipeline Test Script
# Tests the synchronous pipeline implementation

echo "🧪 Quick Pipeline Test"
echo "======================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    local status=$1
    local message=$2
    case $status in
        "SUCCESS") echo -e "${GREEN}✅ $message${NC}" ;;
        "WARNING") echo -e "${YELLOW}⚠️  $message${NC}" ;;
        "ERROR") echo -e "${RED}❌ $message${NC}" ;;
        "INFO") echo -e "${BLUE}ℹ️  $message${NC}" ;;
    esac
}

echo "🔍 Checking if ROS2 container is running..."
if docker compose ps | grep -q "ros2.*running"; then
    print_status "SUCCESS" "ROS2 container is running"
else
    print_status "ERROR" "ROS2 container is not running. Start with: make up"
    exit 1
fi

echo ""
echo "🚀 Building ROS2 package..."
docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select recycling_robot"

if [ $? -eq 0 ]; then
    print_status "SUCCESS" "ROS2 package built successfully"
else
    print_status "ERROR" "Failed to build ROS2 package"
    exit 1
fi

echo ""
echo "🔍 Checking pipeline topics..."
docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic list | grep pipeline"

echo ""
echo "📊 Checking current pipeline state..."
docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 3s ros2 topic echo /pipeline/state --once" || echo "No pipeline state available yet"

echo ""
echo "🧪 Starting pipeline test (30 seconds)..."
docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 30s ros2 launch recycling_robot robot.launch.py" &

# Wait for launch to start
sleep 10

echo ""
echo "📈 Monitoring pipeline state changes..."
docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 15s ros2 topic echo /pipeline/state" &

# Wait for monitoring
sleep 15

echo ""
echo "🔍 Final pipeline status check..."
docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 3s ros2 topic echo /pipeline/state --once"

echo ""
print_status "INFO" "Test completed. Check the output above for pipeline behavior."
print_status "INFO" "You should see:"
print_status "INFO" "  - Pipeline coordinator starting"
print_status "INFO" "  - State transitions: idle → processing → idle"
print_status "INFO" "  - Camera nodes respecting pipeline state"
print_status "INFO" "  - Classifier and sorting coordination"

echo ""
echo "🎯 To run a longer test with the test script:"
echo "   make test-pipeline-manual"
echo ""
echo "📊 To monitor pipeline in real-time:"
echo "   make monitor-pipeline"
