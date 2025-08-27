#!/bin/bash

# Pipeline Status Check Script
# Quickly check the health and status of the synchronous pipeline

echo "🔍 Pipeline Status Check"
echo "========================"

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if ROS2 container is running
if docker compose ps | grep -q "ros2.*running"; then
    echo -e "${GREEN}✅ ROS2 container is running${NC}"
else
    echo -e "${RED}❌ ROS2 container is not running${NC}"
    echo "Start with: make up"
    exit 1
fi

echo ""
echo "📡 Checking pipeline topics..."

# Check pipeline topics
TOPICS=$(docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic list | grep pipeline" 2>/dev/null)

if [ -n "$TOPICS" ]; then
    echo -e "${GREEN}✅ Pipeline topics found:${NC}"
    echo "$TOPICS"
else
    echo -e "${YELLOW}⚠️  No pipeline topics found${NC}"
    echo "Pipeline coordinator may not be running"
fi

echo ""
echo "📊 Checking pipeline state..."

# Check current pipeline state
STATE=$(docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 3s ros2 topic echo /pipeline/state --once" 2>/dev/null | grep "data:" | cut -d'"' -f2)

if [ -n "$STATE" ]; then
    case $STATE in
        "idle")
            echo -e "${GREEN}🟢 Pipeline State: IDLE (Ready for next item)${NC}"
            ;;
        "processing")
            echo -e "${YELLOW}🔴 Pipeline State: PROCESSING (Item being sorted)${NC}"
            ;;
        *)
            echo -e "${BLUE}🔵 Pipeline State: $STATE${NC}"
            ;;
    esac
else
    echo -e "${YELLOW}⚠️  No pipeline state available${NC}"
    echo "Pipeline coordinator may not be running"
fi

echo ""
echo "🔍 Checking pipeline nodes..."

# Check if pipeline coordinator is running
NODES=$(docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 node list" 2>/dev/null)

if echo "$NODES" | grep -q "pipeline_coordinator"; then
    echo -e "${GREEN}✅ Pipeline coordinator is running${NC}"
else
    echo -e "${RED}❌ Pipeline coordinator is not running${NC}"
fi

if echo "$NODES" | grep -q "classifier_node"; then
    echo -e "${GREEN}✅ Classifier node is running${NC}"
else
    echo -e "${RED}❌ Classifier node is not running${NC}"
fi

if echo "$NODES" | grep -q "sorting_node"; then
    echo -e "${GREEN}✅ Sorting node is running${NC}"
else
    echo -e "${RED}❌ Sorting node is not running${NC}"
fi

if echo "$NODES" | grep -q "mock_camera_node\|camera_node"; then
    echo -e "${GREEN}✅ Camera node is running${NC}"
else
    echo -e "${RED}❌ Camera node is not running${NC}"
fi

echo ""
echo "🌐 Checking backend API..."

# Check backend pipeline state endpoint
if curl -s http://localhost:8000/api/pipeline/state >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Backend API is accessible${NC}"
    
    # Get pipeline state from backend
    BACKEND_STATE=$(curl -s http://localhost:8000/api/pipeline/state | grep -o '"state":"[^"]*"' | cut -d'"' -f4)
    if [ -n "$BACKEND_STATE" ]; then
        echo -e "${BLUE}📊 Backend reports pipeline state: $BACKEND_STATE${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Backend API is not accessible${NC}"
    echo "Backend may not be running"
fi

echo ""
echo "📋 Summary:"
if [ "$STATE" = "idle" ] || [ "$STATE" = "processing" ]; then
    echo -e "${GREEN}✅ Pipeline is operational${NC}"
elif [ -n "$TOPICS" ]; then
    echo -e "${YELLOW}⚠️  Pipeline topics exist but state unclear${NC}"
else
    echo -e "${RED}❌ Pipeline is not operational${NC}"
fi

echo ""
echo "🎯 Next steps:"
echo "  - To test pipeline: ./quick_pipeline_test.sh"
echo "  - To monitor real-time: make monitor-pipeline"
echo "  - To launch robot: make launch-robot"
echo "  - To verify components: make verify-pipeline"
