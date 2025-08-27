# Recycling Robot Makefile
# Includes commands for testing the synchronous pipeline

.PHONY: help install up down build test-pipeline monitor-pipeline clean

help: ## Show this help message
	@echo "Recycling Robot Management Commands:"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

install: ## Install dependencies
	@echo "Installing dependencies..."
	# Add your installation commands here
	@echo "✅ Dependencies installed"

up: ## Start services
	@echo "Starting services..."
	# Add your service startup commands here
	@echo "✅ Services started"

down: ## Stop services
	@echo "Stopping services..."
	# Add your service stop commands here
	@echo "✅ Services stopped"

build: ## Build ROS2 package
	@echo "Building ROS2 package..."
	docker compose exec ros2 bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select recycling_robot"
	@echo "✅ ROS2 package built"

test-pipeline: ## Test synchronous pipeline
	@echo "🧪 Testing synchronous pipeline..."
	docker compose exec ros2 bash -c " \
		cd /workspace/ros2_ws && \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		echo '🔍 Checking pipeline topics...' && \
		timeout 5s ros2 topic list | grep pipeline && \
		echo '🚀 Starting pipeline test...' && \
		timeout 20s ros2 launch recycling_robot robot.launch.py & \
		sleep 15 && \
		echo '📊 Pipeline state:' && \
		timeout 5s ros2 topic echo /pipeline/state --once && \
		echo '✅ Pipeline test completed' \
	"

monitor-pipeline: ## Monitor pipeline state in real-time
	@echo "📊 Monitoring pipeline state..."
	docker compose exec ros2 bash -c " \
		cd /workspace/ros2_ws && \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		echo '🔍 Available pipeline topics:' && \
		ros2 topic list | grep pipeline && \
		echo '📈 Monitoring pipeline state (Ctrl+C to stop):' && \
		ros2 topic echo /pipeline/state \
	"

test-pipeline-manual: ## Run manual pipeline test with test script
	@echo "🧪 Running manual pipeline test..."
	docker compose exec ros2 bash -c " \
		cd /workspace/ros2_ws && \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		echo '🚀 Starting pipeline coordinator...' && \
		ros2 launch recycling_robot robot.launch.py & \
		sleep 10 && \
		echo '🧪 Running test script...' && \
		python3 test_pipeline_comprehensive.py \
	"

verify-pipeline: ## Verify pipeline components are working
	@echo "🔍 Verifying pipeline components..."
	docker compose exec ros2 bash -c " \
		cd /workspace/ros2_ws && \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		echo '📋 Checking nodes:' && \
		ros2 node list | grep -E '(pipeline|classifier|sorting|camera)' && \
		echo '📡 Checking topics:' && \
		ros2 topic list | grep pipeline && \
		echo '📊 Checking pipeline state:' && \
		timeout 3s ros2 topic echo /pipeline/state --once || echo 'No pipeline state available yet' \
	"

launch-robot: ## Launch the complete robot system
	@echo "🚀 Launching recycling robot with synchronous pipeline..."
	docker compose exec ros2 bash -c " \
		cd /workspace/ros2_ws && \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		echo '🎯 Launching robot.launch.py...' && \
		ros2 launch recycling_robot robot.launch.py \
	"

clean: ## Clean build artifacts
	@echo "🧹 Cleaning build artifacts..."
	docker compose exec ros2 bash -c "cd /workspace/ros2_ws && rm -rf build/ install/ log/"
	@echo "✅ Build artifacts cleaned"

# Default target
all: install up build
	@echo "🎉 All tasks completed!"
