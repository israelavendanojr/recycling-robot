# Recycling Robot Makefile
# Essential commands for managing the recycling robot system

.PHONY: install up down build launch-robot clean run

install: ## Install project dependencies
	@echo "Installing dependencies..."
	@echo "Dependencies installed"

up: ## Start all Docker services
	@echo "Starting Docker services..."
	docker compose up -d
	@echo "Services started"

down: ## Stop all Docker services
	@echo "Stopping Docker services..."
	docker compose down
	@echo "Services stopped"

build: ## Build ROS2 package
	@echo "Building ROS2 package..."
	docker compose exec -T ros2 bash -c " \
		cd /workspace/ros2_ws && \
		source /opt/ros/humble/setup.bash && \
		colcon build --packages-select recycling_robot \
	"
	@echo "ROS2 package built"

launch-robot: ## Launch the complete robot system with manual camera capture
	@echo "Launching recycling robot with manual capture pipeline..."
	@echo "Starting Docker services..."
	@docker compose up -d
	@sleep 5
	@echo "Starting ROS2 pipeline and key listener..."
	@echo "Starting ROS2 pipeline in background..."
	@docker compose exec -d ros2 bash -c " \
		cd /workspace/ros2_ws && \
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		ros2 launch recycling_robot robot.launch.py \
	"
	@sleep 3
	@echo "Starting key listener..."
	@python3 key_listener.py
	@echo "Robot system stopped"

clean: ## Remove ROS2 build artifacts
	@echo "Cleaning ROS2 build artifacts..."
	docker compose exec -T ros2 bash -c "cd /workspace/ros2_ws && rm -rf build/ install/ log/"
	@echo "Build artifacts cleaned"

run: ## Full pipeline setup (down + up + build + launch-robot)
	@echo "Running full pipeline setup..."
	@echo "Step 1: Clean shutdown of any running services..."
	@$(MAKE) down
	@echo "Step 2: Starting Docker services..."
	@$(MAKE) up
	@echo "Step 3: Building ROS2 package (ensuring updated classifier code)..."
	@$(MAKE) build
	@echo "Step 4: Launching robot pipeline with manual camera capture..."
	@echo "Press 'c' in the terminal to capture frames when ready"
	@$(MAKE) launch-robot
