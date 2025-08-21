docker compose exec ros2 bash

source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
source install/setup.bash
ros2 launch recycling_robot robot.launch.py
