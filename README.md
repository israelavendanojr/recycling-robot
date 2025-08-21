BUILD ORDER: 

React: 
cd ../frontend
npm install
npm run build

Backend: 
cd ../backend
docker compose up --build -d
docker compose ps   # confirm ros2 and api are up

ROS: 
docker compose exec ros2 bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch recycling_robot robot.launch.py