BUILD ORDER: 

Frontend: 
cd ../frontend
npm run build

Backend: 
cd ../backend
docker compose up --build -d

ROS: 
docker compose exec ros2 bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch recycling_robot robot.launch.py













docker compose -f /home/israelavendanojr/Desktop/recycling-robot/docker-compose.yml up --build -d
xdg-open http://localhost:5173
ros2 launch recycling_robot robot.launch.py  # inside ROS2 container/shell