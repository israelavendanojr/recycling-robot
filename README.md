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



Dashboard & API

- Dashboard: http://localhost:5173
- Backend API base: http://localhost:8000
- MJPEG video (if web_node serves it): http://localhost:8080/video_feed

Docker Compose tips

- Bring services up:
  docker compose -f /home/israelavendanojr/Desktop/recycling-robot/docker-compose.yml up --build -d
- If you see orphan containers (e.g., old camera-stream):
  docker compose down --remove-orphans


Commands to run
docker compose down --remove-orphans
docker compose up --build -d
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch recycling_robot robot.launch.py"