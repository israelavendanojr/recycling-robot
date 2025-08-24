BUILD ORDER: 

Frontend: 
cd ../frontend
npm run build

Backend: 
cd ../backend
docker compose up --build -d

ROS: 
docker compose down --remove-orphans
docker compose up --build -d
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch recycling_robot robot.launch.py"
xdg-open http://localhost:5173



Dashboard & API
- Dashboard: http://localhost:5173
- Backend API base: http://localhost:8000
- MJPEG video (if web_node serves it): http://localhost:8080/video_feed

