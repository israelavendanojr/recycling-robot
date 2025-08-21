# Recycling Robot
Quick Self-Check Rubric 

End-to-End Proof: I can place an object → see it in live view → see a classification event → press Pause/Resume → see bin counters change.

Non-Blocking: No while True loops inside ROS callbacks; inference queue bounded; DB writes batched.

Contracts Written: Topics/actions + message fields are in the repo (docs or .msg/.action).

Pi-Safe Defaults: SQLite WAL on; MJPEG preview; telemetry downsampled; CPU temp visible.

Repro: ros2 launch ... + one docker compose up path for API/dashboard.


docker compose run --rm recycling-robot bash

rm -rf build/ install/ log/
colcon build --packages-select recycling_robot --symlink-install
source install/setup.bash

ros2 launch recycling_robot minimal.launch.py


rm -rf build/ install/ log/ core


# Build and run
docker compose up --build

# Or run in background
docker compose up -d --build

# View logs
docker compose logs -f