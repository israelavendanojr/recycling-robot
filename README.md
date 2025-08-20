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


Camera Node

Runs the camera or mock camera.

Publishes images to the topic /camera/image_raw.

Has parameters like resolution, frame rate, and camera type.

Classifier Node

Loads a trained PyTorch model (recycler.pth).

Provides a ROS service /classify_image → you can send it an image, it replies with what material it sees (cardboard, glass, plastic, etc.).

Web Bridge Node

Starts a small web server (Flask).

Serves a dashboard at http://localhost:8000.

Streams video frames and shows classification stats in the browser.



