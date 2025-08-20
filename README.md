# Recycling Robot
Quick Self-Check Rubric 

End-to-End Proof: I can place an object → see it in live view → see a classification event → press Pause/Resume → see bin counters change.

Non-Blocking: No while True loops inside ROS callbacks; inference queue bounded; DB writes batched.

Contracts Written: Topics/actions + message fields are in the repo (docs or .msg/.action).

Pi-Safe Defaults: SQLite WAL on; MJPEG preview; telemetry downsampled; CPU temp visible.

Repro: ros2 launch ... + one docker compose up path for API/dashboard.