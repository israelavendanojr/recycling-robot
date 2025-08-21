# Orchestration Scripts

Two helpers to start/stop the camera stream and ROS2 stack together.

## Quick Start

```bash
# Start camera + ROS2 (waits for stream first)
cd backend/scripts
chmod +x robot_up.sh robot_down.sh
./robot_up.sh

# Stop ROS2 + camera
./robot_down.sh
```

By default, `robot_up.sh` checks these in order:
- `start_camera_stream.sh start`
- `working_camera_stream.sh`
- `python3 browser_stream.py 8554`

It then waits until the stream is reachable at `http://localhost:8554/feed.mjpg` before starting Docker `docker compose up -d`.

If your endpoint uses `/stream.mjpg`, either:
- change `STREAM_URL` in `robot_up.sh`, or
- update `start_camera_stream.sh` to serve `/feed.mjpg`.

## Systemd (Optional)

Create a unit to auto-start on boot after network:

```
# /etc/systemd/system/recycling-robot.service
[Unit]
Description=Recycling Robot Camera + ROS2
After=network-online.target docker.service
Wants=network-online.target docker.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/Desktop/recycling-robot/backend/scripts
ExecStart=/bin/bash -lc './robot_up.sh'
ExecStop=/bin/bash -lc './robot_down.sh'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl enable recycling-robot
sudo systemctl start recycling-robot
# logs
journalctl -u recycling-robot -f
```

## Notes
- The ROS2 container expects the stream URL in `backend/src/recycling_robot/config/camera.yaml` using `http://host.docker.internal:8554/…`.
- `robot_up.sh` only waits on `localhost:8554` to avoid false negatives if hostnames aren’t resolvable.
- You can hardcode a different host/port if you move the camera service.
