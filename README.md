# Recycling Robot with Synchronous Pipeline

A ROS2-based recycling robot that automatically classifies and sorts recyclable materials using computer vision, motor control, and a frontend dashboard

## Quick Start

### 1. First Boot
```bash
make run
```

### 3. Launch Robot
```bash
make launch-robot
```


## Architecture

```
Camera → Classifier → Sorting → Complete → Next Item
   ↓         ↓         ↓         ↓
  Wait    Process   Move Motor  Notify
  (if     (if       (if        Coordinator
   busy)   idle)     busy)
```

## Project Structure

```
recycling-robot/
├── ros2/src/recycling_robot/          # ROS2 package
│   ├── nodes/                         # ROS2 nodes
│   │   ├── pipeline_coordinator_node.py  # Pipeline coordinator
│   │   ├── camera_node.py            # Real camera
│   │   ├── mock_camera_node.py       # Mock camera
│   │   ├── classifier_node.py        # ML classifier
│   │   └── sorting_node.py           # Motor control
│   ├── launch/                       # Launch files
│   └── setup.py                      # Package configuration
├── backend/                          # Flask API
├── web/                             # React frontend
├── Makefile                         # Build and test commands

```
