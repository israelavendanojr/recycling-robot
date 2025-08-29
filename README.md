# Recycling Robot with Synchronous Pipeline

A ROS2-based recycling robot that automatically classifies and sorts recyclable materials using computer vision, motor control, and a frontend dashboard

## Quick Start

### 1. Complete Launch
```bash
make run
```

### 2. Only Launch Robot
```bash
make launch-robot
```

### Dashboard
```bash
http://localhost:5173
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
│   │   ├── pipeline_coordinator_node.py  # Pipeline State Manager
│   │   ├── camera_node.py            # Live camera
│   │   ├── classifier_node.py        # TorchScript classifier
│   │   └── sorting_node.py           # Motor control
│   ├── launch/                       # Launch files
│   └── setup.py                      # Package configuration
├── backend/                          # Flask API
├── web/                             # React dashboard / controller
├── Makefile                         # Build and test commands

```

### Add to CI/CD Later
- Docker image building
- Integration tests with hardware
- Actual CD + Deployment steps