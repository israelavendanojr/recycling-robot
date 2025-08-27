# Recycling Robot with Synchronous Pipeline

A ROS2-based recycling robot that automatically classifies and sorts recyclable materials using computer vision and motor control, with a **synchronous pipeline** that eliminates race conditions.

## 🎯 Features

- **Computer Vision Classification**: Uses PyTorch to classify materials (cardboard, glass, metal, plastic, trash)
- **Motor Control**: Automated sorting with motor-driven actuators
- **Synchronous Pipeline**: Eliminates race conditions with coordinated processing
- **Web Interface**: Real-time monitoring dashboard with pipeline status
- **Docker Integration**: Containerized development environment

## 🚀 Quick Start

### 1. Install Dependencies
```bash
make install
```

### 2. Start Services
```bash
make up
```

### 3. Launch ROS2 Robot
```bash
make launch-robot
```

## 🧪 Testing the Synchronous Pipeline

### Quick Test
```bash
./quick_pipeline_test.sh
```

### Comprehensive Testing
```bash
# Test pipeline functionality
make test-pipeline

# Monitor pipeline state in real-time
make monitor-pipeline

# Run manual test with simulation
make test-pipeline-manual

# Verify pipeline components
make verify-pipeline
```

## 📊 Pipeline Monitoring

The synchronous pipeline ensures:
- **No Race Conditions**: Only one item processes at a time
- **Physical Consistency**: Motor completes before next classification
- **Real-time Status**: Web interface shows current pipeline state
- **Automatic Recovery**: Timeout protection and error handling

### Expected Output During Launch
```
[LAUNCH] 🚀 Starting Recycling Robot with Synchronous Pipeline...
[LAUNCH] 🔄 Starting Pipeline Coordinator...
🟢 PIPELINE: READY FOR NEXT ITEM
[LAUNCH] 📷 Starting Camera and Processing Nodes...
[VERIFY] 🔍 Checking Pipeline Topics...
[VERIFY] 📊 Current Pipeline State:
🟢 PIPELINE: READY FOR NEXT ITEM
```

## 🏗️ Architecture

```
Camera → Classifier → Sorting → Complete → Next Item
   ↓         ↓         ↓         ↓
  Wait    Process   Move Motor  Notify
  (if     (if       (if        Coordinator
   busy)   idle)     busy)
```

## 📁 Project Structure

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
├── quick_pipeline_test.sh           # Quick pipeline test
└── test_pipeline_comprehensive.py   # Comprehensive test script
```

## 🔧 Development

### Building the Package
```bash
make build
```

### Monitoring Pipeline State
```bash
# In ROS2 container
ros2 topic echo /pipeline/state

# Via web interface
# Open browser and check Pipeline Status component
```

### Testing Individual Components
```bash
# Test pipeline coordinator
ros2 run recycling_robot pipeline_coordinator_node

# Test with mock camera
ros2 launch recycling_robot robot.launch.py

# Test with real camera
ros2 launch recycling_robot robot.launch.py use_real_camera:=true
```

## 📈 Pipeline States

- **🟢 IDLE**: Ready for new item, cameras can publish, classifier can process
- **🔴 PROCESSING**: Item being sorted, cameras wait, classifier waits
- **⏸️ WAITING**: Intermediate state during transitions

## 🎯 Success Criteria

When the pipeline is working correctly, you should see:
- ✅ Pipeline coordinator starts first
- ✅ State transitions: idle → processing → idle
- ✅ Camera nodes respect pipeline state
- ✅ Classifier waits when pipeline busy
- ✅ Motor operations complete before next item
- ✅ No race conditions during rapid item presentation
- ✅ Web interface shows real-time pipeline status

## 🐛 Troubleshooting

### Common Issues
1. **Pipeline stuck in "processing" state**
   - Check sorting node logs for motor errors
   - Pipeline auto-resets after 10s timeout

2. **No images being published**
   - Check pipeline state: `ros2 topic echo /pipeline/state`
   - If "processing", wait for sorting to complete

3. **Classifier not processing**
   - Verify pipeline state
   - Check classifier logs for errors

### Debug Commands
```bash
# Check pipeline topics
ros2 topic list | grep pipeline

# Monitor message flow
ros2 topic echo /pipeline/state &
ros2 topic echo /pipeline/classification_done &
ros2 topic echo /pipeline/sorting_done &

# Check node status
ros2 node list
ros2 node info /pipeline_coordinator_node
```

## 📚 Documentation

- [PIPELINE_README.md](PIPELINE_README.md) - Detailed pipeline implementation guide
- [Makefile](Makefile) - Available commands and testing options

## 🤝 Contributing

This is a beginner-friendly robotics project. All code follows ROS2 best practices and includes clear comments for maintainability.

## 📄 License

Apache License 2.0