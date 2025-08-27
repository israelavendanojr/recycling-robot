# Synchronous Pipeline Implementation

## Overview

This implementation adds a **synchronous pipeline coordinator** to eliminate race conditions in the recycling robot. The pipeline ensures that only one item is processed at a time, maintaining physical consistency.

## Architecture

```
Camera → Classifier → Sorting → Complete → Next Item
   ↓         ↓         ↓         ↓
  Wait    Process   Move Motor  Notify
  (if     (if       (if        Coordinator
   busy)   idle)     busy)
```

## Key Components

### 1. Pipeline Coordinator Node (`pipeline_coordinator_node.py`)
- **Purpose**: Manages pipeline state and coordinates between nodes
- **States**: `idle` (ready for new item) and `processing` (item being sorted)
- **Topics**:
  - Publishes: `/pipeline/state` (current state)
  - Subscribes: `/pipeline/classification_done`, `/pipeline/sorting_done`
- **Features**: Timeout protection (10s), state persistence, logging

### 2. Modified Nodes

#### Classifier Node
- **New**: Subscribes to `/pipeline/state`
- **New**: Publishes to `/pipeline/classification_done` when done
- **Behavior**: Skips classification if pipeline is busy

#### Sorting Node  
- **New**: Publishes to `/pipeline/sorting_done` when motor completes
- **Behavior**: Processes one item at a time

#### Camera Nodes (Real & Mock)
- **New**: Subscribe to `/pipeline/state`
- **Behavior**: Skip publishing new images if pipeline is busy

## How It Works

1. **Idle State**: Camera can publish images, classifier can process
2. **Classification**: When classifier completes, notifies coordinator
3. **Processing State**: Coordinator transitions to "processing", cameras stop publishing
4. **Sorting**: Sorting node processes item and controls motor
5. **Completion**: When sorting done, notifies coordinator
6. **Return to Idle**: Coordinator returns to "idle" state

## Installation & Usage

### 1. Build the Package
```bash
cd ros2
colcon build --packages-select recycling_robot
source install/setup.bash
```

### 2. Launch the Robot
```bash
# Launch with mock camera (default)
ros2 launch recycling_robot robot.launch.py

# Launch with real camera
ros2 launch recycling_robot robot.launch.py use_real_camera:=true
```

### 3. Monitor Pipeline State
```bash
# Check pipeline state topic
ros2 topic echo /pipeline/state

# Check pipeline state via backend API
curl http://localhost:8000/api/pipeline/state
```

### 4. Test the Pipeline
```bash
# Run the test script to simulate message flow
python3 test_pipeline.py
```

## Web Interface

The dashboard now includes a **Pipeline Status** component that shows:
- Current state (Ready/Processing Item...)
- Current item details (material, confidence)
- Last update time
- State change timestamp
- Manual refresh button

## Configuration

### Pipeline Coordinator Parameters
- `timeout_seconds`: Maximum time for processing (default: 10s)
- `state_file_path`: File to persist state for backend access

### Launch File Parameters
```python
# In robot.launch.py
Node(
    package='recycling_robot',
    executable='pipeline_coordinator_node',
    name='pipeline_coordinator_node',
    parameters=[{
        'timeout_seconds': 10.0,
        'state_file_path': '/tmp/pipeline_state.json'
    }]
)
```

## Troubleshooting

### Common Issues

1. **Pipeline stuck in "processing" state**
   - Check if sorting node is running
   - Look for motor errors in sorting node logs
   - Pipeline will auto-reset after timeout (10s)

2. **No images being published**
   - Check pipeline state: `ros2 topic echo /pipeline/state`
   - If "processing", wait for sorting to complete
   - Verify camera nodes are running

3. **Classifier not processing**
   - Check pipeline state
   - Verify classifier node is subscribed to `/pipeline/state`
   - Check classifier logs for errors

### Debug Commands

```bash
# Check all pipeline topics
ros2 topic list | grep pipeline

# Monitor message flow
ros2 topic echo /pipeline/state &
ros2 topic echo /pipeline/classification_done &
ros2 topic echo /pipeline/sorting_done &

# Check node status
ros2 node list
ros2 node info /pipeline_coordinator_node
```

## Benefits

✅ **Eliminates Race Conditions**: Only one item processes at a time
✅ **Physical Consistency**: Motor completes before next classification
✅ **Observable State**: Web interface shows current status
✅ **Timeout Protection**: Auto-recovery from stuck states
✅ **Maintainable**: Simple 2-state machine, clear logging
✅ **ROS2 Compliant**: Uses standard message types and patterns

## Future Enhancements

- **Priority Queue**: Handle multiple items waiting
- **Error Recovery**: Automatic retry mechanisms
- **Performance Metrics**: Track processing times
- **Configuration UI**: Web-based parameter adjustment

## Code Quality

- **Total New Code**: ~400 lines
- **Coordinator Node**: 80 lines
- **Node Modifications**: <20 lines each
- **Web Component**: 100 lines
- **Backend Changes**: 30 lines

All code follows existing patterns, includes proper error handling, and maintains beginner-friendly readability.
