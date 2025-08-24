# Recycling Robot - ROS2 + Raspberry Pi 5 + Docker

AI-powered waste classification & sorting system with real-time dashboard.

## 🏗️ Architecture

```
Camera (or Mock) → Classifier (PyTorch) → Sorting → Flask/SQLite → React Dashboard
```

- **ROS2 Nodes**: Camera, Classifier, Sorting, Web Dashboard
- **Backend**: Flask API with SQLite database
- **Frontend**: React dashboard with real-time updates
- **AI**: PyTorch model for waste classification

## 🚀 Quick Start

### 1. Start the System

```bash
# Start all services
docker-compose up -d

# Check service status
docker-compose ps
```

### 2. Launch ROS2 Pipeline

```bash
# Enter ROS2 container
docker-compose exec ros2 bash

# Source ROS2 workspace
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with mock camera (recommended for testing)
ros2 launch recycling_robot robot.launch.py use_mock_camera:=true

# Or launch with real camera
ros2 launch recycling_robot robot.launch.py use_mock_camera:=false
```

### 3. Access Dashboard

- **Frontend**: http://localhost:5173
- **Backend API**: http://localhost:8000
- **ROS2 Web Node**: http://localhost:8080

## 🔍 Verifying Data Flow

### Frontend Data Flow Verification

The React frontend should now be receiving and displaying:

1. **Classification Logs** from SQLite database
2. **Current Images** from camera (mock or live)

#### Console Logs to Look For

Open browser DevTools (F12) and check the Console tab:

```
[API] Fetching events from: http://localhost:8000/api/classifications
[API] Received classifications response: {success: true, count: 5, classifications: [...]}
[API] Successfully fetched 5 classifications
[ClassificationLog] Events updated: 0 → 5
[ClassificationLog] Latest event: plastic (82%) at 14:30:25
[VideoFeed] Fetching current image...
[VideoFeed] Successfully loaded image: {image_url: "...", source: "mock_camera"}
```

#### Visual Verification

1. **Classification Log**: Should show real-time classification results
2. **Video Feed**: Should display current image (mock camera for testing)
3. **Counters**: Should show material counts
4. **Status**: Should show "System Online" and "Live Updates"

### Backend API Verification

Test the Flask endpoints:

```bash
# Check health
curl http://localhost:8000/api/health

# Get classifications
curl http://localhost:8000/api/classifications

# Get current image info
curl http://localhost:8000/api/current_image
```

Expected responses:
- `/api/health`: System status and service health
- `/api/classifications`: Array of classification results
- `/api/current_image`: Current image URL and metadata

### ROS2 Pipeline Verification

#### Terminal Output

With cleaned-up logging, you should see:

```
🚀 Mock camera node started
📁 Image folder: test_images
⏱️  Publish rate: 3.0s
📊 Found 5 test images

🚀 Classifier node started
📱 API endpoint: http://backend:8000
💻 Using device: cpu
⏳ Waiting for backend to be ready...
✅ Backend is ready!
✅ Backend ready, classification service active

🚀 Sorting node started
⏱️  Sorting delay: 1.0s
📋 Available sorting actions:
  🔵 cardboard → Bin 1 (Blue)
  🟢 glass → Bin 2 (Green)
  🟡 metal → Bin 3 (Yellow)
  🔴 plastic → Bin 4 (Red)
  ⚫ trash → Bin 5 (Black)

🌐 Web dashboard starting on 0.0.0.0:8080
📊 API endpoints: /api/classifications, /api/classifications/latest
📹 Video feed: /video_feed
```

#### Classification Events

When the system is running, you should see:

```
📸 Published test image: test_plastic.jpg
🎯 Predicted: plastic (0.82 confidence)
📥 Received: plastic (82.0% confidence)
🎯 SORTING: PLASTIC → BIN_4
   📊 Confidence: 82.0%
   🔧 Action: Move to Red Bin (Plastic)
✅ Sorting completed: plastic → BIN_4
```

## 🛠️ Development Workflow

### Frontend Development

```bash
# Development mode (hot reload)
cd frontend
npm run dev

# Production build
npm run build
```

**Important**: Use `npm run dev` during development for hot reloading. Use `npm run build` for production builds.

### Backend Development

```bash
# Flask development server
cd backend
python -m flask --app api/app.py run --host=0.0.0.0 --port=8000 --debug
```

### ROS2 Development

```bash
# Build ROS2 package
docker-compose exec ros2 bash
cd /workspace/ros2_ws
colcon build --packages-select recycling_robot

# Source and test individual nodes
source install/setup.bash
ros2 run recycling_robot mock_camera_node
```

## 🔧 Troubleshooting

### Frontend Not Updating

1. **Check console logs** for API errors
2. **Verify backend is running**: `curl http://localhost:8000/api/health`
3. **Check CORS**: Ensure Flask CORS is enabled
4. **Restart frontend**: `npm run dev`

### No Classifications Showing

1. **Check ROS2 nodes**: Ensure classifier is running
2. **Check database**: Verify SQLite files exist
3. **Check API endpoints**: Test `/api/classifications`
4. **Check console logs** for error messages

### Camera Issues

1. **Mock camera**: Use `use_mock_camera:=true` for testing
2. **Real camera**: Check `/dev/video0` permissions
3. **Image format**: Ensure JPEG compression is working

### Logging Too Verbose

All nodes now use INFO level logging by default. To see more details:

```python
# In any ROS2 node
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

## 📊 Monitoring

### Real-time Dashboard Features

- **Live camera feed** (mock or real)
- **Classification log** with confidence scores
- **Material counters** by type
- **System health** status
- **Performance metrics**

### Data Flow Confirmation

1. **Camera → Classifier**: Check ROS2 topic `/camera/image_raw`
2. **Classifier → Database**: Check SQLite file `~/classifications.db`
3. **Database → Flask**: Check API responses
4. **Flask → React**: Check browser console logs
5. **React → Display**: Check dashboard updates

## 🚀 Production Deployment

### Docker Production

```bash
# Build production images
docker-compose -f docker-compose.prod.yml up -d

# Use real camera
ros2 launch recycling_robot robot.launch.py use_mock_camera:=false
```

### Raspberry Pi 5 Specific

- **Camera**: Use USB camera or Pi Camera Module
- **Performance**: Monitor CPU/GPU usage
- **Storage**: Ensure sufficient space for database
- **Network**: Stable connection for dashboard access

## 📝 API Reference

### Endpoints

- `GET /api/health` - System health status
- `GET /api/classifications` - Classification history
- `GET /api/classifications/latest` - Most recent classification
- `GET /api/current_image` - Current image info
- `GET /api/counters` - Material counts
- `POST /api/classifier/start` - Start classifier
- `POST /api/classifier/stop` - Stop classifier

### Data Formats

See `frontend/src/api/client.ts` for TypeScript interfaces and example usage.

## 🎯 Success Criteria

The system is working correctly when:

1. ✅ **Frontend loads** without errors
2. ✅ **Classification log updates** in real-time
3. ✅ **Images display** (mock or live camera)
4. ✅ **Console shows** successful API calls
5. ✅ **Terminal shows** clean, focused ROS2 logs
6. ✅ **Full pipeline** from camera to dashboard works

## 📞 Support

For issues or questions:
1. Check console logs (frontend + backend)
2. Check ROS2 terminal output
3. Verify all services are running
4. Test API endpoints individually
5. Check database connectivity

---

**Happy Recycling! ♻️**

