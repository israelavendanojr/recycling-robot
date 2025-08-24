# 🧪 Recycling Robot Verification Checklist

Use this checklist to verify that your entire system is working correctly from end-to-end.

## 📋 Pre-flight Checklist

- [ ] Docker containers are running (`docker-compose ps`)
- [ ] All services show "Up" status
- [ ] Ports 8000, 5173, and 8080 are accessible

## 🚀 Step 1: Start the System

### 1.1 Start Docker Services
```bash
docker-compose up -d
docker-compose ps
```

**Expected**: All services show "Up" status

### 1.2 Verify Backend API
```bash
# Test health endpoint
curl http://localhost:8000/api/health

# Test classifications endpoint  
curl http://localhost:8000/api/classifications

# Test current image endpoint
curl http://localhost:8000/api/current_image
```

**Expected**: All endpoints return 200 status with JSON data

### 1.3 Run API Test Script
```bash
cd scripts
python test_api.py
```

**Expected**: All tests pass with ✅ status

## 🎯 Step 2: Launch ROS2 Pipeline

### 2.1 Enter ROS2 Container
```bash
docker-compose exec ros2 bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 2.2 Build and Launch
```bash
# Build the package
colcon build --packages-select recycling_robot

# Launch with mock camera (recommended for testing)
ros2 launch recycling_robot robot.launch.py use_mock_camera:=true
```

**Expected**: Clean startup logs with emojis and clear status messages

### 2.3 Verify ROS2 Nodes
```bash
# In another terminal, enter the container
docker-compose exec ros2 bash
cd /workspace/ros2_ws
source install/setup.bash

# Run verification script
python scripts/verify_ros2.py
```

**Expected**: All nodes, topics, and data flow checks pass

## 🌐 Step 3: Test Frontend Dashboard

### 3.1 Open Dashboard
- Navigate to: http://localhost:5173
- Open browser DevTools (F12)
- Go to Console tab

### 3.2 Check Console Logs
Look for these successful API calls:

```
[API] Fetching events from: http://localhost:8000/api/classifications
[API] Successfully fetched X classifications
[VideoFeed] Successfully loaded image: {...}
[ClassificationLog] Events updated: 0 → X
```

**Expected**: No red error messages, all API calls successful

### 3.3 Visual Verification
- [ ] Dashboard loads without errors
- [ ] Video feed shows image (mock camera for testing)
- [ ] Classification log shows real-time updates
- [ ] Counters display material counts
- [ ] Status shows "System Online" and "Live Updates"

## 🔄 Step 4: Verify Data Flow

### 4.1 Monitor ROS2 Terminal
Watch for classification events:

```
📸 Published test image: test_plastic.jpg
🎯 Predicted: plastic (0.82 confidence)
📥 Received: plastic (82.0% confidence)
🎯 SORTING: PLASTIC → BIN_4
✅ Sorting completed: plastic → BIN_4
```

**Expected**: Regular classification events every 3-5 seconds

### 4.2 Check Database
```bash
# In ROS2 container
sqlite3 ~/classifications.db "SELECT COUNT(*) FROM classifications;"
sqlite3 ~/classifications.db "SELECT * FROM classifications ORDER BY created_at DESC LIMIT 5;"
```

**Expected**: Database contains classification records

### 4.3 Verify API Updates
```bash
# Check classifications endpoint
curl http://localhost:8000/api/classifications | jq '.count'

# Check latest classification
curl http://localhost:8000/api/classifications/latest | jq '.classification.label'
```

**Expected**: Count increases over time, latest classification shows recent results

## ✅ Step 5: Success Criteria

Your system is working correctly when:

### 5.1 Frontend ✅
- [ ] Dashboard loads without errors
- [ ] Classification log updates in real-time
- [ ] Images display (mock or live camera)
- [ ] Console shows successful API calls
- [ ] No JavaScript errors in console

### 5.2 Backend ✅
- [ ] All API endpoints return 200 status
- [ ] `/api/classifications` returns classification data
- [ ] `/api/current_image` returns image info
- [ ] Database contains classification records

### 5.3 ROS2 Pipeline ✅
- [ ] All nodes start successfully
- [ ] Camera publishes images to `/camera/image_raw`
- [ ] Classifier processes images and publishes results
- [ ] Sorting node receives and processes classifications
- [ ] Terminal shows clean, focused logs

### 5.4 Data Flow ✅
- [ ] Camera → Classifier: Images flowing
- [ ] Classifier → Database: Results stored
- [ ] Database → Flask: Data accessible via API
- [ ] Flask → React: Frontend receives updates
- [ ] React → Display: Dashboard shows real-time data

## 🚨 Troubleshooting

### Frontend Issues
- **Not loading**: Check if `npm run dev` is running
- **No updates**: Check console for API errors
- **CORS errors**: Verify Flask CORS is enabled

### Backend Issues
- **API not responding**: Check if Flask service is running
- **Database errors**: Verify SQLite file permissions
- **Port conflicts**: Check if port 8000 is available

### ROS2 Issues
- **Nodes not starting**: Check workspace is sourced
- **No data flow**: Verify topics are active
- **Verbose logging**: All nodes now use INFO level by default

### Camera Issues
- **Mock camera**: Use `use_mock_camera:=true` for testing
- **Real camera**: Check `/dev/video0` permissions
- **No images**: Verify image compression is working

## 🎯 Final Verification

Run this command to test everything at once:

```bash
# Test API
python scripts/test_api.py

# Test ROS2 (in container)
python scripts/verify_ros2.py

# Check dashboard
open http://localhost:5173
```

**Expected Result**: All systems green ✅, dashboard showing live data, ROS2 pipeline running smoothly.

---

## 🏆 Success!

If you've completed all steps and see:
- ✅ All API tests passing
- ✅ ROS2 nodes running with clean logs
- ✅ Dashboard showing real-time updates
- ✅ Classification events flowing through the system

**Congratulations! Your recycling robot is fully operational! 🎉**

The full loop is working:
**Camera (Mock) → Classifier (PyTorch) → Sorting → SQLite → Flask → React → Browser Display**


