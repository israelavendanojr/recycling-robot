# Raspberry Pi 5 + ArduCam IMX708 Camera Setup Guide

This guide provides a complete solution for setting up your ArduCam IMX708 camera on Raspberry Pi 5 for your robotics recycling project.

## 🎯 Quick Start

1. **Run the quick test:**
   ```bash
   ./test_camera.sh
   ```

2. **If issues are found, run the comprehensive diagnostic:**
   ```bash
   ./camera_diagnostic.sh
   ```

3. **Start the camera stream:**
   ```bash
   ./start_camera_stream.sh start
   ```

4. **Test in your browser:**
   ```
   http://<your-pi-ip>:8554/feed.mjpg
   ```

## 📋 Scripts Overview

### `test_camera.sh` - Quick Health Check
- Fast system verification
- Basic camera detection
- Dependency check
- Quick capture test

### `camera_diagnostic.sh` - Comprehensive Diagnostic
- Detailed hardware analysis
- Multiple capture method testing
- Automatic issue fixing
- Detailed logging and recommendations

### `start_camera_stream.sh` - Production Streaming
- Automatic method selection (libcamera → V4L2 fallback)
- Multiple format support
- HTTP MJPEG streaming
- Process management and logging

## 🔧 Installation & Setup

### 1. Install Dependencies
```bash
# Install required packages
sudo apt update
sudo apt install -y ffmpeg libcamera-apps v4l-utils curl

# Or use the script
./start_camera_stream.sh install
```

### 2. Enable Camera in Config
Add to `/boot/firmware/config.txt`:
```
camera_auto_detect=1
# For IMX708 specifically:
dtoverlay=imx708
```

### 3. Reboot
```bash
sudo reboot
```

## 🎥 Streaming Methods

### Method 1: libcamera-vid (Recommended for Pi 5)
- **Best compatibility** with Raspberry Pi 5
- **Native support** for IMX708
- **Lower latency** than V4L2
- **Automatic format detection**

### Method 2: V4L2 + FFmpeg (Fallback)
- **Universal compatibility**
- **Multiple format support** (MJPEG, YUYV, etc.)
- **Higher latency** but more reliable
- **Used when libcamera fails**

## 🚀 Usage Examples

### Basic Streaming
```bash
# Start with defaults (640x480, 10fps, port 8554)
./start_camera_stream.sh start

# Custom settings
./start_camera_stream.sh start 8080 1280 720 15 90
#                    port  width height fps quality
```

### Stream Management
```bash
# Check status
./start_camera_stream.sh status

# Test stream
./start_camera_stream.sh test

# Stop stream
./start_camera_stream.sh stop

# Restart stream
./start_camera_stream.sh restart
```

### Diagnostics
```bash
# Quick test
./test_camera.sh

# Comprehensive diagnostic
./camera_diagnostic.sh
```

## 🔍 Troubleshooting

### Common Issues

#### 1. "No video devices found"
**Symptoms:** No `/dev/video*` devices
**Solutions:**
- Check physical connection (CSI cable orientation)
- Ensure camera is enabled in config.txt
- Reboot the Pi
- Check if camera is detected: `vcgencmd get_camera`

#### 2. "libcamera-vid failed"
**Symptoms:** libcamera tools not working
**Solutions:**
- Install libcamera-apps: `sudo apt install libcamera-apps`
- Check camera compatibility with Pi 5
- Try V4L2 fallback method

#### 3. "FFmpeg Invalid argument"
**Symptoms:** V4L2 streaming fails
**Solutions:**
- Check device capabilities: `v4l2-ctl --device=/dev/video0 --list-formats-ext`
- Try different input formats (mjpeg, yuyv422, yuv420p)
- Use libcamera-vid instead

#### 4. "Permission denied"
**Symptoms:** Cannot access video devices
**Solutions:**
- Add user to video group: `sudo usermod -a -G video $USER`
- Log out and back in
- Set device permissions: `sudo chmod 666 /dev/video*`

### Debug Commands

```bash
# Check camera detection
vcgencmd get_camera

# List video devices
ls -la /dev/video*

# Check device capabilities
v4l2-ctl --device=/dev/video0 --list-formats-ext

# Test libcamera
libcamera-hello --list-cameras

# Check kernel messages
dmesg | grep -i camera

# Monitor stream logs
tail -f /tmp/camera_stream.log
```

## 🌐 Integration with Your Project

### For Your React Dashboard
The stream will be available at:
```
http://<pi-ip>:8554/feed.mjpg
```

**Example React component:**
```jsx
function CameraFeed() {
  const piIP = '192.168.1.100'; // Your Pi's IP
  const streamUrl = `http://${piIP}:8554/feed.mjpg`;
  
  return (
    <div>
      <h3>Live Camera Feed</h3>
      <img 
        src={streamUrl} 
        alt="Camera Feed"
        style={{ width: '100%', maxWidth: '640px' }}
      />
    </div>
  );
}
```

### For Your ROS2 Docker Setup
In your Docker container, access the stream via:
```
http://host.docker.internal:8554/feed.mjpg
```

**Docker Compose example:**
```yaml
version: '3.8'
services:
  ros2-node:
    image: your-ros2-image
    network_mode: host
    environment:
      - CAMERA_STREAM_URL=http://localhost:8554/feed.mjpg
```

### For Your API
The MJPEG stream can be consumed by:
- **OpenCV**: `cv2.VideoCapture('http://pi-ip:8554/feed.mjpg')`
- **FFmpeg**: Direct HTTP input
- **Web browsers**: Native MJPEG support
- **Image processing libraries**: PIL, scikit-image, etc.

## 📊 Performance Optimization

### Recommended Settings by Use Case

#### Low Latency (Robotics Control)
```bash
./start_camera_stream.sh start 8554 640 480 15 70
# Smaller resolution, higher fps, lower quality
```

#### High Quality (Object Detection)
```bash
./start_camera_stream.sh start 8554 1280 720 10 90
# Higher resolution, lower fps, higher quality
```

#### Balanced (General Use)
```bash
./start_camera_stream.sh start 8554 800 600 12 80
# Medium resolution, balanced fps and quality
```

### Network Considerations
- **Local network**: MJPEG over HTTP works well
- **Remote access**: Consider H.264 + WebRTC for better compression
- **Multiple clients**: The stream supports multiple simultaneous viewers

## 🔄 Advanced Configuration

### Custom FFmpeg Parameters
Edit `start_camera_stream.sh` to modify FFmpeg settings:
```bash
# For H.264 streaming instead of MJPEG
-c:v libx264 -preset ultrafast -tune zerolatency

# For different output formats
-f mp4 -movflags frag_keyframe+empty_moov
```

### Systemd Service
Create a persistent service:
```bash
# Create service file
sudo tee /etc/systemd/system/camera-stream.service > /dev/null << EOF
[Unit]
Description=Camera Stream Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/recycling-robot
ExecStart=/home/pi/recycling-robot/start_camera_stream.sh start
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
sudo systemctl enable camera-stream.service
sudo systemctl start camera-stream.service
```

## 📝 Log Files

- **Stream logs**: `/tmp/camera_stream.log`
- **Diagnostic logs**: `/tmp/camera_diagnostic.log`
- **Process ID**: `/tmp/camera_stream.pid`

## 🆘 Getting Help

1. **Run diagnostics first**: `./camera_diagnostic.sh`
2. **Check logs**: `tail -f /tmp/camera_stream.log`
3. **Verify hardware**: `vcgencmd get_camera`
4. **Test connectivity**: `curl http://localhost:8554/feed.mjpg`

## 🎯 Next Steps for Your Project

1. **Get the basic stream working** with these scripts
2. **Test in your browser** to confirm functionality
3. **Integrate with your React dashboard** using the MJPEG URL
4. **Connect to your ROS2 nodes** via HTTP stream
5. **Optimize settings** for your specific use case
6. **Consider WebRTC** for remote access if needed

The MJPEG over HTTP approach is perfect for your current needs and will integrate seamlessly with your React dashboard and ROS2 setup!
