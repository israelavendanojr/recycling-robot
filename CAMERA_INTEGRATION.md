# Camera Integration Guide

## Overview

This robotics dashboard now includes live camera integration using the browser's `getUserMedia()` API, specifically designed to work with the Logitech C270 webcam. The system provides both live camera feed and backend API modes for flexibility.

## Features

### 🎥 Live Camera Mode
- **Direct Webcam Access**: Uses `navigator.mediaDevices.getUserMedia()` for real-time video
- **Device Selection**: Automatically detects and allows selection of available camera devices
- **HD Resolution**: Supports up to 1280x720 resolution with 30fps
- **Error Handling**: Comprehensive error handling for permissions, hardware issues, and API limitations
- **Fallback Support**: Automatically falls back to default camera settings if high-resolution fails

### 🔄 Dual Mode Operation
- **Live Camera**: Direct browser-to-camera communication
- **Backend API**: Server-side image processing (existing functionality)
- **Easy Toggle**: Switch between modes with a simple button interface

### 📊 System Status Integration
- **Real-time Monitoring**: Camera status is integrated with system health monitoring
- **Visual Indicators**: Color-coded status indicators (green=active, red=error, yellow=inactive)
- **Component Health**: Camera, Database, and ROS2 status are all monitored and displayed

## Technical Implementation

### Camera API Usage

```typescript
// Camera constraints for Logitech C270
const constraints = {
  video: {
    deviceId: { exact: selectedDevice },
    width: { ideal: 1280 },
    height: { ideal: 720 },
    frameRate: { ideal: 30 }
  },
  audio: false
};

// Start camera stream
const stream = await navigator.mediaDevices.getUserMedia(constraints);
```

### Error Handling

The system handles various camera-related errors:

- **NotAllowedError**: Camera permissions denied
- **NotFoundError**: Camera device not found
- **NotReadableError**: Camera in use by another application
- **OverconstrainedError**: Resolution not supported (with fallback)

### Status Propagation

Camera status changes are propagated through the component hierarchy:

```typescript
// LiveCamera → VideoFeed → Dashboard → SystemStatus
onCameraStatusChange?.(status: 'active' | 'inactive' | 'error')
```

## Usage Instructions

### 1. Enable Camera Permissions
- When first accessing the dashboard, your browser will request camera permissions
- Click "Allow" to enable camera access
- If denied, refresh the page and try again

### 2. Select Camera Mode
- **Live Camera**: For real-time webcam feed (recommended for Logitech C270)
- **Backend API**: For server-processed images (fallback mode)

### 3. Camera Controls
- **Start Camera**: Begin live video stream
- **Stop Camera**: Stop video stream
- **Device Selection**: Choose from available camera devices
- **Retry**: Attempt to reconnect if errors occur

### 4. Monitor System Status
- Check the System Status panel for real-time component health
- Camera status is integrated with overall system monitoring
- Green indicators show healthy components, red shows issues

## Browser Compatibility

### Supported Browsers
- ✅ Chrome 53+ (recommended)
- ✅ Firefox 36+
- ✅ Edge 12+
- ✅ Safari 11+

### Requirements
- HTTPS connection (required for camera access)
- Camera permissions enabled
- Modern browser with MediaDevices API support

## Troubleshooting

### Common Issues

#### Camera Not Detected
1. Check physical connection (USB cable)
2. Ensure no other applications are using the camera
3. Try refreshing the page
4. Check browser permissions

#### Permission Denied
1. Click the camera icon in the browser address bar
2. Select "Allow" for camera access
3. Refresh the page
4. Check browser settings for site permissions

#### Poor Video Quality
1. Ensure good lighting conditions
2. Check camera focus
3. Verify USB connection is stable
4. Try different resolution settings

#### Browser Not Supported
1. Update to latest browser version
2. Use Chrome, Firefox, or Edge
3. Enable experimental features if available

### Debug Information

The dashboard includes debug information:
- Device IDs and names
- Stream status
- Last update timestamps
- Error messages with specific details

## System Integration

### Health Monitoring
The camera status is integrated with the backend health check system:

```python
# Backend health check includes camera availability
def check_camera_availability():
    # Check for video devices in /dev
    # Check v4l2-ctl availability
    # Check for camera-related processes
```

### Status Indicators
- **Dashboard Header**: Overall system status
- **VideoFeed Component**: Camera-specific status
- **SystemStatus Component**: Detailed component health
- **Controls Component**: System control status

## Performance Considerations

### Browser Resources
- Camera streams consume CPU and memory
- 720p resolution provides good balance of quality vs performance
- Automatic fallback to lower resolutions if needed

### Network Impact
- Live camera mode: No network overhead (local processing)
- Backend API mode: Network requests for image updates

### Memory Management
- Streams are properly cleaned up on component unmount
- Video elements are disposed when switching modes
- Automatic garbage collection of media streams

## Future Enhancements

### Planned Features
- **Recording**: Save video clips for analysis
- **Screenshot**: Capture still images
- **Resolution Control**: User-selectable video quality
- **Multiple Cameras**: Support for multiple camera feeds
- **AI Integration**: Real-time object detection overlay

### API Extensions
- **WebRTC**: For low-latency streaming
- **MediaRecorder**: For video recording capabilities
- **Canvas Integration**: For image processing and analysis

## Support

For technical support or feature requests:
1. Check the browser console for error messages
2. Verify camera hardware compatibility
3. Test with different browsers
4. Review system requirements

---

*This camera integration provides a robust foundation for robotics applications requiring real-time visual feedback and monitoring.*
