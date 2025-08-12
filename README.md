## 📁 Directory Layout

```
recycling-robot/
├── src/
│   ├── core/
│   │   ├── __init__.py
│   │   ├── classifier.py          # Core AI classification logic
│   │   └── camera.py              # Camera abstraction layer
│   ├── services/
│   │   ├── __init__.py
│   │   └── inference_service.py   # Orchestrates camera + classifier
│   ├── web/
│   │   ├── __init__.py
│   │   ├── api.py                 # Flask REST API
│   │   └── templates/
│   │       └── dashboard.html     # Modern HTML dashboard
│   └── __init__.py
├── scripts/
│   ├── run_inference.py           # Streamlined main application
│   └── legacy/                    # Backup of old scripts
│       ├── inference_test.py
│       └── web_dashboard.py
├── models/                        # Model storage
│   └── recycler.pth
├── logs/                          # Application logs
├── requirements.txt
├── setup.py
└── README.md
```

## 🏗️ Architecture Overview

### Core Components

**🧠 `classifier.py`**
- Pure AI inference logic
- Hardware-agnostic model loading
- Structured result objects
- Easy to unit test and mock

**📷 `camera.py`**
- Abstract camera interface
- Pi camera and mock implementations  
- Context manager support
- Swappable backends for testing

**⚙️ `inference_service.py`**
- Coordinates camera + classifier
- Thread-safe continuous inference
- Performance statistics tracking
- Clean separation of concerns

**🌐 `api.py`**
- RESTful Flask API
- MJPEG video streaming
- JSON statistics endpoints
- Ready for React frontend

## 🚀 Usage Examples

### Web Dashboard (Primary Mode)
```bash
# Start web dashboard on Pi
python scripts/run_inference.py --model models/recycler.pth --web

# Access dashboard
http://your-pi-ip:8000
```

### Console Mode
```bash
# Continuous console output
python scripts/run_inference.py --model models/recycler.pth --console

# Single classification
python scripts/run_inference.py --model models/recycler.pth --single-shot
```

### Testing & Validation
```bash
# Test model loading
python scripts/run_inference.py --test-model

# Test camera functionality
python scripts/run_inference.py --test-camera
```

## 🔧 Configuration Options

| Argument | Description | Default |
|----------|-------------|---------|
| `--model` | Path to model file | `recycler.pth` |
| `--camera` | Camera type (auto/pi/mock) | `auto` |
| `--resolution` | Camera resolution | `640x480` |
| `--inference-rate` | Inference frequency (Hz) | `5.0` |
| `--host` | Web server host | `0.0.0.0` |
| `--port` | Web server port | `8000` |

## 📡 API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | System health check |
| `/api/stats` | GET | Current statistics |
| `/api/stats/reset` | POST | Reset statistics |
| `/api/config` | GET | System configuration |
| `/api/video/stream.mjpeg` | GET | MJPEG video stream |

## 🔄 ROS2 Migration Path

This modular design makes ROS2 migration straightforward:

1. **`classifier.py`** → ROS2 service node
2. **`camera.py`** → ROS2 camera publisher node  
3. **`inference_service.py`** → ROS2 action server
4. **Web API** → ROS2 web bridge or separate dashboard node

Each component has minimal dependencies and clear interfaces, making the transition smooth.

## ✨ Key Improvements

### Modularity
- ✅ Single responsibility per module
- ✅ Clear interfaces and abstractions
- ✅ Easy to test individual components
- ✅ Minimal coupling between layers

### Scalability
- ✅ Thread-safe inference service
- ✅ Configurable inference rates
- ✅ Memory-efficient streaming
- ✅ Ready for horizontal scaling

### Maintainability
- ✅ Comprehensive logging
- ✅ Error handling and recovery
- ✅ Clean configuration system
- ✅ Self-documenting code

### Development Experience
- ✅ Removed X11 display complexity
- ✅ Streamlined command-line interface
- ✅ Built-in testing utilities
- ✅ Mock camera for development

## 🎯 Phase 4 Readiness

This architecture provides a solid foundation for Phase 4 (Physical Sorting Mechanism):

- **Hardware Control**: Add new service modules for actuators
- **Coordination Logic**: Extend inference service for sorting decisions  
- **Safety Systems**: Implement emergency stops and error recovery
- **Integration Testing**: Mock systems already in place

The modular design ensures that adding physical sorting won't require major architectural changes.