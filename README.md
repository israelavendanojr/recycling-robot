# 🤖 Recycling Robot - Lean & Sustainable

A robotics project focused on AI-powered waste classification and sorting, built with a clean, maintainable architecture.

## 🏗️ Architecture Overview

```
recycling-robot/
├── backend/               # Single Flask API
│   ├── app.py            # Main Flask application
│   ├── Dockerfile        # Backend container
│   └── requirements.txt  # Python dependencies
├── web/                  # React + TypeScript frontend
│   ├── src/              # Frontend source code
│   ├── package.json      # Node.js dependencies
│   └── Dockerfile        # Frontend container
├── ros2/                 # Pure ROS2 workspace
│   └── src/recycling_robot/
│       ├── nodes/        # ROS2 nodes (no web APIs)
│       ├── launch/       # ROS2 launch files
│       └── config/       # ROS2 configuration
├── scripts/              # Utility scripts
├── docker-compose.yml    # Service orchestration
├── Makefile             # Development commands
└── README.md            # This file
```

## 🚀 Quick Start (3 Steps)

### 1. Install Dependencies
```bash
make install
```

### 2. Start Services
```bash
make up
```

### 3. Access Dashboard
- **Frontend**: http://localhost:5173
- **Backend API**: http://localhost:8000
- **ROS2**: Running in container

## 🛠️ Development Commands

```bash
make help          # Show all available commands
make dev           # Start both backend and frontend in dev mode
make dev-backend   # Start only backend in dev mode
make dev-frontend  # Start only frontend in dev mode
make test          # Run tests
make lint          # Run code linting
make format        # Format code
make clean         # Clean Docker resources
make reset         # Reset everything (use with caution!)
```

## 📡 API Endpoints

The backend provides a single, unified API:

- `GET /api/health` - System health status
- `GET /api/classifications` - Classification history
- `GET /api/classifications/latest` - Most recent classification
- `GET /api/current_image` - ROS2 camera stream
- `GET /api/counters` - Material counts
- `POST /api/classifier/start|stop` - Control classifier

## 🔄 Data Flow

```
ROS2 Camera Node → ROS2 Classifier → SQLite Database → Flask API → React Frontend
```

## 🎯 Key Design Principles

- **Single API**: One Flask backend, no duplicate endpoints
- **Clean Separation**: ROS2 for robotics, Flask for API, React for UI
- **Minimal UI**: Dashboard shows only camera feed + classification logs
- **Auto-connect**: Everything works without user configuration
- **Real-time**: Live camera stream + auto-refreshing logs

## 🧹 What Was Cleaned Up

### ❌ Removed Bloat
- Duplicate Flask APIs (web_node.py)
- Nested backend/api/ structure
- Unnecessary status indicators and controls
- Complex UI toggles and device selectors
- Browser camera access (replaced with ROS2 stream)

### ✅ Kept Essentials
- Core ROS2 robotics functionality
- Single Flask API with all endpoints
- Minimal React dashboard
- Docker containerization
- Comprehensive .gitignore

## 🔧 Technology Stack

- **Backend**: Flask + SQLite + psutil
- **Frontend**: React + TypeScript + Tailwind CSS
- **Robotics**: ROS2 Humble
- **Containerization**: Docker + Docker Compose
- **Development**: Make + Virtual Environments

## 📋 Prerequisites

- Docker and Docker Compose
- Python 3.10+
- Node.js 18+
- ROS2 Humble (for local development)

## 🚨 Troubleshooting

### Common Issues
1. **Port conflicts**: Ensure ports 8000, 5173, and 8080 are available
2. **Permission errors**: Use `sudo` for file operations if needed
3. **Dependencies**: Run `make install` to set up virtual environments

### Reset Everything
```bash
make reset        # Clean slate
make install      # Reinstall dependencies
make up          # Start services
```

## 📚 Documentation

- **API Reference**: Check backend/app.py for endpoint details
- **ROS2 Nodes**: See ros2/src/recycling_robot/recycling_robot/nodes/
- **Frontend Components**: Explore web/src/components/

## 🤝 Contributing

1. **Keep it lean**: Don't add complexity without clear need
2. **Single responsibility**: Each component has one job
3. **No duplication**: Don't create duplicate APIs or functionality
4. **Document changes**: Update README and add comments

## 📄 License

This project is part of a learning robotics initiative.

---

*Built with ❤️ for sustainable robotics development*

