# Recycling Robot

A robotics project focused on AI-powered waste classification and sorting.

## Architecture Overview

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

## Quick Start 

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

## API Endpoints

The backend provides a single, unified API:

- `GET /api/health` - System health status
- `GET /api/classifications` - Classification history
- `GET /api/classifications/latest` - Most recent classification
- `GET /api/current_image` - ROS2 camera stream
- `GET /api/counters` - Material counts
- `POST /api/classifier/start|stop` - Control classifier

## Pipeline

```
ROS2 Camera Node → ROS2 Classifier → SQLite Database → Flask API → React Frontend
```


## Technology Stack

- **Backend**: Flask + SQLite + psutil
- **Frontend**: React + TypeScript + Tailwind CSS
- **Robotics**: ROS2 Humble, Pytorch Model
- **Containerization**: Docker + Docker Compose

## Troubleshooting

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

## Contributing

1. **Keep it lean**: Don't add complexity without clear need
2. **Single responsibility**: Each component has one job
3. **No duplication**: Don't create duplicate APIs or functionality
4. **Document changes**: Update README and add comments

---

*Built for sustainable robotics development*

