#!/usr/bin/env python3
# scripts/run_inference.py (Fixed)
"""
Streamlined main application for Phase 3.
Clean, focused on web dashboard functionality.
Ready for ROS2 migration in future phases.
"""

import os
import sys
import time
import logging
import argparse
from pathlib import Path

# Add project root to path for imports
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('logs/recycling_robot.log')
    ]
)

logger = logging.getLogger(__name__)

def check_dependencies():
    """Check if required modules can be imported."""
    missing = []
    
    try:
        import torch
        import torchvision
        import cv2
        import numpy as np
        from PIL import Image
        import flask
    except ImportError as e:
        missing.append(str(e))
    
    if missing:
        logger.error("Missing dependencies:")
        for dep in missing:
            logger.error(f"  - {dep}")
        logger.error("Please install requirements: pip install -r requirements.txt")
        return False
    
    return True

def setup_argument_parser() -> argparse.ArgumentParser:
    """Setup command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Recycling Robot Inference System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_inference.py --model recycler.pth --web
  python run_inference.py --model recycler.pth --single-shot
  python run_inference.py --model recycler.pth --console
        """
    )
    
    # Model and hardware settings
    parser.add_argument(
        '--model', 
        type=str, 
        default='recycler.pth',
        help='Path to trained model file'
    )
    parser.add_argument(
        '--camera', 
        choices=['auto', 'pi', 'mock'], 
        default='auto',
        help='Camera type to use'
    )
    parser.add_argument(
        '--resolution', 
        type=str, 
        default='640x480',
        help='Camera resolution (WIDTHxHEIGHT)'
    )
    
    # Operation modes
    parser.add_argument(
        '--web', 
        action='store_true',
        help='Run web dashboard (default mode)'
    )
    parser.add_argument(
        '--console', 
        action='store_true',
        help='Run in console-only mode'
    )
    parser.add_argument(
        '--single-shot', 
        action='store_true',
        help='Take single classification and exit'
    )
    parser.add_argument(
        '--test-minimal',
        action='store_true',
        help='Run minimal Flask test server'
    )
    
    # Web server settings
    parser.add_argument(
        '--host', 
        type=str, 
        default='0.0.0.0',
        help='Web server host address'
    )
    parser.add_argument(
        '--port', 
        type=int, 
        default=8000,
        help='Web server port'
    )
    
    # Inference settings
    parser.add_argument(
        '--inference-rate', 
        type=float, 
        default=5.0,
        help='Inference frequency in Hz'
    )
    parser.add_argument(
        '--console-delay', 
        type=float, 
        default=2.0,
        help='Delay between console outputs in seconds'
    )
    
    # Testing and validation
    parser.add_argument(
        '--test-model', 
        action='store_true',
        help='Test model loading and exit'
    )
    parser.add_argument(
        '--test-camera', 
        action='store_true',
        help='Test camera functionality and exit'
    )
    parser.add_argument(
        '--test-imports',
        action='store_true', 
        help='Test module imports and exit'
    )
    
    return parser

def parse_resolution(resolution_str: str) -> tuple:
    """Parse resolution string like '640x480' into tuple."""
    try:
        width, height = resolution_str.lower().split('x')
        return (int(width), int(height))
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Invalid resolution format: {resolution_str}. Use WIDTHxHEIGHT"
        )

def test_imports() -> bool:
    """Test if all project modules can be imported."""
    try:
        logger.info("Testing module imports...")
        
        # Test core imports
        from src.core.classifier import RecyclingClassifier, ClassificationResult
        from src.core.camera import CameraManager, create_camera
        logger.info("✓ Core modules")
        
        # Test service imports  
        from src.services.inference_service import InferenceService, create_inference_service
        logger.info("✓ Service modules")
        
        # Test web imports
        from src.web.api import WebAPI, create_web_api
        logger.info("✓ Web modules")
        
        logger.info("✓ All imports successful")
        return True
        
    except ImportError as e:
        logger.error(f"✗ Import failed: {e}")
        logger.error("Make sure you're running from the project root directory")
        return False

def test_minimal_flask(host: str, port: int) -> None:
    """Run a minimal Flask test server."""
    from flask import Flask, jsonify, render_template_string
    
    app = Flask(__name__)
    
    # Minimal test HTML
    TEST_HTML = """
    <!DOCTYPE html>
    <html>
    <head><title>Flask Test</title></head>
    <body>
        <h1>🧪 Flask Routing Test</h1>
        <p>If you can see this, Flask routing is working!</p>
        <button onclick="fetch('/api/test').then(r=>r.json()).then(d=>alert(JSON.stringify(d)))">
            Test API
        </button>
    </body>
    </html>
    """
    
    @app.route('/')
    def test_dashboard():
        return render_template_string(TEST_HTML)
    
    @app.route('/api/test')
    def test_api():
        return jsonify({"status": "Flask routing works!", "timestamp": time.time()})
    
    logger.info(f"🧪 Starting minimal Flask test on {host}:{port}")
    logger.info(f"   Open: http://{host}:{port}/")
    app.run(host=host, port=port, debug=True)

def test_model_loading(model_path: str) -> bool:
    """Test if model can be loaded successfully."""
    try:
        from src.core.classifier import RecyclingClassifier
        classifier = RecyclingClassifier(model_path)
        logger.info("✓ Model loaded successfully")
        logger.info(f"  Classes: {classifier.class_names}")
        logger.info(f"  Device: {classifier.device}")
        return True
    except Exception as e:
        logger.error(f"✗ Model loading failed: {e}")
        return False

def test_camera_functionality(camera_type: str, resolution: tuple) -> bool:
    """Test camera initialization and frame capture."""
    try:
        from src.core.camera import CameraManager
        with CameraManager(camera_type, resolution) as camera:
            frame = camera.capture_frame()
            logger.info(f"✓ Camera working: {frame.shape} frame captured")
            return True
    except Exception as e:
        logger.error(f"✗ Camera test failed: {e}")
        return False

def run_single_shot(inference_service) -> None:
    """Run single classification and display results."""
    logger.info("Taking single classification...")
    
    # Wait a moment for camera to stabilize
    time.sleep(1.0)
    
    result = inference_service.get_latest_result()
    if not result:
        logger.warning("No classification result available yet")
        return
    
    print("\n" + "="*50)
    print("CLASSIFICATION RESULT")
    print("="*50)
    print(f"Predicted Class: {result.predicted_class}")
    print(f"Confidence: {result.confidence:.4f} ({result.confidence*100:.2f}%)")
    print(f"Timestamp: {time.ctime(result.timestamp)}")
    
    print("\nAll Class Probabilities:")
    for class_name, prob in result.all_probabilities.items():
        print(f"  {class_name:>10}: {prob:.4f} ({prob*100:.2f}%)")
    print("="*50)

def run_console_mode(inference_service, delay: float) -> None:
    """Run continuous console output mode."""
    logger.info("Starting console mode (Ctrl+C to stop)")
    print("\n" + "="*60)
    print("CONTINUOUS CLASSIFICATION")
    print("="*60)
    print("Time     | Prediction      | Confidence | FPS")
    print("-"*60)
    
    try:
        while True:
            result = inference_service.get_latest_result()
            stats = inference_service.get_stats()
            
            if result:
                timestamp = time.strftime("%H:%M:%S")
                print(f"{timestamp} | {result.predicted_class:<15} | "
                      f"{result.confidence*100:>5.1f}%    | {stats.fps:>4.1f}")
            else:
                print(f"{time.strftime('%H:%M:%S')} | Waiting...      |    --     |  --")
            
            time.sleep(delay)
            
    except KeyboardInterrupt:
        print("\nStopping console mode...")

def main():
    """Main application entry point."""
    # Check dependencies first
    if not check_dependencies():
        return 1
        
    parser = setup_argument_parser()
    args = parser.parse_args()
    
    # Parse resolution
    try:
        resolution = parse_resolution(args.resolution)
    except argparse.ArgumentTypeError as e:
        logger.error(e)
        return 1
    
    # Print startup banner
    print("\n" + "="*60)
    print("🤖 RECYCLING ROBOT INFERENCE SYSTEM")
    print("="*60)
    
    # Handle test modes first
    if args.test_imports:
        return 0 if test_imports() else 1
    
    if args.test_minimal:
        test_minimal_flask(args.host, args.port)
        return 0
    
    if args.test_model:
        if not test_imports():
            return 1
        return 0 if test_model_loading(args.model) else 1
    
    if args.test_camera:
        if not test_imports():
            return 1
        return 0 if test_camera_functionality(args.camera, resolution) else 1
    
    # Test imports before proceeding
    if not test_imports():
        logger.error("Cannot proceed due to import failures")
        return 1
    
    logger.info(f"Model: {args.model}")
    logger.info(f"Camera: {args.camera} @ {resolution}")
    logger.info(f"Inference rate: {args.inference_rate} Hz")
    
    # Determine operation mode
    if not any([args.web, args.console, args.single_shot]):
        args.web = True  # Default to web mode
        logger.info("No mode specified, defaulting to web dashboard")
    
    # Validate model exists
    if not os.path.exists(args.model):
        logger.error(f"Model file not found: {args.model}")
        return 1
    
    # Initialize components
    try:
        logger.info("Initializing inference service...")
        from src.core.classifier import RecyclingClassifier
        from src.core.camera import CameraManager
        from src.services.inference_service import InferenceService
        from src.web.api import WebAPI
        
        classifier = RecyclingClassifier(args.model)
        camera = CameraManager(args.camera, resolution)
        inference_service = InferenceService(
            classifier=classifier,
            camera_manager=camera,
            inference_rate=args.inference_rate
        )
        logger.info("✓ All components initialized")
        
    except Exception as e:
        logger.error(f"Failed to initialize components: {e}")
        return 1
    
    # Run selected mode
    try:
        with inference_service:
            logger.info("Starting inference service...")
            
            if args.single_shot:
                run_single_shot(inference_service)
                
            elif args.console:
                run_console_mode(inference_service, args.console_delay)
                
            elif args.web:
                logger.info(f"🌐 Starting web dashboard on {args.host}:{args.port}")
                print(f"\n📱 Open: http://{args.host}:{args.port}/")
                print("   (Replace with your Pi's IP if accessing remotely)")
                print("\nPress Ctrl+C to stop server\n")
                
                web_api = WebAPI(inference_service)
                web_api.run(host=args.host, port=args.port)
    
    except KeyboardInterrupt:
        logger.info("Received interrupt signal")
    except Exception as e:
        logger.error(f"Application error: {e}")
        return 1
    finally:
        logger.info("Shutting down...")
    
    logger.info("✓ Shutdown complete")
    return 0

if __name__ == "__main__":
    exit(main())