#!/usr/bin/env python3
"""
Simple launcher for the recycling robot
Starts all nodes in separate processes
"""

import subprocess
import time
import signal
import sys
import os

# Add current directory to Python path
sys.path.insert(0, '/workspace')

def run_node(script_path, node_name):
    """Run a ROS2 node in a subprocess"""
    cmd = [
        'python3', script_path,
        '--ros-args',
        '--log-level', 'info'
    ]
    
    print(f"Starting {node_name}...")
    try:
        process = subprocess.Popen(
            cmd,
            env=dict(os.environ, PYTHONPATH='/workspace:' + os.environ.get('PYTHONPATH', ''))
        )
        return process
    except Exception as e:
        print(f"Failed to start {node_name}: {e}")
        return None

def main():
    print("="*60)
    print("🤖 STARTING SIMPLE RECYCLING ROBOT")
    print("="*60)
    
    # Start nodes
    processes = []
    
    # Camera node
    camera_proc = run_node('/workspace/src/recycling_robot/nodes/camera.py', 'Camera')
    if camera_proc:
        processes.append(('Camera', camera_proc))
        time.sleep(2)  # Give camera time to initialize
    
    # Classifier node
    classifier_proc = run_node('/workspace/src/recycling_robot/nodes/classifier.py', 'Classifier')
    if classifier_proc:
        processes.append(('Classifier', classifier_proc))
        time.sleep(1)
    
    # Web dashboard node
    web_proc = run_node('/workspace/src/recycling_robot/nodes/web.py', 'Web Dashboard')
    if web_proc:
        processes.append(('Web Dashboard', web_proc))
        time.sleep(2)
    
    print("="*60)
    print("✅ ALL NODES STARTED")
    print("🌐 Dashboard: http://localhost:8000")
    print("🎥 Camera feed should be visible in dashboard")
    print("🧠 Auto-classification every 3 seconds")
    print("")
    print("Press Ctrl+C to stop all nodes")
    print("="*60)
    
    # Handle shutdown
    def signal_handler(signum, frame):
        print("\n" + "="*60)
        print("🛑 SHUTTING DOWN...")
        print("="*60)
        
        for name, proc in processes:
            try:
                print(f"Stopping {name}...")
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print(f"Force killing {name}...")
                proc.kill()
                proc.wait()
            except Exception as e:
                print(f"Error stopping {name}: {e}")
        
        print("✅ All nodes stopped")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Monitor processes
    try:
        while True:
            time.sleep(5)
            
            # Check if any process died
            for name, proc in processes[:]:  # Copy list to avoid modification during iteration
                if proc.poll() is not None:
                    print(f"⚠️  {name} node died with return code {proc.returncode}")
                    processes.remove((name, proc))
                    
                    # Optionally restart dead nodes
                    if name == 'Camera':
                        print("🔄 Restarting camera node...")
                        new_proc = run_node('/workspace/simple_nodes/simple_camera.py', 'Camera')
                        if new_proc:
                            processes.append(('Camera', new_proc))
            
            if not processes:
                print("❌ All nodes died, exiting...")
                break
                
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)

if __name__ == '__main__':
    main()