#!/usr/bin/env python3
"""
Simple launcher that starts all recycling robot nodes
"""
import subprocess
import sys
import time
import signal
import os

def run_command_in_background(cmd):
    """Run a command in the background and return the process"""
    print(f"Starting: {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)

def main():
    # Source ROS2 environment
    print("Setting up ROS2 environment...")
    
    # List of processes to manage
    processes = []
    
    try:
        # Start each node
        commands = [
            ["python3", "/workspace/src/recycling_robot/recycling_robot/nodes/camera.py"],
            ["python3", "/workspace/src/recycling_robot/recycling_robot/nodes/classifier.py"],
            ["python3", "/workspace/src/recycling_robot/recycling_robot/nodes/web.py"],
        ]
        
        for cmd in commands:
            proc = run_command_in_background(cmd)
            processes.append(proc)
            time.sleep(1)  # Stagger startup
        
        print("\n✅ All nodes started!")
        print("🌐 Web dashboard should be available at: http://localhost:8000")
        print("📹 Camera feed: /camera/image_raw")
        print("🤖 Classification results: /classification_result")
        print("\nPress Ctrl+C to stop all nodes...")
        
        # Wait for all processes
        while True:
            time.sleep(1)
            
            # Check if any process died
            for i, proc in enumerate(processes):
                if proc.poll() is not None:
                    print(f"⚠️ Process {i} exited with code {proc.returncode}")
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        
    finally:
        # Clean shutdown
        for proc in processes:
            if proc.poll() is None:  # Still running
                print(f"Terminating process {proc.pid}...")
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print(f"Force killing process {proc.pid}...")
                    proc.kill()
        
        print("✅ All processes stopped")

if __name__ == '__main__':
    main()