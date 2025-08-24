#!/usr/bin/env python3
"""
Script to verify ROS2 nodes are running and communicating correctly.
Run this after launching the ROS2 pipeline.
"""

import subprocess
import time
import sys

def run_command(cmd, description):
    """Run a command and return success status"""
    print(f"🔍 {description}...")
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✅ {description} - Success")
            if result.stdout.strip():
                print(f"   📄 Output: {result.stdout.strip()}")
            return True
        else:
            print(f"❌ {description} - Failed")
            if result.stderr.strip():
                print(f"   📄 Error: {result.stderr.strip()}")
            return False
    except subprocess.TimeoutExpired:
        print(f"⏰ {description} - Timeout")
        return False
    except Exception as e:
        print(f"❌ {description} - Error: {e}")
        return False

def check_ros2_nodes():
    """Check if ROS2 nodes are running"""
    print("🧪 Verifying ROS2 Node Status")
    print("=" * 50)
    
    # Check node list
    success = run_command(
        "ros2 node list", 
        "Checking ROS2 node list"
    )
    
    if not success:
        print("❌ ROS2 nodes not found. Make sure to:")
        print("   1. Launch the ROS2 pipeline")
        print("   2. Source the workspace: source install/setup.bash")
        return False
    
    # Check specific nodes
    expected_nodes = [
        "mock_camera_node",
        "classifier_node", 
        "sorting_node",
        "simple_web"
    ]
    
    print("\n🔍 Checking expected nodes...")
    for node in expected_nodes:
        run_command(f"ros2 node info {node}", f"Node info for {node}")
    
    return True

def check_ros2_topics():
    """Check if ROS2 topics are active"""
    print("\n📡 Verifying ROS2 Topics")
    print("=" * 50)
    
    # Check topic list
    success = run_command(
        "ros2 topic list", 
        "Checking ROS2 topic list"
    )
    
    if not success:
        return False
    
    # Check specific topics
    expected_topics = [
        "/camera/image_raw",
        "/classifier/result"
    ]
    
    print("\n🔍 Checking expected topics...")
    for topic in expected_topics:
        run_command(f"ros2 topic info {topic}", f"Topic info for {topic}")
    
    return True

def check_ros2_services():
    """Check if ROS2 services are available"""
    print("\n🔧 Verifying ROS2 Services")
    print("=" * 50)
    
    # Check service list
    success = run_command(
        "ros2 service list", 
        "Checking ROS2 service list"
    )
    
    if not success:
        return False
    
    return True

def check_data_flow():
    """Check if data is flowing through the system"""
    print("\n🌊 Verifying Data Flow")
    print("=" * 50)
    
    # Check if camera is publishing images
    print("🔍 Checking camera image publishing...")
    success = run_command(
        "ros2 topic echo /camera/image_raw --once --timeout 5",
        "Camera image publishing"
    )
    
    if success:
        print("✅ Camera is publishing images")
    else:
        print("❌ Camera is not publishing images")
    
    # Check if classifier is receiving images
    print("\n🔍 Checking classifier subscription...")
    success = run_command(
        "ros2 topic info /camera/image_raw",
        "Camera topic subscription info"
    )
    
    return True

def main():
    """Main verification function"""
    print("🧪 ROS2 System Verification")
    print("=" * 50)
    print(f"⏰ Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # Check if we're in a ROS2 environment
    try:
        result = subprocess.run("ros2 --version", shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            print("❌ ROS2 not found. Make sure to:")
            print("   1. Source ROS2: source /opt/ros/humble/setup.bash")
            print("   2. Source workspace: source install/setup.bash")
            sys.exit(1)
    except Exception:
        print("❌ ROS2 environment not available")
        sys.exit(1)
    
    # Run all checks
    nodes_ok = check_ros2_nodes()
    topics_ok = check_ros2_topics()
    services_ok = check_ros2_services()
    data_flow_ok = check_data_flow()
    
    # Summary
    print("\n🎯 Verification Summary")
    print("=" * 50)
    print(f"📱 Nodes: {'✅' if nodes_ok else '❌'}")
    print(f"📡 Topics: {'✅' if topics_ok else '❌'}")
    print(f"🔧 Services: {'✅' if services_ok else '❌'}")
    print(f"🌊 Data Flow: {'✅' if data_flow_ok else '❌'}")
    
    if all([nodes_ok, topics_ok, services_ok, data_flow_ok]):
        print("\n🚀 All checks passed! ROS2 system is running correctly.")
        print("📊 Check the dashboard at http://localhost:5173")
        print("🔍 Monitor logs for classification events")
    else:
        print("\n⚠️  Some checks failed. Review the output above.")
        print("💡 Common issues:")
        print("   - Nodes not launched: run ros2 launch recycling_robot robot.launch.py")
        print("   - Workspace not sourced: source install/setup.bash")
        print("   - Docker container not running: docker-compose ps")

if __name__ == "__main__":
    main()


