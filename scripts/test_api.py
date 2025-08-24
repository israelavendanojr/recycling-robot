#!/usr/bin/env python3
"""
Test script to verify Flask API endpoints are working correctly.
Run this after starting the backend service.
"""

import requests
import json
import time

# Configuration
BASE_URL = "http://localhost:8000"
ENDPOINTS = [
    "/api/health",
    "/api/classifications", 
    "/api/classifications/latest",
    "/api/current_image",
    "/api/counters"
]

def test_endpoint(endpoint):
    """Test a single endpoint and return results"""
    try:
        url = f"{BASE_URL}{endpoint}"
        print(f"🔍 Testing: {endpoint}")
        
        response = requests.get(url, timeout=5)
        
        if response.status_code == 200:
            print(f"✅ {endpoint} - Status: {response.status_code}")
            
            # Try to parse JSON response
            try:
                data = response.json()
                if isinstance(data, dict):
                    if 'success' in data:
                        print(f"   📊 Success: {data['success']}")
                    if 'count' in data:
                        print(f"   📈 Count: {data['count']}")
                    if 'classifications' in data:
                        print(f"   🗂️  Classifications: {len(data['classifications'])} items")
                    if 'image_url' in data:
                        print(f"   🖼️  Image URL: {data['image_url']}")
                    if 'source' in data:
                        print(f"   📹 Source: {data['source']}")
                elif isinstance(data, list):
                    print(f"   📋 List with {len(data)} items")
                else:
                    print(f"   📄 Response type: {type(data)}")
                    
            except json.JSONDecodeError:
                print(f"   📄 Non-JSON response: {len(response.text)} characters")
                
        else:
            print(f"❌ {endpoint} - Status: {response.status_code}")
            print(f"   📄 Response: {response.text[:100]}...")
            
    except requests.exceptions.ConnectionError:
        print(f"❌ {endpoint} - Connection failed (service not running?)")
    except requests.exceptions.Timeout:
        print(f"❌ {endpoint} - Request timeout")
    except Exception as e:
        print(f"❌ {endpoint} - Error: {e}")
    
    print()

def test_classifier_control():
    """Test classifier start/stop endpoints"""
    print("🔍 Testing classifier control endpoints...")
    
    try:
        # Test start
        response = requests.post(f"{BASE_URL}/api/classifier/start", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Start classifier - Success: {data.get('success')}, Running: {data.get('running')}")
        else:
            print(f"❌ Start classifier - Status: {response.status_code}")
        
        # Test stop
        response = requests.post(f"{BASE_URL}/api/classifier/stop", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Stop classifier - Success: {data.get('success')}, Running: {data.get('running')}")
        else:
            print(f"❌ Stop classifier - Status: {response.status_code}")
            
    except Exception as e:
        print(f"❌ Classifier control test failed: {e}")
    
    print()

def main():
    """Main test function"""
    print("🧪 Testing Recycling Robot API Endpoints")
    print("=" * 50)
    print(f"🌐 Base URL: {BASE_URL}")
    print(f"⏰ Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # Test all GET endpoints
    for endpoint in ENDPOINTS:
        test_endpoint(endpoint)
    
    # Test POST endpoints
    test_classifier_control()
    
    print("🎯 Test Summary")
    print("=" * 50)
    print("✅ All endpoints should return 200 status codes")
    print("✅ /api/classifications should return classification data")
    print("✅ /api/current_image should return image info")
    print("✅ /api/health should return system status")
    print()
    print("🔍 Check browser console for frontend API calls")
    print("📊 Verify data is flowing from ROS2 → Database → Flask → React")
    print()
    print("🚀 If all tests pass, your API is working correctly!")

if __name__ == "__main__":
    main()


