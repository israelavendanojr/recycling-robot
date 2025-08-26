#!/usr/bin/env python3
"""
Host GPIO Service for Raspberry Pi
This service runs on the host Pi and controls GPIO pins via HTTP API calls.
Your Docker container can call this service to control the motor.
"""

import RPi.GPIO as GPIO
import time
import json
from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# GPIO Configuration
IN1_PIN = 17
IN2_PIN = 27
ENA_PIN = 22
PWM_FREQ = 1000
DUTY_CYCLE = 50

# Motor state
current_position = 0
pwm = None

def setup_gpio():
    """Initialize GPIO pins"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1_PIN, GPIO.OUT)
    GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(ENA_PIN, GPIO.OUT)
    
    # Setup PWM
    global pwm
    pwm = GPIO.PWM(ENA_PIN, PWM_FREQ)
    pwm.start(0)
    
    # Stop motor initially
    stop_motor()
    print(f"[HostGPIO] GPIO configured: IN1={IN1_PIN}, IN2={IN2_PIN}, ENA={ENA_PIN}")

def cleanup_gpio():
    """Clean up GPIO resources"""
    if pwm:
        pwm.stop()
    GPIO.cleanup()
    print("[HostGPIO] GPIO cleanup completed")

def forward():
    """Move motor forward"""
    GPIO.output(IN1_PIN, GPIO.HIGH)
    GPIO.output(IN2_PIN, GPIO.LOW)
    pwm.ChangeDutyCycle(DUTY_CYCLE)

def backward():
    """Move motor backward"""
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.HIGH)
    pwm.ChangeDutyCycle(DUTY_CYCLE)

def stop_motor():
    """Stop motor"""
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

def move_to_bin(target_bin):
    """Move motor to target bin"""
    global current_position
    
    if target_bin == current_position:
        return {"success": True, "message": f"Already at bin {target_bin}"}
    
    try:
        # Calculate direction and steps
        steps = target_bin - current_position
        direction = 1 if steps > 0 else -1
        
        # Move motor
        if direction > 0:
            forward()
        else:
            backward()
        
        # Crude timing: 0.5 seconds per bin step
        step_time = abs(steps) * 0.5
        time.sleep(step_time)
        
        # Stop motor
        stop_motor()
        
        # Update position
        current_position = target_bin
        
        return {
            "success": True, 
            "message": f"Moved to bin {target_bin}",
            "position": current_position
        }
        
    except Exception as e:
        return {"success": False, "error": str(e)}

# API Endpoints
@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({"status": "healthy", "service": "host-gpio"})

@app.route('/motor/move', methods=['POST'])
def motor_move():
    """Move motor to target bin"""
    try:
        data = request.get_json()
        target_bin = data.get('bin')
        
        if target_bin is None:
            return jsonify({"success": False, "error": "Missing 'bin' parameter"}), 400
        
        if not isinstance(target_bin, int) or target_bin < 0 or target_bin > 4:
            return jsonify({"success": False, "error": "Bin must be 0-4"}), 400
        
        result = move_to_bin(target_bin)
        return jsonify(result)
        
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/motor/position', methods=['GET'])
def motor_position():
    """Get current motor position"""
    return jsonify({
        "success": True,
        "position": current_position
    })

@app.route('/motor/stop', methods=['POST'])
def motor_stop():
    """Stop motor immediately"""
    try:
        stop_motor()
        return jsonify({"success": True, "message": "Motor stopped"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/motor/forward', methods=['POST'])
def motor_forward():
    """Move motor forward"""
    try:
        forward()
        return jsonify({"success": True, "message": "Motor moving forward"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/motor/backward', methods=['POST'])
def motor_backward():
    """Move motor backward"""
    try:
        backward()
        return jsonify({"success": True, "message": "Motor moving backward"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

if __name__ == '__main__':
    try:
        print("[HostGPIO] Starting Host GPIO Service...")
        setup_gpio()
        
        print("[HostGPIO] Service ready on http://localhost:5001")
        print("[HostGPIO] Available endpoints:")
        print("  GET  /health")
        print("  POST /motor/move")
        print("  GET  /motor/position")
        print("  POST /motor/stop")
        print("  POST /motor/forward")
        print("  POST /motor/backward")
        
        app.run(host='0.0.0.0', port=5001, debug=False)
        
    except KeyboardInterrupt:
        print("\n[HostGPIO] Shutting down...")
    except Exception as e:
        print(f"[HostGPIO] Error: {e}")
    finally:
        cleanup_gpio()
