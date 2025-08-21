#!/usr/bin/env python3
from flask import Flask, jsonify, request, Response, send_from_directory
from flask_cors import CORS
import sqlite3
import time
import os
import requests
import threading
from collections import defaultdict

app = Flask(__name__)
CORS(app)

# Environment
DATABASE_PATH = '/data/robot.db'
STREAM_URL = 'http://host.docker.internal:8554/feed.mjpg'

# State
events_cache = []
counters_cache = defaultdict(int)
health_status = {'ros2': False, 'camera': False, 'db': False}

def init_db():
    """Initialize SQLite database"""
    os.makedirs('/data', exist_ok=True)
    with sqlite3.connect(DATABASE_PATH) as conn:
        conn.execute('''
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                class TEXT NOT NULL,
                confidence REAL NOT NULL,
                timestamp REAL NOT NULL
            )
        ''')
        conn.execute('PRAGMA journal_mode=WAL')  # Pi-safe
        conn.commit()

def check_health():
    """Background health checker"""
    global health_status
    
    while True:
        try:
            # Check database
            with sqlite3.connect(DATABASE_PATH) as conn:
                conn.execute('SELECT 1').fetchone()
            health_status['db'] = True
        except:
            health_status['db'] = False
        
        # Check camera stream
        try:
            response = requests.head(STREAM_URL, timeout=3)
            health_status['camera'] = response.status_code == 200
        except:
            health_status['camera'] = False
        
        # Check ROS2 (simple heartbeat file approach)
        health_status['ros2'] = os.path.exists('/tmp/ros2_alive')
        
        time.sleep(10)

# API Routes
@app.route('/api/health')
def get_health():
    overall = all(health_status.values())
    return jsonify({
        'ok': overall,
        'services': health_status,
        'timestamp': time.time()
    })

@app.route('/api/stream')
def get_stream():
    """Proxy MJPEG stream"""
    try:
        response = requests.get(STREAM_URL, stream=True, timeout=5)
        return Response(
            response.iter_content(chunk_size=1024),
            content_type=response.headers.get('Content-Type', 'multipart/x-mixed-replace')
        )
    except:
        return "Stream unavailable", 503

@app.route('/api/events', methods=['GET', 'POST'])
def handle_events():
    if request.method == 'POST':
        try:
            data = request.get_json()
            with sqlite3.connect(DATABASE_PATH) as conn:
                conn.execute(
                    'INSERT INTO events (class, confidence, timestamp) VALUES (?, ?, ?)',
                    (data['class'], data['confidence'], data['timestamp'])
                )
                conn.commit()
            
            # Update counters cache
            counters_cache[data['class']] += 1
            
            return jsonify({'success': True})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500
    
    else:  # GET
        try:
            with sqlite3.connect(DATABASE_PATH) as conn:
                cursor = conn.execute(
                    'SELECT * FROM events ORDER BY timestamp DESC LIMIT 50'
                )
                events = [
                    {'id': row[0], 'class': row[1], 'confidence': row[2], 'timestamp': row[3]}
                    for row in cursor.fetchall()
                ]
            return jsonify(events)
        except Exception as e:
            return jsonify([]), 500

@app.route('/api/counters')
def get_counters():
    try:
        with sqlite3.connect(DATABASE_PATH) as conn:
            cursor = conn.execute(
                'SELECT class, COUNT(*) FROM events GROUP BY class'
            )
            counters = dict(cursor.fetchall())
        return jsonify(counters)
    except Exception as e:
        return jsonify({}), 500

# Serve React app
@app.route('/')
def serve_app():
    return send_from_directory('static', 'index.html')

@app.route('/<path:path>')
def serve_static(path):
    return send_from_directory('static', path)

if __name__ == '__main__':
    init_db()
    
    # Start health checker
    health_thread = threading.Thread(target=check_health, daemon=True)
    health_thread.start()
    
    app.run(host='0.0.0.0', port=5000, debug=False)