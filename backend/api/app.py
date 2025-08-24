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

# State
events_cache = []
counters_cache = defaultdict(int)
health_status = {'ros2': False, 'camera': False, 'db': False}
classifier_running = True

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
        
        # Check camera stream (simplified - just check if ROS2 is running)
        health_status['camera'] = health_status['ros2']
        
        # Check ROS2 by looking for events in database (more reliable)
        try:
            with sqlite3.connect(DATABASE_PATH) as conn:
                cursor = conn.execute('SELECT COUNT(*) FROM events')
                count = cursor.fetchone()[0]
                health_status['ros2'] = count > 0  # If we have events, ROS2 is working
        except:
            health_status['ros2'] = False
        
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
    """Stream endpoint - returns status for now"""
    return jsonify({
        'status': 'stream_unavailable',
        'message': 'Direct camera access via ROS2'
    })

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

@app.route('/api/classifications')
def get_classifications():
    """Get classification history from SQLite database - matches ROS2 web_node format"""
    try:
        # Try to connect to the classifier's database first
        db_path = os.path.expanduser('~/classifications.db')
        if not os.path.exists(db_path):
            # Fallback to our local database
            db_path = DATABASE_PATH
        
        with sqlite3.connect(db_path) as conn:
            # Check if this is the classifier database or our events database
            try:
                cursor = conn.execute('''
                    SELECT id, timestamp, label, confidence, raw_logits, image_source, created_at
                    FROM classifications 
                    ORDER BY created_at DESC 
                    LIMIT 100
                ''')
                rows = cursor.fetchall()
                classifications = []
                
                for row in rows:
                    classifications.append({
                        'id': row[0],
                        'timestamp': row[1],
                        'label': row[2],
                        'confidence': row[3],
                        'raw_logits': row[4],
                        'image_source': row[5],
                        'created_at': row[6]
                    })
                
                return jsonify({
                    'success': True,
                    'count': len(classifications),
                    'classifications': classifications
                })
                
            except sqlite3.OperationalError:
                # This is our events database, use different schema
                cursor = conn.execute('''
                    SELECT id, class, confidence, timestamp
                    FROM events 
                    ORDER BY timestamp DESC 
                    LIMIT 100
                ''')
                rows = cursor.fetchall()
                classifications = []
                
                for row in rows:
                    classifications.append({
                        'id': row[0],
                        'timestamp': row[3],  # timestamp is the last column
                        'label': row[1],      # class is the second column
                        'confidence': row[2],  # confidence is the third column
                        'raw_logits': None,
                        'image_source': 'camera',
                        'created_at': row[3]
                    })
                
                return jsonify({
                    'success': True,
                    'count': len(classifications),
                    'classifications': classifications
                })
            
    except Exception as e:
        print(f"Error in get_classifications: {e}")
        return jsonify({'error': str(e), 'classifications': []})

@app.route('/api/classifications/latest')
def get_latest_classification():
    """Get the most recent classification"""
    try:
        # Try to connect to the classifier's database first
        db_path = os.path.expanduser('~/classifications.db')
        if not os.path.exists(db_path):
            # Fallback to our local database
            db_path = DATABASE_PATH
        
        with sqlite3.connect(db_path) as conn:
            # Check if this is the classifier database or our events database
            try:
                cursor = conn.execute('''
                    SELECT id, timestamp, label, confidence, raw_logits, image_source, created_at
                    FROM classifications 
                    ORDER BY created_at DESC 
                    LIMIT 1
                ''')
                
                row = cursor.fetchone()
                if row:
                    return jsonify({
                        'success': True,
                        'classification': {
                            'id': row[0],
                            'timestamp': row[1],
                            'label': row[2],
                            'confidence': row[3],
                            'raw_logits': row[4],
                            'image_source': row[5],
                            'created_at': row[6]
                        }
                    })
                else:
                    return jsonify({'error': 'No classifications found'})
                    
            except sqlite3.OperationalError:
                # This is our events database, use different schema
                cursor = conn.execute('''
                    SELECT id, class, confidence, timestamp
                    FROM events 
                    ORDER BY timestamp DESC 
                    LIMIT 1
                ''')
                
                row = cursor.fetchone()
                if row:
                    return jsonify({
                        'success': True,
                        'classification': {
                            'id': row[0],
                            'timestamp': row[3],
                            'label': row[1],
                            'confidence': row[2],
                            'raw_logits': None,
                            'image_source': 'camera',
                            'created_at': row[3]
                        }
                    })
                else:
                    return jsonify({'error': 'No classifications found'})
                
    except Exception as e:
        print(f"Error in get_latest_classification: {e}")
        return jsonify({'error': str(e)})

@app.route('/api/current_image')
def get_current_image():
    """Get current image from camera or mock image"""
    try:
        # For now, return a mock image URL or status
        # In production, this would fetch from the camera stream
        return jsonify({
            'success': True,
            'image_url': '/api/mock_image',
            'timestamp': time.time(),
            'source': 'mock_camera'
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/mock_image')
def serve_mock_image():
    """Serve a mock image for testing"""
    # Return a simple 1x1 pixel JPEG
    jpeg_data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x11\x08\x00\x01\x00\x01\x01\x01\x11\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\xff\xc4\x00\x14\x10\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00\xaa\xff\xd9'
    return Response(jpeg_data, mimetype='image/jpeg')

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

@app.route('/api/classifier/start', methods=['POST'])
def classifier_start():
    global classifier_running
    classifier_running = True
    return jsonify({'success': True, 'running': classifier_running})

@app.route('/api/classifier/stop', methods=['POST'])
def classifier_stop():
    global classifier_running
    classifier_running = False
    return jsonify({'success': True, 'running': classifier_running})

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
    
    app.run(host='0.0.0.0', port=8000, debug=False)