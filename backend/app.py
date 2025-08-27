#!/usr/bin/env python3
from flask import Flask, jsonify, request, Response, send_from_directory, send_file
from flask_cors import CORS
import sqlite3
import time
import os
import requests
import threading
from collections import defaultdict
import subprocess
import psutil
from werkzeug.exceptions import NotFound


app = Flask(__name__)
CORS(app)

# Environment
DATABASE_PATH = './robot.db'  # Use local directory instead of /data
FRAME_PATH = os.getenv('FRAME_PATH', '/shared/current_frame.jpg')

# State
events_cache = []
counters_cache = defaultdict(int)
health_status = {'ros2': False, 'camera': False, 'db': False}
classifier_running = True

def init_db():
    """Initialize SQLite database"""
    # Create local directory if it doesn't exist
    db_dir = os.path.dirname(DATABASE_PATH)
    if db_dir:
        os.makedirs(db_dir, exist_ok=True)
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

def check_camera_availability():
    """Check if camera devices are available on the system"""
    try:
        # Check for video devices in /dev
        video_devices = [f for f in os.listdir('/dev') if f.startswith('video')]
        if video_devices:
            return True
        
        # Alternative: check if v4l2-ctl is available and can list devices
        try:
            result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                 capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and result.stdout.strip():
                return True
        except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.SubprocessError):
            pass
        
        # Check if any process is using camera-related devices
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if proc.info['name'] and any(cam in proc.info['name'].lower() for cam in ['camera', 'v4l', 'mjpg']):
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        return False
    except Exception as e:
        print(f"Camera availability check failed: {e}")
        return False

def check_ros2_status():
    """Check if ROS2 processes are running"""
    try:
        # Check for ROS2 processes
        ros2_processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if proc.info['name'] and any(ros in proc.info['name'].lower() for ros in ['ros', 'python3']):
                    if proc.info['cmdline'] and any('ros' in str(cmd).lower() for cmd in proc.info['cmdline']):
                        ros2_processes.append(proc.info['name'])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        # Also check if we have recent events in database (indicates ROS2 was working)
        try:
            with sqlite3.connect(DATABASE_PATH) as conn:
                cursor = conn.execute('SELECT COUNT(*) FROM events WHERE timestamp > ?', 
                                    (time.time() - 3600,))  # Events in last hour
                count = cursor.fetchone()[0]
                has_recent_events = count > 0
        except:
            has_recent_events = False
        
        return len(ros2_processes) > 0 or has_recent_events
    except Exception as e:
        print(f"ROS2 status check failed: {e}")
        return False

def check_database_health():
    """Check database connectivity and health"""
    try:
        with sqlite3.connect(DATABASE_PATH) as conn:
            # Test basic operations
            conn.execute('SELECT 1').fetchone()
            
            # Check if tables exist
            cursor = conn.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='events'")
            table_exists = cursor.fetchone() is not None
            
            # Check if we can write (test insert/rollback)
            conn.execute('BEGIN TRANSACTION')
            conn.execute('INSERT INTO events (class, confidence, timestamp) VALUES (?, ?, ?)', 
                        ('health_check', 1.0, time.time()))
            conn.rollback()
            
            return table_exists
    except Exception as e:
        print(f"Database health check failed: {e}")
        return False

def check_health():
    """Background health checker with improved monitoring"""
    global health_status
    
    while True:
        try:
            # Check database health
            health_status['db'] = check_database_health()
            
            # Check camera availability
            health_status['camera'] = check_camera_availability()
            
            # Check ROS2 status
            health_status['ros2'] = check_ros2_status()
            
            print(f"Health check results: {health_status}")
            
        except Exception as e:
            print(f"Health check error: {e}")
            health_status = {'ros2': False, 'camera': False, 'db': False}
        
        time.sleep(30)  # Check every 30 seconds

def get_current_frame_path():
    """Get the current frame path, with fallback from JPG to PNG"""
    # Try JPG first
    jpg_path = '/shared/current_frame.jpg'
    png_path = '/shared/current_frame.png'
    
    if os.path.exists(jpg_path):
        return jpg_path, 'image/jpeg'
    elif os.path.exists(png_path):
        return png_path, 'image/png'
    else:
        return None, None

# API Routes
@app.route('/api/health')
def get_health():
    """Enhanced health endpoint with detailed status"""
    overall = all(health_status.values())
    
    # Get additional system info
    try:
        system_info = {
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_usage': psutil.disk_usage('/').percent if os.path.exists('/') else 0,
            'uptime': time.time() - psutil.boot_time()
        }
    except:
        system_info = {}
    
    return jsonify({
        'status': 'ok' if overall else 'error',
        'services': health_status,
        'system': system_info,
        'timestamp': time.time(),
        'classifier_running': classifier_running
    })

@app.route('/api/stream')
def get_stream():
    """Stream endpoint - returns status for now"""
    return jsonify({
        'status': 'stream_unavailable',
        'message': 'Direct camera access via ROS2'
    })

@app.route('/api/pipeline/state')
def get_pipeline_state():
    """Get current pipeline state"""
    try:
        import json
        import os
        
        # Try to read from pipeline state file
        state_file = '/tmp/pipeline_state.json'
        if os.path.exists(state_file):
            try:
                with open(state_file, 'r') as f:
                    state_data = json.load(f)
                return jsonify(state_data)
            except Exception as e:
                print(f"Error reading pipeline state file: {e}")
        
        # Fallback to default state
        return jsonify({
            'state': 'idle',
            'timestamp': time.time(),
            'last_change': time.time(),
            'current_item': None
        })
        
    except Exception as e:
        return jsonify({
            'state': 'unknown',
            'error': str(e),
            'timestamp': time.time()
        }), 500

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

@app.route('/api/classifications', methods=['GET', 'POST'])
def handle_classifications():
    """Handle both GET and POST requests for classifications"""
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
            
            return jsonify({'success': True, 'message': 'Classification logged'})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500
    
    else:  # GET
        try:
            # Try to connect to the classifier's database first
            db_path = '/home/robot/classifications.db'
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
    """
    Return JSON pointing to the latest JPEG URL with cache-busting.
    Frontend can keep polling this endpoint; it will update image_url.
    """
    try:
        if not os.path.exists(FRAME_PATH):
            return jsonify({'success': False, 'error': 'no_frame'}), 404

        mtime = int(os.path.getmtime(FRAME_PATH) * 1000)
        return jsonify({
            'success': True,
            'image_url': f'/api/current_frame.jpg?ts={mtime}',
            'timestamp': time.time(),
            'source': 'ros2_camera'
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/current_frame.jpg')
def serve_current_frame():
    """
    Serve the latest frame (JPG or PNG) saved by the mock/real camera.
    Handles fallback from JPG to PNG automatically.
    """
    try:
        frame_path, mime_type = get_current_frame_path()
        
        if not frame_path:
            raise NotFound("No current frame available")
        
        # no-store avoids browser caching
        resp = send_file(frame_path, mimetype=mime_type, as_attachment=False, max_age=0, conditional=False)
        resp.headers['Cache-Control'] = 'no-store, max-age=0'
        return resp
        
    except NotFound as e:
        return jsonify({'success': False, 'error': 'no_frame'}), 404
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/ros2_camera_stream/<timestamp>')
def serve_ros2_camera_stream(timestamp):
    """Serve ROS2 camera stream images - simulates real camera feed"""
    try:
        # Generate a simple image with timestamp for demonstration
        # In production, this would serve actual camera frames from ROS2 topics
        
        # Create a simple colored image with timestamp
        from PIL import Image, ImageDraw, ImageFont
        import io
        
        # Create a 640x480 image (common camera resolution)
        width, height = 640, 480
        img = Image.new('RGB', (width, height), color='#1f2937')
        draw = ImageDraw.Draw(img)
        
        # Add timestamp text
        try:
            # Try to use a system font, fallback to default if not available
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
        except:
            font = ImageFont.load_default()
        
        # Draw timestamp
        timestamp_text = f"ROS2 Camera Stream - {int(float(timestamp))}"
        text_bbox = draw.textbbox((0, 0), timestamp_text, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        
        # Center the text
        x = (width - text_width) // 2
        y = (height - text_height) // 2
        
        # Draw text with outline for better visibility
        draw.text((x-1, y-1), timestamp_text, font=font, fill='black')
        draw.text((x+1, y-1), timestamp_text, font=font, fill='black')
        draw.text((x-1, y+1), timestamp_text, font=font, fill='black')
        draw.text((x+1, y+1), timestamp_text, font=font, fill='black')
        draw.text((x, y), timestamp_text, font=font, fill='white')
        
        # Add a simple camera indicator
        draw.ellipse([width-80, height-80, width-20, height-20], outline='#3b82f6', width=3)
        draw.ellipse([width-65, height-65, width-35, height-35], fill='#3b82f6')
        
        # Convert to JPEG
        img_io = io.BytesIO()
        img.save(img_io, 'JPEG', quality=85)
        img_io.seek(0)
        
        return Response(img_io.getvalue(), mimetype='image/jpeg')
        
    except Exception as e:
        print(f"Error generating camera stream image: {e}")
        # Fallback to simple 1x1 pixel if image generation fails
        jpeg_data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x11\x08\x00\x01\x00\x01\x01\x01\x11\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\xff\xc4\x00\x14\x10\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00\xaa\xff\xd9'
        return Response(jpeg_data, mimetype='image/jpeg')

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
    print("🚀 Starting Recycling Robot Backend...")
    print(f"📁 Database path: {DATABASE_PATH}")
    
    init_db()
    print("✅ Database initialized")
    
    # Start health checker
    health_thread = threading.Thread(target=check_health, daemon=True)
    health_thread.start()
    print("✅ Health checker started")
    
    print("🌐 Starting Flask server on port 8000...")
    app.run(host='0.0.0.0', port=8000, debug=False)