#!/usr/bin/env python3
from flask import Flask, jsonify, Response
from flask_cors import CORS
import os
import sqlite3
import threading
import time

app = Flask(__name__)
CORS(app)

DATABASE_PATH = '/data/robot.db'
health_status = {'ros2': False, 'camera': False, 'db': False}

def init_db():
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
        conn.commit()

def check_health():
    global health_status
    while True:
        try:
            with sqlite3.connect(DATABASE_PATH) as conn:
                conn.execute('SELECT 1').fetchone()
            health_status['db'] = True
        except Exception:
            health_status['db'] = False
        # camera mirrors ros2 for now
        health_status['camera'] = health_status['ros2']
        try:
            with sqlite3.connect(DATABASE_PATH) as conn:
                cur = conn.execute('SELECT COUNT(*) FROM events')
                health_status['ros2'] = cur.fetchone()[0] > 0
        except Exception:
            health_status['ros2'] = False
        time.sleep(10)

@app.route('/video_feed')
def video_feed():
    # Return a tiny black JPEG so the endpoint is reachable
    jpeg_base64 = (
        b'/9j/4AAQSkZJRgABAQAAAQABAAD/2wCEAAkGBxAQEA8QDw8QDw8PDw8PDw8PDw8PDw8PFREWFhUR\n'
        b'FRUYHSggGBolGxUVITEhJSkrLi4uFx8zODMtNygtLisBCgoKDg0OGxAQGi0lICUtLS0tLS0tLS0t\n'
        b'LS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLf/AABEIAEAAQAMBIgACEQEDEQH/\n'
        b'xAAbAAACAwEBAQAAAAAAAAAAAAAEBQIDBgEAB//EADkQAAICAQIDBQMFAQkAAAAAAAECAxEEIQUS\n'
        b'MQZBUWEiMnGBkaGx8BRCUpLB0SMzQ1NygqLC/8QAGAEAAwEBAAAAAAAAAAAAAAAAAQIDBAX/xAAj\n'
        b'EQEBAAICAgMBAAAAAAAAAAABAgMRITESQQQiMlFx/9oADAMBAAIRAxEAPwD6ZbW2Ztq7a0q2kq5w\n'
        b'0rZb5Kxw2E4g7gGmXGfV9Y0bN2w9l5yqkQjQyQf8q8n6t9gA1P6b6HqV0pZbjbq7c9kJ3P0Cw5Xh\n'
        b'v8Kk8Q1rj5Jbqk5qKj3VYgJ0gY/kaHqjQq8Fqv4lqZqHn8Ck3L8YbH//2Q=='
    )
    import base64
    try:
        data = base64.b64decode(jpeg_base64)
        return Response(data, mimetype='image/jpeg')
    except Exception:
        return Response(status=200)

def main():
    init_db()
    t = threading.Thread(target=check_health, daemon=True)
    t.start()
    host = os.environ.get('WEB_HOST', '0.0.0.0')
    port = int(os.environ.get('WEB_PORT', '8080'))
    app.run(host=host, port=port, debug=False)

if __name__ == '__main__':
    main()
