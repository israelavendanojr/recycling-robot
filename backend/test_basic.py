#!/usr/bin/env python3
"""
Basic tests for the recycling robot Flask backend.
These tests verify the core functionality without requiring external dependencies.
"""

import pytest
import os
import tempfile
import sqlite3
from unittest.mock import patch, MagicMock

# Test that Flask app can be imported and created
def test_flask_app_import():
    """Test that the Flask app can be imported successfully"""
    try:
        from app import app
        assert app is not None
        assert hasattr(app, 'route')
        print("✅ Flask app imports successfully")
    except ImportError as e:
        pytest.fail(f"Failed to import Flask app: {e}")

def test_flask_app_creation():
    """Test that the Flask app can be created and has expected attributes"""
    try:
        from app import app
        assert app.name == 'app'
        assert hasattr(app, 'config')
        print("✅ Flask app creation successful")
    except Exception as e:
        pytest.fail(f"Failed to create Flask app: {e}")

def test_health_endpoint_exists():
    """Test that the health endpoint route exists"""
    try:
        from app import app
        # Check if the health route is registered
        routes = [str(rule) for rule in app.url_map.iter_rules()]
        assert '/api/health' in routes
        print("✅ Health endpoint route exists")
    except Exception as e:
        pytest.fail(f"Failed to check health endpoint: {e}")

def test_database_initialization():
    """Test that database initialization works correctly"""
    try:
        from app import init_db, DATABASE_PATH
        
        # Create a temporary directory for testing
        with tempfile.TemporaryDirectory() as temp_dir:
            # Mock the DATABASE_PATH to use temp directory
            with patch('app.DATABASE_PATH', os.path.join(temp_dir, 'test_robot.db')):
                # Initialize the database
                init_db()
                
                # Verify database file was created
                assert os.path.exists(os.path.join(temp_dir, 'test_robot.db'))
                
                # Verify table structure
                with sqlite3.connect(os.path.join(temp_dir, 'test_robot.db')) as conn:
                    cursor = conn.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='events'")
                    assert cursor.fetchone() is not None
                    
                    # Check table schema
                    cursor = conn.execute("PRAGMA table_info(events)")
                    columns = {row[1] for row in cursor.fetchall()}
                    expected_columns = {'id', 'class', 'confidence', 'timestamp'}
                    assert expected_columns.issubset(columns)
                
                print("✅ Database initialization successful")
    except Exception as e:
        pytest.fail(f"Database initialization failed: {e}")

def test_main_modules_import():
    """Test that main modules can be imported without errors"""
    try:
        # Test core imports
        import flask
        import sqlite3
        import psutil
        import requests
        print("✅ Core module imports successful")
    except ImportError as e:
        pytest.fail(f"Failed to import core modules: {e}")

def test_app_configuration():
    """Test that the app has basic configuration"""
    try:
        from app import app
        # Basic Flask app should have these attributes
        assert hasattr(app, 'config')
        assert hasattr(app, 'url_map')
        print("✅ App configuration check passed")
    except Exception as e:
        pytest.fail(f"App configuration check failed: {e}")

if __name__ == "__main__":
    # Run tests if executed directly
    pytest.main([__file__, "-v"])
