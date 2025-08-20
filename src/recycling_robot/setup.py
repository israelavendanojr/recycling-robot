from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'recycling_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=['recycling_robot', 'recycling_robot.*']),
    data_files=[
        # Required ROS 2 package index + manifest
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Interfaces (if present)
        (os.path.join('share', package_name, 'msg'),    glob('msg/*.msg')),
        (os.path.join('share', package_name, 'srv'),    glob('srv/*.srv')),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),

        # Optional configs/models/web assets (add these dirs if you use them)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'web', 'static'),    glob('web/static/*')),
        (os.path.join('share', package_name, 'web', 'templates'), glob('web/templates/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',   # camera utils / image IO
        'pillow',          # classifier preprocess (PIL)
        'flask',           # web_bridge (if you use it)
    ],
    extras_require={
        # Install with:  pip install -e .[torch]
        # (On Raspberry Pi you may need a specific wheel; keeping unpinned here.)
        'torch': [
            'torch',
            'torchvision',
        ],
    },
    zip_safe=True,
    maintainer='Recycling Robot Team',
    maintainer_email='dev@recyclingbot.com',
    description='AI-powered recycling robot with computer vision classification and physical sorting',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core nodes
            'camera_node = recycling_robot.nodes.camera_node:main',
            'classifier_node = recycling_robot.nodes.classifier_node:main',
            'sorting_node = recycling_robot.nodes.sorting_node:main',

            # Support nodes
            'web_bridge_node = recycling_robot.nodes.web_bridge_node:main',
            'stats_monitor_node = recycling_robot.nodes.stats_monitor_node:main',
            'safety_node = recycling_robot.nodes.safety_node:main',
        ],
    },
)
