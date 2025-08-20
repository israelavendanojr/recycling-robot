from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'recycling_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files - fixed path
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include message definitions - fixed path  
        (os.path.join('share', package_name, 'msg'),
            glob('msg/*.msg')),
        # Include service definitions - fixed path
        (os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')),
        # Include action definitions - fixed path
        (os.path.join('share', package_name, 'action'),
            glob('action/*.action')),
    ],
    install_requires=['setuptools'],
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