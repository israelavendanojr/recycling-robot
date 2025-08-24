from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'recycling_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools', 
        'requests', 
        'opencv-python', 
        'torch', 
        'torchvision', 
        'numpy',
        'flask',           # Add Flask
        'flask-cors',      # Add Flask-CORS
        'pillow',          # Add Pillow for PIL Image processing
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Recycling robot package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'camera_node = recycling_robot.nodes.camera_node:main',
            'mock_camera_node = recycling_robot.nodes.mock_camera_node:main',
            'classifier_node = recycling_robot.nodes.classifier_node:main',
            'sorting_node = recycling_robot.nodes.sorting_node:main',
            'web_node = recycling_robot.nodes.web_node:main',
        ],
    },
)