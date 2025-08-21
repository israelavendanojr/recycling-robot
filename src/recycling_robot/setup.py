from setuptools import setup, find_packages

package_name = 'recycling_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dev.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Recycling robot ROS2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera.py = recycling_robot.nodes.camera:main',
            'classifier.py = recycling_robot.nodes.classifier:main',
            'web.py = recycling_robot.nodes.web:main',
        ],
    },
)