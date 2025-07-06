# ~/ros2_ws/src/rosbag_control/setup.py

from setuptools import setup

package_name = 'rosbag_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Control rosbag start/stop via topic',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rosbag_control_node = rosbag_control.rosbag_control_node:main',
        ],
    },
)

