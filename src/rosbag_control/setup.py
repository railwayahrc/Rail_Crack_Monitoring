from setuptools import setup
import os

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
    data_files=[
        # Marker file for the ament resource index
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        # Install package.xml
        (f'share/{package_name}', ['package.xml']),
    ],
)

