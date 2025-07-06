from setuptools import setup

package_name = 'multi_camera_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Multi camera publisher using OpenCV and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_camera_publisher = multi_camera_publisher.multi_camera_publisher:main'
        ],
    },
)
