from setuptools import setup

package_name = 'cart_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Cart control with ROS2 and web interface',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cart_control_node = cart_control.cart_control_node:main',
        ],
    },
)

