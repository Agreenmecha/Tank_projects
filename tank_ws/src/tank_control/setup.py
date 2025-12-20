from setuptools import setup
import os
from glob import glob

package_name = 'tank_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Motor control for ODrive via USB',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_interface_node.py = tank_control.odrive_interface_node:main',
            'safety_monitor_node.py = tank_control.safety_monitor_node:main',
            'teleop_gamepad_node.py = tank_control.teleop_gamepad_node:main',
            'shutdown_handler_node.py = tank_control.shutdown_handler_node:main',
        ],
    },
)
