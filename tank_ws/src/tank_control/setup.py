from setuptools import setup

package_name = 'tank_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odrive_interface.launch.py',
                                                 'launch/motor_control.launch.py',
                                                 'launch/teleop_gamepad.launch.py']),
        ('share/' + package_name + '/config', ['config/odrive_params.yaml']),
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
            'odrive_interface_node = tank_control.odrive_interface_node:main',
            'safety_monitor_node = tank_control.safety_monitor_node:main',
            'teleop_gamepad_node = tank_control.teleop_gamepad_node:main',
        ],
    },
)

