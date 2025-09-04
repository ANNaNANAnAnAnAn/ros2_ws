from setuptools import setup

package_name = 'jetracer_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/followe.launch.py']),  # Add your launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Motor and steering controller for JetRacer using ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aoa_node = jetracer_control.aoa_node:main',
            'lidar_node = jetracer_control.lidar_node:main',
            'gamepad_cmd_vel = jetracer_control.gamepad_cmd_vel:main',
        ],
    },
)
