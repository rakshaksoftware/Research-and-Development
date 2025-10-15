from setuptools import setup

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_drone_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Drone simulation package for Gazebo and ROS 2 bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realtime_comm_node = drone_sim.realtime_comm_node:main',
            'comm_node = drone_sim.comm_node:main',
            'receive_node = drone_sim.receive_node:main',
            'move_node = drone_sim.move_node:main',
        ],
    },
)
