#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from setuptools import find_packages, setup

package_name = 'robot_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS 2 odometry and IMU node (MPU6050 + QMC5883L/HMC5883L) with EKF filtering and auto-calibration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_node = robot_odom.odom_node:main',
            'imu_check = robot_odom.imu_check:main',
            'lidar_check = robot_odom.lidar_check:main',
            'waypoint_follower = robot_odom.waypoint_follower:main',
            'obstacle_avoider = robot_odom.obstacle_avoider:main',
        ],
    },
)
