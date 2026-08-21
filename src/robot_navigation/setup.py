#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Автономная уличная GNSS-навигация (Nav2 + robot_localization)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Проезд маршрута из YAML через экшен /follow_gps_waypoints
            'gps_waypoint_commander = robot_navigation.gps_waypoint_commander:main',
            # Запись маршрута: сохраняет текущую позицию робота в YAML
            'gps_waypoint_logger = robot_navigation.gps_waypoint_logger:main',
            # Обработка магнитометра: калибровка, угол монтажа, склонение
            # (/imu/mag_raw -> /imu/mag)
            'mag_declination_node = robot_navigation.mag_declination_node:main',
            # Калибровка магнитометра вращением робота
            'mag_calibrator = robot_navigation.mag_calibrator:main',
            # Сверка курса робота с реальным азимутом
            'heading_check = robot_navigation.heading_check:main',
            # Диагностика готовности стека перед выездом
            'nav_preflight_check = robot_navigation.nav_preflight_check:main',
            # Проверка датчиков на шине I2C (работает без запущенного стека)
            'i2c_check = robot_navigation.i2c_check:main',
            # Пробная поездка на заданное расстояние (проверка связки с Nav2)
            'send_test_goal = robot_navigation.send_test_goal:main',
        ],
    },
)
