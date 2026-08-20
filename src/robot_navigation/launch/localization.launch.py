#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
localization.launch.py — оценка положения робота на местности.

Поднимает четыре узла, образующих замкнутую цепочку:

    compass_control (/imu/mag_raw)
            |
            v
    mag_declination_node  -- снимает магнитное склонение --> /imu/mag
            |
            v
    imu_filter_madgwick   -- /imu/data_raw + /imu/mag --> /imu/data (курс ENU)
            |
            +--> ekf_filter_node_odom  --> TF odom -> base_link, /odometry/local
            |
            +--> ekf_filter_node_map   --> TF map -> odom,       /odometry/global
                        ^
                        |
                  /odometry/gps
                        ^
                        |
                 navsat_transform  <-- /gps/fix + /imu/data + /odometry/global

Запускается отдельно от навигации намеренно: локализацию нужно уметь проверять
без Nav2. Сначала добейтесь, чтобы робот правильно показывал своё положение и
курс, и только потом включайте автономное движение.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')
    config_dir = os.path.join(pkg_share, 'config')

    ekf_params = os.path.join(config_dir, 'dual_ekf_navsat.yaml')
    imu_filter_params = os.path.join(config_dir, 'imu_filter.yaml')

    # ------------------------------------------------------------- аргументы
    declination_arg = DeclareLaunchArgument(
        'declination_deg',
        # Значение для Нижнего Новгорода (56.3° с.ш., 43.9° в.д.) на 2026 год.
        # ОБЯЗАТЕЛЬНО пересчитайте для своего места:
        # https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        default_value='11.9',
        description='Магнитное склонение в градусах, восточное — положительное',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Использовать /clock вместо системного времени',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------- поправка магнитного склонения
    mag_declination_node = Node(
        package='robot_navigation',
        executable='mag_declination_node',
        name='mag_declination_node',
        output='screen',
        parameters=[{
            'declination_deg': LaunchConfiguration('declination_deg'),
            'input_topic': '/imu/mag_raw',
            'output_topic': '/imu/mag',
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------------------------------------------- фильтр ориентации (AHRS)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_filter_params, {'use_sim_time': use_sim_time}],
        # Топики по умолчанию (imu/data_raw, imu/mag, imu/data) совпадают
        # с нашими, поэтому ремапы не нужны.
    )

    # -------------------------------------------------------- локальный EKF
    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            # По умолчанию оба EKF публикуют в /odometry/filtered и
            # затирали бы друг друга. Разводим их по разным топикам.
            ('odometry/filtered', 'odometry/local'),
            ('accel/filtered', 'accel/local'),
            ('/set_pose', '/set_pose_local'),
        ],
    )

    # -------------------------------------------------------- глобальный EKF
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/global'),
            ('accel/filtered', 'accel/global'),
            ('/set_pose', '/set_pose_global'),
        ],
    )

    # ------------------------------------------------------ WGS84 -> метры
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            # Входы
            ('imu', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('odometry/filtered', 'odometry/global'),
            # Выходы
            ('odometry/gps', 'odometry/gps'),
            ('gps/filtered', 'gps/filtered'),
        ],
    )

    return LaunchDescription([
        declination_arg,
        use_sim_time_arg,
        mag_declination_node,
        imu_filter_node,
        ekf_odom_node,
        ekf_map_node,
        navsat_transform_node,
    ])
