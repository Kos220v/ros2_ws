#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Главный launch-файл проекта: поднимает весь стек автономного гусеничного
робота — датчики, привод, локализацию (EKF + GPS) и навигацию по маршруту.

ИСПРАВЛЕНО:
  1. Курс в /odom — СЛИЯНИЕ компаса и инерциального модуля: yaw_source='imu'
     (гироскоп+акселерометр отслеживают повороты) + use_magnetometer=true
     и mag_yaw_only=true (комплементарный фильтр подтягивает yaw к
     магнитному курсу — убирает дрейф гироскопа). Чистый компас
     (yaw_source='mag') «залипает» и больше не используется.
  2. Лидар запускается с задержкой 10 сек (TimerAction) — чтобы автокалибровка
     гироскопа odom_node прошла в тишине (иначе вибрация лидара портит bias →
     дрейф yaw).
  3. GPS: frame_id=gps_link (совпадает с URDF).
  4. executable='odom_node' без ".py".
  5. В ydlidar_launch.py НЕ должно быть static_transform_publisher для
     laser_frame (его публикует URDF).
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _first_existing(*paths):
    """Возвращает первый существующий путь (для udev-алиасов и ttyUSB*)."""
    for p in paths:
        if p and os.path.exists(p):
            return p
    return paths[0] if paths else None


def generate_launch_description():
    project_start_share = get_package_share_directory('project_start')
    config_dir = os.path.join(project_start_share, 'config')

    # --- порты USB (приоритет: udev-алиасы -> ttyUSB0/ttyUSB1) ---
    # После установки 99-robot-usb.rules лидар всегда /dev/lidar, GPS /dev/gps.
    # Если правила не установлены — используем ttyUSB0 (лидар) / ttyUSB1 (GPS)
    # по вашей схеме.
    lidar_port = _first_existing('/dev/lidar', '/dev/ttyUSB0')
    gps_port = _first_existing('/dev/gps', '/dev/ttyUSB1')


    # ------------------------------------------------------------ launch-аргументы
    home_lat_arg = DeclareLaunchArgument('home_latitude', default_value='0.0')
    home_lon_arg = DeclareLaunchArgument('home_longitude', default_value='0.0')
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=os.path.join(config_dir, 'waypoints.yaml')
    )

    # ------------------------------------------------------------ Приемник ELRS
    elrs_node = Node(
        package='elrs_receiver',
        executable='elrs_node',
        name='elrs_receiver',
        parameters=[{
            'port': '/dev/ttyAMA2',
            'baudrate': 420000,
            'deadzone': 0.02,
            'throttle_channel': 1,
            'steering_channel': 0,
            'invert_throttle': False,
            'invert_steering': False,
            'channel_min': 172,
            'channel_center': 992,
            'channel_max': 1811,
        }],
        output='screen'
    )

    # ------------------------------------------------------------ GPS (стандартный nmea_navsat_driver)
    # ВАЖНО (порты по вашей схеме): ЛИДАР = /dev/ttyUSB0, GPS = /dev/ttyUSB1.
    # Если при загрузке USB-устройства получают другие номера (ttyUSB2,3...),
    # сделайте udev-правила по VID/PID (см. README):
    #   лидар CP210x (10c4:ea60) -> /dev/lidar,  GPS PL2303 (067b:2303) -> /dev/gps
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_navsat_driver',
        parameters=[{
            'port': gps_port,   # GPS (Prolific PL2303, 067b:2303)
            'baud': 115200,
            # ВАЖНО: фрейм должен совпадать с gps_link в URDF.
            'frame_id': 'gps_link',
        }],
        remappings=[
          ('/fix', '/gps/fix'),     # перенаправляем /fix → /gnss/fix
          # (опционально) если хотите переименовать и /nav_sat_fix:
          # ('/nav_sat_fix', '/gnss/nav_sat_fix'),
        ],
        output='screen'
    )

    # ------------------------------------------------------------ привод (моторы)
    kolesa_control_node = Node(
        package='kolesa_control',
        executable='kolesa_control',
        name='kolesa_control',
        parameters=[{
            'left_port': '/dev/ttyAMA4',
            'right_port': '/dev/ttyAMA5',
            'baud': 115200,
            'wheel_radius': 0.0723,
            'wheel_separation': 0.48,
            'gear_ratio': 19.5,
            'pole_pairs': 2,
            'invert_left': False,
            'invert_right': True,
            'encoder_invert_left': False,
            'encoder_invert_right': True,
            'left_wheel_joint': 'left_track_joint',
            'right_wheel_joint': 'right_track_joint',
            'invert_angular': False,   # или False — подберите по тесту ниже
        }],
        output='screen'
    )

    # ------------------------------------------------------------ одометрия + IMU (AHRS EKF)
    # Нода читает I2C (MPU6050 + магнитометр QMC5883L/HMC5883L) и публикует
    # /odom и /imu/data.
    # ВАЖНО: курс = СЛИЯНИЕ компаса и инерциального модуля (см. параметры):
    #   * yaw_source='imu' — повороты отслеживает гироскоп (EKF), без
    #     «залипания» чистого компаса;
    #   * use_magnetometer=true + mag_yaw_only=true — комплементарный фильтр
    #     подтягивает yaw к абсолютному магнитному курсу: компас убирает
    #     дрейф гироскопа, гироскоп даёт динамику поворотов.
    odom_node = Node(
        package='robot_odom',
        executable='odom_node',            # без ".py" — имя entry point из setup.py
        name='robot_odom',
        parameters=[{
            'publish_tf': False
        }],
        output='screen'
    )
    
    mpu6050_node = Node(
        package='mpu6050_control',
        executable='mpu6050_control',            # без ".py" — имя entry point из setup.py
        name='mpu6050_control',
        parameters=[{
            'publish_tf': False
        }],
        remappings=[
            ('/imu', '/imu/data'),
        ],
        output='screen'
    )
    
    compass_node = Node(
        package='compass_control',
        executable='compass_control',            # без ".py" — имя entry point из setup.py
        name='compass_control',
        parameters=[{
        }],
        output='screen'
    )

    # ------------------------------------------------------------ TF-дерево (URDF)
    urdf_file = os.path.join(
        get_package_share_directory('tracked_robot_description'),
        'urdf', 'tracked_robot.urdf.xacro'
    )
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # ------------------------------------------------------------ мультиплексор команд
    cmd_mux_node = Node(
        package='cmd_switcher',
        executable='cmd_mux_node',
        name='cmd_switcher',
        output='screen'
    )

    # ------------------------------------------------------------ локализация (EKF Global + GPS)
    ekf_config_path = os.path.join(config_dir, 'ekf.yaml')
    navsat_transform_config_path = os.path.join(config_dir, 'navsat_transform.yaml')

    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[
          ekf_config_path,
          {'use_sim_time': False}
        ],
    )
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{'use_sim_time': False}, navsat_transform_config_path],

    )
    # ------------------------------------------------------------ лидар
    # ЗАДЕРЖКА 10 сек: чтобы автокалибровка гироскопа odom_node (первые ~2 сек)
    # прошла БЕЗ вибрации лидара (иначе bias портится -> дрейф yaw).
    # Параметры — из config/ydlidar_params.yaml (порт /dev/ttyUSB0 + Tmini Plus).
    ydlidar_params = os.path.join(config_dir, 'ydlidar_params.yaml')
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch', 'ydlidar_launch.py'
            )
        ),
        launch_arguments={
            'params_file': ydlidar_params,
        }.items(),
    )
    ydlidar_delayed = TimerAction(
        period=10.0,
        actions=[ydlidar_launch],
    )

    relay_node = Node(
        package='relay_reliable',
        executable='relay_node',
        name='relay_reliable',
        output='screen'
    )

    waypoint_follower_node = Node(
        package='robot_odom', executable='waypoint_follower',
        name='waypoint_follower', output='screen',
        parameters=[{
            'waypoints_file': os.path.join(config_dir, 'waypoints.yaml'),
            'home_latitude': LaunchConfiguration('home_latitude'),
            'home_longitude': LaunchConfiguration('home_longitude'),
        'max_speed': 0.8, 'k_lin': 0.5, 'k_ang': 1.5,
        'max_angular': 1.0,          # ограничение поворота (рад/с) — без него робот
                                     # крутится с 4.6 рад/с и «дёргается влево-вправо»
        'yaw_tolerance_deg': 10.0, 'stop_dist': 1.0,
        'bearing_smoothing': 0.3,    # сглаживание целевого курса — убирает флип знака
                                     # при ошибке ~180° из-за шума GPS-позиции
        'goal_yaw_offset_deg': 150.5,   # компенсация поворота map (-150.5° от севера)
                                     # истинного азимута (проверено телефоном: азимут 264°,
                                     # а map показывает -20°) — доворачиваем цель на 180°
        'angular_z_sign': -1.0,        # ИНВЕРСИЯ знака поворота: при ang<0 робот крутился
                                     # так, что yaw РАСТЁТ (убегал от цели, делал круги).
                                     # С -1.0 поворот совпадает с командой.
        }],
    )

    obstacle_avoider_node = Node(
        package='robot_odom', executable='obstacle_avoider',
        name='obstacle_avoider', output='screen',
        parameters=[{'safety_dist': 1.5, 'front_sector_deg': 45.0}],
    )

    return LaunchDescription([
        home_lat_arg,
        home_lon_arg,
        waypoints_file_arg,

        elrs_node,
        gps_node,
        kolesa_control_node,
        odom_node,                    # калибровка гироскопа в тишине (без лидара)
        robot_state_publisher_node,
        cmd_mux_node,
        compass_node,
        mpu6050_node,
        navsat_transform_node,
        ekf_localization_node,
        ydlidar_delayed,              # лидар через 10 сек (после калибровки IMU)
        relay_node,
        #waypoint_follower_node,
        #obstacle_avoider_node,
    ])
