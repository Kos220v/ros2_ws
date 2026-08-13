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
import subprocess
def generate_launch_description():
    project_start_share = get_package_share_directory('project_start')
    config_dir = os.path.join(project_start_share, 'config')

    # --- ЖЁСТКАЯ СХЕМА ПОРТОВ: ЛИДАР = /dev/ttyUSB0, GPS = /dev/ttyUSB1 ---
    # Номера ttyUSB* назначает ядро по порядку опроса USB (физический порт
    # хаба): какое устройство опрошено первым — то и ttyUSB0.
    # Чтобы лидар ВСЕГДА был ttyUSB0, он должен быть физически воткнут в
    # USB-порт, который опрашивается первым (сейчас там GPS — поменяйте
    # кабели местами). Ниже — проверка при старте: если схема нарушена,
    # стек НЕ запустится и выведет понятную инструкцию.
    lidar_vendor = _tty_vendor_id('/dev/ttyUSB0')
    gps_vendor = _tty_vendor_id('/dev/ttyUSB1')
    if lidar_vendor != '10c4':
        raise RuntimeError(
            "ЛИДАР НЕ НА /dev/ttyUSB0! Сейчас ttyUSB0 = "
            f"VID:{lidar_vendor or '?'} (ожидался 10c4 = CP210x, лидар).\n"
            "  Схема: ЛИДАР = ttyUSB0, GPS = ttyUSB1.\n"
            "  Что сделать: 1) выключите робота; 2) поменяйте USB-кабели "
            "местами — ЛИДАР в тот USB-порт, где сейчас GPS (он опрашивается "
            "первым и даёт ttyUSB0), GPS — в другой; 3) включите и проверьте:\n"
            "    udevadm info /dev/ttyUSB0 | grep ID_VENDOR_ID   # 10c4\n"
            "    udevadm info /dev/ttyUSB1 | grep ID_VENDOR_ID   # 067b"
        )
    if gps_vendor != '067b':
        raise RuntimeError(
            "GPS НЕ НА /dev/ttyUSB1! Сейчас ttyUSB1 = "
            f"VID:{gps_vendor or '?'} (ожидался 067b = PL2303, GPS).\n"
            "  Схема: ЛИДАР = ttyUSB0, GPS = ttyUSB1. Проверьте подключение "
            "USB-устройств (см. инструкцию выше)."
        )
    lidar_port = '/dev/ttyUSB0'
    gps_port = '/dev/ttyUSB1'


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
            'publish_tf': True,
            'robot_x_joint': 'robot_x',
            'robot_y_joint': 'robot_y',
            'acc_invert': True,            # фикс «вверх ногами» (ENU-модель ahrs)

            # --- КУРС: инерциальный модуль (гироскоп) ---
            # ВАЖНО: компас ВЫКЛЮЧЕН (use_magnetometer=False) — в этом монтаже
            # он «залипает»: при вращении робота azim/MAG почти не меняются
            # (поле моторов/металла доминирует над земным). Включённый компас
            # тянул курс назад после поворотов (yaw 47°->4°). Курс — от
            # гироскопа с автокомпенсацией дрейфа (drift_compensation).
            # Включить компас можно после переноса датчика и проверки:
            #   ros2 run robot_odom imu_check --mag-heading-live
            # (вращать робота: ANG должен проходить 0..360°).
            'yaw_source': 'imu',           # гироскоп+акселерометр (динамика)
            'use_magnetometer': False,     # компас ВЫКЛЮЧЕН (залипает в монтаже)
            'mag_yaw_only': True,          # компас влияет только на yaw (не на уровень)
            'mag_yaw_gain': 0.01,          # компл. фильтр: пост. времени ~2 с
                                           # (код нормирует по imu_rate, 50 Гц -> 25 Гц)
            'mag_yaw_anchor_gain': 10.0,   # привязка к компасу в ПОКОЕ: x10 —
                                           # курс не «плывёт», когда робот стоит
            'mag_yaw_deadzone_deg': 1.0,   # не дёргать yaw шумом компаса (< 1°)
            # Поправка осей компаса относительно робота (град):
            #   ros2 run robot_odom imu_check --heading <азимут_телефона>
            'mag_yaw_offset_deg': 0.0,
            # QMC5883L выдаёт MZ «вниз» (NED-стиль) — при наклоне курс прыгает;
            # для QMC обычно нужен True, для HMC5883L — False.
            'mag_z_invert': False,
            # Калибровка компаса (hard/soft-iron): вращение робота на 360° —
            #   ros2 run robot_odom imu_check --calibrate-mag
            # затем пропишите сюда mag_hard_iron_x/y/z и mag_scale_x/y/z.
            'mag_hard_iron_x': 0.0,
            'mag_hard_iron_y': 0.0,
            'mag_hard_iron_z': 0.0,
            'mag_scale_x': 1.0,
            'mag_scale_y': 1.0,
            'mag_scale_z': 1.0,

            # --- ось X odom = стартовая ориентация робота (для RViz) ---
            # Компас даёт АБСОЛЮТНЫЙ курс (от севера), из-за чего odom
            # повёрнут на курс робота даже в покое. Фиксируем начальный
            # курс через 10 с после старта (после сходимости компаса) и
            # вычитаем его — оси X odom и base_link при старте совпадают.
            'yaw_zero_at_start': True,
            'yaw_zero_delay_sec': 10.0,

            # --- диагностика «угол скачет при стоящем роботе» ---
            # Лог yaw/магнитного курса каждые 5 с (в INFO). После отладки
            # можно поставить 0.0 или убрать параметр.
            'debug_period_sec': 5.0,

            # --- частота IMU: 25 Гц (меньше нагрузка на Pi) ---
            'imu_rate': 25.0,

            # --- статические TF теперь публикует robot_state_publisher из URDF ---
            'publish_imu_tf': False,
            'publish_mag_gps_tf': False,
            'publish_laser_tf': False,
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
    ekf_config_path = os.path.join(config_dir, 'ekf_gps.yaml')

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('/imu', '/imu/data'),
            ('/gps/fix', '/fix'),
            ('/odometry/filtered', '/odometry/global'),
        ],
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('/odometry/filtered', '/odometry/global')
        ],
    )

    # ------------------------------------------------------------ лидар
    # ЗАДЕРЖКА 10 сек: чтобы автокалибровка гироскопа odom_node (первые ~2 сек)
    # прошла БЕЗ вибрации лидара (иначе bias портится -> дрейф yaw).
    # Параметры — из config/ydlidar_params.yaml (ЖЁСТКО /dev/ttyUSB0 +
    # рабочий конфиг Tmini Plus: lidar_type 1, isSingleChannel false,
    # sample_rate 9).
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
        navsat_transform_node,
        ekf_global_node,
        ydlidar_delayed,              # лидар через 10 сек (после калибровки IMU)
        relay_node,
        waypoint_follower_node,
        obstacle_avoider_node,
    ])
