#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bringup.launch.py — запуск ВСЕГО стека автономного робота одной командой.

    ros2 launch robot_navigation bringup.launch.py \\
        declination_deg:=17.21 \\
        waypoints_file:=/home/admin/route.yaml

Порядок запуска не случаен и задан таймерами:

    0 c   железо: моторы, GNSS, IMU, магнитометр, пульт, TF
          (гироскоп калибруется в первые ~4 секунды — робот должен стоять)
    8 c   локализация: фильтр курса, два EKF, navsat_transform
          (к этому моменту уже идут данные со всех датчиков)
   10 c   лидар (его мотор вибрирует, поэтому включается ПОСЛЕ калибровки;
          задержка задаётся внутри start.launch.py аргументом lidar_delay)
   15 c   Nav2 (нужна готовая TF-цепочка map -> odom -> base_link)
   20 c   командир маршрута (ждёт экшен Nav2)

Робот НЕ ПОЕДЕТ сразу после запуска. Командир маршрута стартует выключенным
и ждёт либо перевода тумблера пульта в режим AUTO, либо вызова сервиса
/gps_waypoint_commander/start_route.

ПЕРЕД ПЕРВЫМ ВЫЕЗДОМ обязательно прогоните проверку:
    ros2 run robot_navigation nav_preflight_check

ЗАМЕЧАНИЕ ПРО УСЛОВИЯ ЗАПУСКА
-----------------------------
Условия (condition=IfCondition(...)) навешены прямо на действия, а не через
GroupAction. GroupAction создаёт отдельную область видимости для аргументов
launch, и вложенные в неё TimerAction, у которых period задан подстановкой,
падают с ошибкой "launch configuration ... does not exist" — область
закрывается раньше, чем таймер успевает вычислить своё значение. Без лишней
обёртки такой проблемы не возникает в принципе.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')
    hw_share = get_package_share_directory('project_start')

    default_waypoints = os.path.join(nav_share, 'config', 'gps_waypoints.yaml')

    # ------------------------------------------------------------- аргументы
    args = [
        DeclareLaunchArgument(
            'declination_deg', default_value='11.9',
            description='Магнитное склонение в градусах для вашей местности. '
                        'Узнать: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml'),
        DeclareLaunchArgument(
            'waypoints_file', default_value=default_waypoints,
            description='YAML-файл с маршрутом из GPS-точек'),
        DeclareLaunchArgument(
            'number_of_loops', default_value='0',
            description='Сколько раз повторить маршрут (0 = один проезд)'),
        DeclareLaunchArgument(
            'use_hardware', default_value='true',
            description='Запускать слой железа (false — если он уже запущен)'),
        DeclareLaunchArgument(
            'use_navigation', default_value='true',
            description='Запускать Nav2 (false — только локализация, '
                        'для отладки положения и курса)'),
        DeclareLaunchArgument(
            'use_commander', default_value='true',
            description='Запускать командира маршрута'),
        DeclareLaunchArgument(
            'lidar_delay', default_value='10.0',
            description='Задержка старта лидара, сек'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            description='Файл параметров Nav2'),
    ]

    # ------------------------------------------------------- слой 1: железо
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_share, 'launch', 'start.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_hardware')),
        launch_arguments={
            'lidar_delay': LaunchConfiguration('lidar_delay'),
        }.items(),
    )

    # -------------------------------------------------- слой 2: локализация
    # Ждём 8 секунд: за это время mpu6050 успевает откалибровать ноль
    # гироскопа, а драйверы — открыть порты. Если поднять EKF раньше, он
    # получит поток с ещё не устоявшимся смещением и «уедет» по курсу.
    localization = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, 'launch', 'localization.launch.py')),
                launch_arguments={
                    'declination_deg': LaunchConfiguration('declination_deg'),
                }.items(),
            ),
        ],
    )

    # ------------------------------------------------------ слой 3: Nav2
    # Ждём 15 секунд: костмапам Nav2 при старте нужна готовая TF-цепочка
    # map -> odom -> base_link, иначе они сыплют ошибками трансформа
    # и уходят в состояние failure.
    navigation = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, 'launch', 'navigation.launch.py')),
                condition=IfCondition(LaunchConfiguration('use_navigation')),
                launch_arguments={
                    'params_file': LaunchConfiguration('nav2_params_file'),
                }.items(),
            ),
        ],
    )

    # ------------------------------------------------ слой 4: командир маршрута
    commander = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='robot_navigation',
                executable='gps_waypoint_commander',
                name='gps_waypoint_commander',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_commander')),
                parameters=[{
                    'waypoints_file': LaunchConfiguration('waypoints_file'),
                    # Тип указан явно. Иначе launch угадывает его по строке:
                    # "0" стало бы целым, а вот "2" при объявленном float
                    # параметре дало бы ошибку несовпадения типов.
                    'number_of_loops': ParameterValue(
                        LaunchConfiguration('number_of_loops'), value_type=int),
                    # Никогда не стартуем сами: только по пульту или по сервису.
                    'autostart': False,
                    'use_rc_mode': True,
                    'require_gps_fix': True,
                }],
            ),
        ],
    )

    return LaunchDescription(args + [
        hardware,
        localization,
        navigation,
        commander,
    ])
