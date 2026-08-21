#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
navigation.launch.py — стек Nav2 для уличной навигации без карты.

Отличия от стандартного nav2_bringup:

  * НЕ запускаются map_server и amcl. Глобальную локализацию даёт
    ekf_filter_node_map (см. localization.launch.py), карты местности нет.

  * Выход скоростей заворачивается в /cmd_vel/auto, а НЕ напрямую в /cmd_vel.
    Это принципиально: /cmd_vel слушает драйвер моторов, а решает, кого
    слушать, узел cmd_switcher. У пульта приоритет выше, чем у автопилота,
    поэтому оператор в любой момент перехватывает управление, просто двинув
    стик. Если подключить Nav2 напрямую к /cmd_vel, эта страховка исчезнет.

    Цепочка получается такая:
        controller_server -> /cmd_vel_nav
                          -> velocity_smoother (плавный разгон)
                          -> /cmd_vel/auto
                          -> cmd_switcher (приоритеты)
                          -> /cmd_vel -> kolesa_control
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Узлы Nav2 управляются машиной состояний lifecycle. Менеджер поднимает их
# строго в этом порядке и следит, чтобы упавший узел не остался забытым.
LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Файл параметров Nav2',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Автоматически переводить узлы Nav2 в состояние active',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    common = {'use_sim_time': use_sim_time}
    arguments = ['--ros-args', '--log-level', log_level]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, common],
        arguments=arguments,
        remappings=[
            # Сырые команды контроллера идут в сглаживатель, а не на моторы.
            ('cmd_vel', 'cmd_vel_nav'),
        ],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, common],
        arguments=arguments,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, common],
        arguments=arguments,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, common],
        arguments=arguments,
        remappings=[
            # Восстановительные манёвры (разворот, откат назад) тоже обязаны
            # проходить через мультиплексор, иначе пультом их не перебить.
            ('cmd_vel', 'cmd_vel_nav'),
        ],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, common],
        arguments=arguments,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, common],
        arguments=arguments,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, common],
        arguments=arguments,
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),          # вход
            ('cmd_vel_smoothed', 'cmd_vel/auto'),  # выход -> в мультиплексор
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': LIFECYCLE_NODES,
            # Перезапускать узел, если он умер: на Pi4 под нагрузкой это
            # изредка случается, и лучше пережить это автоматически.
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
        }],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        log_level_arg,

        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
