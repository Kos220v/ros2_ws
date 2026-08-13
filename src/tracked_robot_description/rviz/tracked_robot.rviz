#!/usr/bin/env python3
"""
Launch файл для визуализации гусеничного робота в RViz2.

Запускает три ноды:
  1. robot_state_publisher — читает URDF и публикует TF-дерево
  2. joint_state_publisher — публикует состояния суставов (статических)
  3. rviz2 — визуализатор 3D-модели

Использование:
  ros2 launch tracked_robot_description display.launch.py
  ros2 launch tracked_robot_description display.launch.py gui:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # ДОБАВИТЬ ЭТУ СТРОКУ


def generate_launch_description():

    # ── Пути к файлам ─────────────────────────────────────────────────────────
    pkg_share = FindPackageShare('tracked_robot_description')

    # Путь к Xacro файлу описания робота
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'tracked_robot.urdf.xacro'])

    # Путь к конфигурации RViz2
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'tracked_robot.rviz'])

    # ── Аргументы запуска ────────────────────────────────────────────────────
    # gui:=true  — показать слайдеры Joint State Publisher GUI
    # gui:=false — запустить без GUI (только статика)
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        choices=['true', 'false'],
        description='Запустить joint_state_publisher_gui со слайдерами'
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Запустить RViz2'
    )

    # ── Обработка robot_description ──────────────────────────────────────────
    # ИСПРАВЛЕНО: оборачиваем Command в ParameterValue с типом str
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ── Нода 1: robot_state_publisher ────────────────────────────────────────
    # Читает URDF через xacro и публикует:
    #   - параметр /robot_description
    #   - TF-трансформации между фреймами
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,  # ИСПРАВЛЕНО: используем ParameterValue
            'use_sim_time': False,
        }]
    )

    # ── Нода 2a: joint_state_publisher (без GUI) ──────────────────────────────
    # Публикует нулевые состояния для всех не-fixed суставов.
    # Условие: запускается ТОЛЬКО если gui:=false
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # ── Нода 2b: joint_state_publisher_gui (со слайдерами) ───────────────────
    # Открывает окно со слайдерами для управления суставами вручную.
    # Условие: запускается ТОЛЬКО если gui:=true
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # ── Нода 3: rviz2 ────────────────────────────────────────────────────────
    # Визуализатор. Загружает конфигурацию из .rviz файла.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ── Сборка LaunchDescription ─────────────────────────────────────────────
    return LaunchDescription([
        gui_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])