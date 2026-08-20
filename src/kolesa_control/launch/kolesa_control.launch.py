# -*- coding: utf-8 -*-
"""Запуск ноды kolesa_control с параметрами.

Отредактируйте значения под свой робот и запустите:
    ros2 launch kolesa_control kolesa_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kolesa_control",
            executable="kolesa_control",
            name="kolesa_control",
            output="screen",
            parameters=[{
                # Порты (зависят от платформы, см. README)
                "left_port": "/dev/ttyAMA4",
                "right_port": "/dev/ttyAMA5",
                "baud": 115200,

                # Геометрия и привод — ЗАМЕНИТЕ на реальные значения робота 0.0723
                "wheel_radius": 0.0723,
                "wheel_separation": 0.48,
                "gear_ratio": 88.92,
                "pole_pairs": 2,

                # Калибровка одометрии по оборотам выходного вала.
                # 0.0 = посчитать грубую оценку автоматически при старте
                # (см. лог узла) — ОБЯЗАТЕЛЬНО замените на настоящие числа
                # после экспериментальной калибровки (README.md, раздел
                # "Калибровка одометрии").
                "tacho_counts_per_revolution": 2157.0,
                "distance_per_revolution": 2.011,

                # Направление и пределы
                "invert_left": False,
                "invert_right": True,

                # Скважность VESC (duty cycle, управление разомкнутое —
                # см. README.md). ПОДБЕРИТЕ экспериментально под свой робот.
                "duty_min": 0.05,
                "duty_max": 0.5,

                # Тайминги
                "control_rate": 50.0,
                "telemetry_rate": 20.0,
                "cmd_timeout": 0.5,

                # Публикации
                "publish_joint_states": True,
                "left_wheel_joint": "left_track_joint",
                "right_wheel_joint": "right_track_joint",
                "publish_diagnostics": True,
            }],
        ),
    ])
