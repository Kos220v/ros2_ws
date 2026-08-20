#!/usr/bin/env python3
"""
Запускает всё, что нужно desktop-приложению для связи с роботом:
  - rosbridge_websocket  (порт 9090) — телеметрия, публикация /cmd_vel,
    смена режимов, чтение/запись ROS2-параметров из приложения
  - usb_cam              — драйвер USB-камеры, публикует /usb_cam/image_raw
  - web_video_server     (порт 8080) — отдаёт /usb_cam/image_raw как MJPEG
    по HTTP, приложение показывает его в обычном <img>/<video> теге

Требуемые системные пакеты (НЕ входят в этот workspace, ставятся из apt):
    sudo apt install ros-$ROS_DISTRO-rosbridge-suite \
                      ros-$ROS_DISTRO-web-video-server \
                      ros-$ROS_DISTRO-usb-cam

Запускается ОТДЕЛЬНО от start.launch.py (обычно бо́льшую часть времени вам
не нужно видео/rosbridge, если вы не используете desktop-приложение), но
можно объединить, добавив этот файл в start.launch.py через
IncludeLaunchDescription, если хотите, чтобы всё поднималось одной командой.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml'
            )
        )
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 15.0,
            'pixel_format': 'yuyv',
            'image_width': 640,
            'image_height': 480,
            'camera_name': 'robot_cam',
        }],
        output='screen',
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080}],
        output='screen',
    )

    return LaunchDescription([
        rosbridge_launch,
        usb_cam_node,
        web_video_server_node,
    ])
