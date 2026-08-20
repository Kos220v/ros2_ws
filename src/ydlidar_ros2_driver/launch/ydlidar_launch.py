#!/usr/bin/python3
# Copyright 2020, EAIBOT
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.',
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )

    # УДАЛЁН static_transform_publisher для laser_frame.
    # Причина: laser_frame публикует robot_state_publisher из URDF
    # (tracked_robot.urdf.xacro, джойнт base_to_laser, xyz=-0.21 0.16 0.55).
    # Два издателя одного статического TF = конфликт (и old-style warning).
    #
    # Если пакет ydlidar запускается ОТДЕЛЬНО (без robot_state_publisher),
    # добавьте здесь static publisher обратно, но в новом синтаксисе:
    #
    # tf2_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub_laser',
    #     arguments=[
    #         '--x', '-0.21', '--y', '0.16', '--z', '0.55',
    #         '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
    #         '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
    #     ],
    # )

    return LaunchDescription([
        params_declare,
        driver_node,
    ])