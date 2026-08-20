#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/admin/ros2_ws/src/tracked_robot_description/rviz/lidar_display_fixed.rviz'],
        parameters=[{
            'use_sim_time': False,
        }],
        output='screen'
    )
    
    return LaunchDescription([rviz_node])
