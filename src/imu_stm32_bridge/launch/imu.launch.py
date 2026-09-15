"""Запуск моста STM32 IMU."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('imu_stm32_bridge')
    params_file = os.path.join(pkg, 'config', 'imu_params.yaml')

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/imu_stm32',
        description='Устройство UART IMU-модуля')
    ns_arg = DeclareLaunchArgument(
        'namespace', default_value='imu',
        description='Неймспейс топиков (imu/data, imu/mag, imu/azimuth)')

    node = Node(
        package='imu_stm32_bridge',
        executable='bridge_node',
        name='imu_stm32_bridge',
        namespace=LaunchConfiguration('namespace'),
        parameters=[params_file, {'port': LaunchConfiguration('port')}],
        output='screen',
    )
    return LaunchDescription([port_arg, ns_arg, node])
