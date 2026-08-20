import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('view_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'view_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    # 1) Генерируем Substitution для xacro -> urdf
    robot_description_substitution = Command(['xacro ', xacro_file])
    # 2) Явно указываем, что это строка
    robot_description = ParameterValue(robot_description_substitution, value_type=str)
    # 3) Готовим словарь
    robot_description_param = {'robot_description': robot_description}

    # --- Узел robot_state_publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]  # <-- передаём dict
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Собираем всё в LaunchDescription
    return LaunchDescription([
        rsp_node,
        joint_state_publisher_node,
        rviz_node,
    ])