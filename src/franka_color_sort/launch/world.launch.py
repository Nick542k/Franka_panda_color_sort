import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('franka_color_sort')
    world_path = os.path.join(pkg_share, 'worlds', 'color_sort_world.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-r'],
        output='screen',
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
    )

    return LaunchDescription([
        gz_sim,
        bridge_node,
    ])
