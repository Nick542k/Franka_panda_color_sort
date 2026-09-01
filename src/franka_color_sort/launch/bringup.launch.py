import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    color_sort_share = get_package_share_directory('franka_color_sort')
    moveit_config_share = get_package_share_directory('franka_color_sort_moveit_config')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(color_sort_share, 'launch', 'world.launch.py')
        )
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(color_sort_share, 'launch', 'spawn_franka.launch.py')
        )
    )

    color_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(color_sort_share, 'launch', 'color_detector.launch.py')
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'move_group.launch.py')
        )
    )

    pick_and_sort_node = Node(
        package='franka_color_sort',
        executable='pick_and_sort_node',
        name='pick_and_sort_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_launch])
    delayed_color_detector = TimerAction(period=10.0, actions=[color_detector_launch])
    delayed_move_group = TimerAction(period=12.0, actions=[move_group_launch])
    delayed_pick_and_sort = TimerAction(period=18.0, actions=[pick_and_sort_node])

    return LaunchDescription([
        world_launch,
        delayed_spawn,
        delayed_color_detector,
        delayed_move_group,
        delayed_pick_and_sort,
    ])
