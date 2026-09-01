from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller'],
        output='screen',
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_hand_controller'],
        output='screen',
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
    ])
