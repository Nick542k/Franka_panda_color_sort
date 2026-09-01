import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    franka_description_share = get_package_share_directory('franka_description')
    default_xacro = os.path.join(
        franka_description_share, 'robots', 'fer', 'fer.urdf.xacro')

    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file', default_value=default_xacro,
        description='Path to the Franka FER xacro file')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='fer',
        description='Name to spawn the robot as in the Ignition world')

    xacro_file = LaunchConfiguration('xacro_file')
    robot_name = LaunchConfiguration('robot_name')

    robot_description_content = Command([
        'bash -c "xacro ', xacro_file,
        ' arm_id:=fer hand:=true gazebo:=true ros2_control:=true | \/home/nick542k/color_sort_ws/install/franka_color_sort/lib/franka_color_sort/patch_inertia"',
    ], on_stderr='warn')
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_fer',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fer_arm_controller'],
        output='screen',
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fer_hand_controller'],
        output='screen',
    )

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                hand_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        xacro_file_arg,
        robot_name_arg,
        robot_state_publisher_node,
        spawn_entity,
        delayed_controllers,
    ])
