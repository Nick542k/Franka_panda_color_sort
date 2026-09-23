#!/usr/bin/env python3
"""
Launch file for Ridgeback AMR simulation in Ignition Fortress
Starts Gazebo and Ridgeback robot with sensors and control nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulated time'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty',
        choices=['empty', 'obstacles'],
        description='World environment'
    )
    
    # Build world file path
    ridgeback_sim_launch = FindPackageShare('ridgeback_sim_launch')
    world_file = PathJoinSubstitution(
        [ridgeback_sim_launch, 'worlds', f'{world}.sdf']
    )
    
    # Gazebo Fortress environment
    gazebo_env = {
        'GZ_SIM_RESOURCE_PATH': os.path.join(
            os.path.expanduser('~'),
            '.local/share/ignition'
        ),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ.get(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', ''
        ),
    }
    
    # Ignition Fortress server (no GUI for now)
    ignition_server = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-r',  # Run server only
            '-v', '1',  # Verbosity
            world_file
        ],
        output='screen',
        env=gazebo_env,
        name='ignition_server'
    )
    
    # Ignition Fortress GUI (optional)
    ignition_gui = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-g',  # GUI only
            world_file
        ],
        output='screen',
        name='ignition_gui'
    )
    
    # ROS 2 - Ignition Bridge (connects Ignition topics to ROS)
    # This bridges Ignition Fortress to ROS2 Humble
    ignition_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            # Robot odometry
            '/model/ridgeback/odometry@nav_msgs/Odometry[ignition.msgs.Odometry',
            # Robot command
            '/model/ridgeback/cmd_vel@geometry_msgs/Twist]ignition.msgs.Twist',
            # IMU sensor
            '/model/ridgeback/imu_sensor/imu@sensor_msgs/Imu[ignition.msgs.IMU',
            # LiDAR scan
            '/model/ridgeback/gpu_lidar/scan@sensor_msgs/LaserScan[ignition.msgs.LaserScan',
        ],
        output='screen',
        name='ros_gz_bridge'
    )
    
    # Velocity command repeater (ensures continuity)
    cmd_vel_repeater = Node(
        package='topic_tools',
        executable='relay',
        parameters=[
            {'input_topic': '/cmd_vel'},
            {'output_topic': '/model/ridgeback/cmd_vel'},
        ],
        remappings=[
            ('/input_topic', '/cmd_vel'),
            ('/output_topic', '/model/ridgeback/cmd_vel'),
        ],
        output='screen'
    )
    
    # Sensor listener node (optional, for monitoring)
    sensor_listener = Node(
        package='ridgeback_sim_control',
        executable='sensor_listener',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Obstacle avoider node (uncomment to enable)
    # obstacle_avoider = Node(
    #     package='ridgeback_sim_control',
    #     executable='obstacle_avoider',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'safe_distance': 0.5},
    #         {'linear_speed': 0.3},
    #     ]
    # )
    
    # Create launch description
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_world,
        ignition_server,
        ignition_gui,
        ignition_bridge,
        sensor_listener,
        # obstacle_avoider,  # Uncomment to enable
    ])
    
    return ld
