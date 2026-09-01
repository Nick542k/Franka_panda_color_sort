from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Camera image topic to subscribe to')

    return LaunchDescription([
        image_topic_arg,
        Node(
            package='franka_color_sort',
            executable='color_detector_node',
            name='color_detector_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
            }],
        ),
    ])
