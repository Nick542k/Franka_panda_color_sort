import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    franka_description_share = get_package_share_directory('franka_description')
    default_xacro = os.path.join(
        franka_description_share, 'robots', 'fer', 'fer.urdf.xacro')

    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file', default_value=default_xacro,
        description='Path to the Franka FER xacro file')
    xacro_file = LaunchConfiguration('xacro_file')

    robot_description_content = Command([
        'bash -c "xacro ', xacro_file,
        ' arm_id:=fer hand:=true gazebo:=true ros2_control:=true | $(ros2 pkg prefix franka_color_sort)/lib/franka_color_sort/patch_inertia"',
    ], on_stderr='warn')
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config_share = get_package_share_directory('franka_color_sort_moveit_config')
    with open(os.path.join(moveit_config_share, 'config', 'fer.srdf'), 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('franka_color_sort_moveit_config', 'config/kinematics.yaml')
    ompl_yaml = load_yaml('franka_color_sort_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml(
        'franka_color_sort_moveit_config', 'config/moveit_controllers.yaml')
    joint_limits_yaml = load_yaml('franka_color_sort_moveit_config', 'config/joint_limits.yaml')

    planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_yaml,
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            planning_pipeline_config,
            {'robot_description_planning': joint_limits_yaml},
            moveit_controllers_yaml,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        xacro_file_arg,
        move_group_node,
    ])
