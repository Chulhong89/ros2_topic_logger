from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_topic_logger'), 'config', 'logger.yaml'
        ]),
        description='Absolute path to the logger YAML config file',
    )

    logger_node = Node(
        package='ros2_topic_logger',
        executable='topic_logger_node',
        name='topic_logger_node',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
        }],
    )

    return LaunchDescription([
        config_path_arg,
        logger_node,
    ])
