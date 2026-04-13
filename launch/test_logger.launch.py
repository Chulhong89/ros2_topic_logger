"""
Test launch file for ros2_topic_logger.

Starts the logger node with the test config and spins up dummy publishers for
every topic type used in logger_test.yaml:

  /test/imu   — sensor_msgs/msg/Imu        (mode: always,        20 Hz)
  /test/odom  — nav_msgs/msg/Odometry      (mode: periodic 100ms, 10 Hz)
  /test/pose  — geometry_msgs/msg/PoseStamped (mode: always,       1 Hz)
  /test/scan  — sensor_msgs/msg/LaserScan  (mode: event_window,   5 Hz)
                ranges contain 0.5 m which is below the trigger threshold
                of 1.0 m, so event logging fires immediately.

Log output is written to /tmp/ros2_logger_integration_test/ (see logger_test.yaml).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_topic_logger'), 'config', 'logger_test.yaml'
        ]),
        description='Absolute path to the test logger YAML config file',
    )

    # ── Logger node ──────────────────────────────────────────────────────────────
    logger_node = Node(
        package='ros2_topic_logger',
        executable='topic_logger_node',
        name='topic_logger_node',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
        }],
    )

    # ── Test publishers ──────────────────────────────────────────────────────────

    # /test/imu — mode: always (every message logged)
    imu_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/test/imu',
            'sensor_msgs/msg/Imu',
            (
                '{'
                '"header": {"frame_id": "imu_link"},'
                '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},'
                '"angular_velocity": {"x": 0.05, "y": 0.02, "z": 0.01},'
                '"linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81}'
                '}'
            ),
            '--rate', '20',
        ],
        output='screen',
    )

    # /test/odom — mode: periodic (min_period_ms: 100 → at most 10 Hz logged)
    odom_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/test/odom',
            'nav_msgs/msg/Odometry',
            (
                '{'
                '"header": {"frame_id": "odom"},'
                '"child_frame_id": "base_link",'
                '"pose": {"pose": {"position": {"x": 1.0, "y": 0.5, "z": 0.0},'
                '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},'
                '"twist": {"twist": {"linear": {"x": 0.5, "y": 0.0, "z": 0.0},'
                '"angular": {"x": 0.0, "y": 0.0, "z": 0.1}}}'
                '}'
            ),
            '--rate', '10',
        ],
        output='screen',
    )

    # /test/pose — mode: always
    pose_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/test/pose',
            'geometry_msgs/msg/PoseStamped',
            (
                '{'
                '"header": {"frame_id": "map"},'
                '"pose": {"position": {"x": 3.0, "y": 1.0, "z": 0.0},'
                '"orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}}'
                '}'
            ),
            '--rate', '1',
        ],
        output='screen',
    )

    # /test/scan — mode: event_window, trigger_distance_threshold: 1.0
    # ranges = [0.5, 0.5, ...] → min_valid_range = 0.5 < 1.0 → event fires
    scan_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/test/scan',
            'sensor_msgs/msg/LaserScan',
            (
                '{'
                '"header": {"frame_id": "laser"},'
                '"angle_min": -1.5708,'
                '"angle_max":  1.5708,'
                '"angle_increment": 0.0349,'
                '"scan_time": 0.2,'
                '"range_min": 0.1,'
                '"range_max": 10.0,'
                '"ranges": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]'
                '}'
            ),
            '--rate', '5',
        ],
        output='screen',
    )

    # Delay the scan publisher by 2 s so the logger node is fully up before
    # the first event-window trigger arrives.
    delayed_scan_pub = TimerAction(period=2.0, actions=[scan_pub])

    return LaunchDescription([
        config_path_arg,
        logger_node,
        imu_pub,
        odom_pub,
        pose_pub,
        delayed_scan_pub,
    ])
