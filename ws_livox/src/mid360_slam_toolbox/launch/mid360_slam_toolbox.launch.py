import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("mid360_slam_toolbox")
    livox_path = get_package_share_directory("livox_ros_driver2")
    fast_lio_path = get_package_share_directory("fast_lio")

    default_livox_config = os.path.join(livox_path, "config", "MID360_config.json")
    default_fast_lio_config_path = os.path.join(fast_lio_path, "config")
    default_slam_config = os.path.join(package_path, "config", "slam_toolbox_mid360.yaml")
    default_rviz_config = os.path.join(package_path, "rviz", "slam_toolbox_mid360.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    start_rviz = LaunchConfiguration("rviz")
    livox_config = LaunchConfiguration("livox_config")
    fast_lio_config_path = LaunchConfiguration("fast_lio_config_path")
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")
    range_min = LaunchConfiguration("range_min")
    range_max = LaunchConfiguration("range_max")
    angle_increment = LaunchConfiguration("angle_increment")
    slam_config = LaunchConfiguration("slam_config")
    rviz_config = LaunchConfiguration("rviz_config")

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=IfCondition(start_livox),
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"lvx_file_path": "/home/livox/livox_test.lvx"},
            {"user_config_path": livox_config},
            {"cmdline_input_bd_code": "livox0000000001"},
        ],
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        condition=IfCondition(start_fast_lio),
        parameters=[
            PathJoinSubstitution([fast_lio_config_path, fast_lio_config_file]),
            {"use_sim_time": use_sim_time},
        ],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", scan_topic),
        ],
        parameters=[
            {
                "target_frame": target_frame,
                "transform_tolerance": 0.05,
                "min_height": min_height,
                "max_height": max_height,
                "angle_min": -3.141592653589793,
                "angle_max": 3.141592653589793,
                "angle_increment": angle_increment,
                "scan_time": 0.1,
                "range_min": range_min,
                "range_max": range_max,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
            {
                "scan_topic": scan_topic,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
            },
        ],
    )

    slam_view_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="slam_view_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "3.141592653589793",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "slam_view_frame",
            "--child-frame-id", "map",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("livox_config", default_value=default_livox_config),
            DeclareLaunchArgument("fast_lio_config_path", default_value=default_fast_lio_config_path),
            DeclareLaunchArgument("fast_lio_config_file", default_value="mid360.yaml"),
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("target_frame", default_value="body"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("min_height", default_value="-0.2"),
            DeclareLaunchArgument("max_height", default_value="1.2"),
            DeclareLaunchArgument("range_min", default_value="0.3"),
            DeclareLaunchArgument("range_max", default_value="25.0"),
            DeclareLaunchArgument("angle_increment", default_value="0.008726646"),
            DeclareLaunchArgument("slam_config", default_value=default_slam_config),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            livox_driver,
            fast_lio,
            pointcloud_to_laserscan,
            slam_toolbox,
            slam_view_tf,
            rviz,
        ]
    )
