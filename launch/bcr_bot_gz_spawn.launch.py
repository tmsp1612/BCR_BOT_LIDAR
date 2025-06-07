#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    bcr_bot_path = get_package_share_directory("bcr_bot")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")
    three_d_lidar_enabled = LaunchConfiguration("three_d_lidar_enabled", default=True)
    


    # robot_description_content = get_xacro_to_doc(
    #     join(bcr_bot_path, "urdf", "bcr_bot.xacro"),
    #     {"sim_gz": "true",
    #      "two_d_lidar_enabled": "true",
    #      "conveyor_enabled": "false",
    #      "camera_enabled": "true"
    #     }
    # ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', join(bcr_bot_path, 'urdf/bcr_bot.xacro'),
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' odometry_source:=', odometry_source,
                    ' three_d_lidar_enabled:=', three_d_lidar_enabled,
                    ' sim_gz:=', "true"
                    ])}],
        remappings=[
            ('/joint_states', 'bcr_bot/joint_states'),
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "bcr_bot",
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "kinect_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/default/model/bcr_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan_3d/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "kinect_camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

        ],
        remappings=[
            ('/world/default/model/bcr_bot/joint_state', 'bcr_bot/joint_states'),
            ('/odom', 'bcr_bot/odom'),
            ('/scan', 'bcr_bot/scan'),
            ('/kinect_camera', 'bcr_bot/kinect_camera'),
            ('/stereo_camera/left/image_raw', 'bcr_bot/stereo_camera/left/image_raw'),
            ('/stereo_camera/right/image_raw', 'bcr_bot/stereo_camera/right/image_raw'),
            ('/imu', 'bcr_bot/imu'),
            ('/cmd_vel', 'bcr_bot/cmd_vel'),
            ('kinect_camera/camera_info', 'bcr_bot/kinect_camera/camera_info'),
            ('stereo_camera/left/camera_info', 'bcr_bot/stereo_camera/left/camera_info'),
            ('stereo_camera/right/camera_info', 'bcr_bot/stereo_camera/right/camera_info'),
            ('/kinect_camera/points', 'bcr_bot/kinect_camera/points'),
            ('/kinect_camera/image_raw', 'bcr_bot/kinect_camera/image_raw'),
            ('kinect_camera/color/camera_info', 'bcr_bot/kinect_camera/color/camera_info')
            
        ]
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "kinect_camera",
                    "--child-frame-id", "bcr_bot/base_footprint/kinect_camera"]
    )
#     transform_publisher_lidar_3d = Node(
#     package="tf2_ros",
#     executable="static_transform_publisher",
#     arguments = ["--x", "0.0",
#                  "--y", "0.0",
#                  "--z", "0.0",
#                  "--yaw", "0.0",
#                  "--pitch", "0.0",
#                  "--roll", "0.0",
#                  "--frame-id", "velodyne",
#                  "--child-frame-id", "bcr_bot/base_footprint/scan_3d"]
# )
    static_tranform_base_link_to_base_footprint= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base',
        output='screen',
        arguments=['0','0','0.195','0','0','0', 'base_footprint', 'base_link']
    )

    static_tranform_base_link_to_imu_frame= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base',
        output='screen',
        arguments=['0','0','0','0','0','0', 'base_link', 'imu_frame']
    )
    static_tranform_imu_frame_to_velodyne= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base',
        output='screen',
        arguments=['0.34','0','0.5','0','0','0', 'imu_frame', 'velodyne']
    )
    
    static_tranform_map_to_odom= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base',
        output='screen',
        arguments=['0.0','0','0.0','0','0','0', 'map', 'odom']
    )


    static_tranform_odom_to_base_link= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base',
        output='screen',
        arguments=['0.0','0','0.0','0','0','0', 'odom', 'base_link']
    )
    return LaunchDescription([
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        DeclareLaunchArgument("three_d_lidar_enabled", default_value=three_d_lidar_enabled),

        robot_state_publisher,
        gz_spawn_entity, transform_publisher,gz_ros2_bridge,static_tranform_base_link_to_base_footprint,
        static_tranform_base_link_to_imu_frame,static_tranform_imu_frame_to_velodyne,
         static_tranform_map_to_odom, static_tranform_odom_to_base_link ])