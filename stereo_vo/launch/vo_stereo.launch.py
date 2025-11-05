#!/usr/bin/env python3
"""
Launch file for stereo visual odometry stack
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    stereo_vo_dir = get_package_share_directory('stereo_vo')
    
    # Launch arguments
    camera_driver_arg = DeclareLaunchArgument(
        'camera_driver',
        default_value='argus',
        description='Camera driver to use: argus or v4l2'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Enable IMU integration'
    )
    
    publish_debug_images_arg = DeclareLaunchArgument(
        'publish_debug_images',
        default_value='true',
        description='Publish debug visualization images'
    )
    
    sensor_id_left_arg = DeclareLaunchArgument(
        'sensor_id_left',
        default_value='0',
        description='Argus sensor ID for left camera'
    )
    
    sensor_id_right_arg = DeclareLaunchArgument(
        'sensor_id_right',
        default_value='1',
        description='Argus sensor ID for right camera'
    )
    
    # Configuration files
    params_file = PathJoinSubstitution([
        FindPackageShare('stereo_vo'),
        'config',
        'params.yaml'
    ])
    
    # VO Node
    vo_node = Node(
        package='stereo_vo',
        executable='vo_node',
        name='vo_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_imu': LaunchConfiguration('use_imu'),
                'publish_debug_images': LaunchConfiguration('publish_debug_images'),
            }
        ]
    )
    
    # Static TF publisher for camera frames
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=[
            '--x', '0.1', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'left_camera_optical_frame',
            '--child-frame-id', 'right_camera_optical_frame'
        ]
    )
    
    # IMU node (conditional)
    imu_node = Node(
        package='lsm9ds0_imu',
        executable='lsm9ds0_node',
        name='lsm9ds0_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('stereo_vo'),
                'config',
                'imu.yaml'
            ])
        ],
        condition=IfCondition(LaunchConfiguration('use_imu'))
    )
    
    return LaunchDescription([
        camera_driver_arg,
        use_imu_arg,
        publish_debug_images_arg,
        sensor_id_left_arg,
        sensor_id_right_arg,
        vo_node,
        tf_static_node,
        imu_node,
    ])
