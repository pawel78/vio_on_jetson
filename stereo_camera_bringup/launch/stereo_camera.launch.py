#!/usr/bin/env python3
"""
Launch file for stereo camera (Argus or V4L2)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    driver_arg = DeclareLaunchArgument(
        'driver',
        default_value='argus',
        description='Camera driver: argus or v4l2'
    )
    
    # Argus node
    argus_node = Node(
        package='stereo_camera_bringup',
        executable='argus_stereo_node',
        name='argus_stereo_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('stereo_camera_bringup'),
                'config',
                'stereo_argus.yaml'
            ])
        ],
        condition=IfCondition(
            LaunchConfiguration('driver', default='argus').equals('argus')
        )
    )
    
    # V4L2 node
    v4l2_node = Node(
        package='stereo_camera_bringup',
        executable='v4l2_stereo_node',
        name='v4l2_stereo_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('stereo_camera_bringup'),
                'config',
                'stereo_v4l2.yaml'
            ])
        ],
        condition=UnlessCondition(
            LaunchConfiguration('driver', default='argus').equals('argus')
        )
    )
    
    return LaunchDescription([
        driver_arg,
        argus_node,
        v4l2_node,
    ])
