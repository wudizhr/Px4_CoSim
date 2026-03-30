#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('test_interface'),
            'config',
            'px4.rviz'
        ])]
    )

    return LaunchDescription([
        rviz_node,  # RViz visualization
    ])
