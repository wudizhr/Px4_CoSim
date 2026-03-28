#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='odom', description='Odometry topic')
    odom_topic = LaunchConfiguration('odom_topic')
    map_pcd_path = PathJoinSubstitution([
        FindPackageShare('map_generator'),
        'resource',
        'small_forest01cutoff.pcd',
    ])

    # Map generator node
    map_generator_node = Node(
        package='map_generator',
        executable='map_pub',
        name='map_pub',
        output='screen',
        arguments=[map_pcd_path],
        parameters=[{
            'frame_id': 'map',
            'add_boundary': 0,
            'is_bridge': 0,
            'downsample_res': 0.1,
            'map_offset_x': 0.0,
            'map_offset_y': 0.0,
            'map_offset_z': 0.0,
        }]
    )

    # LiDAR simulation node
    lidar_node = Node(
        package='local_sensing_node',
        executable='opengl_render_node',
        name='quad0_pcl_render_node',
        output='screen',
        arguments=[map_pcd_path],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', '/odom'),
        ],
        parameters=[{
            'frame_id': 'map',
            'quadrotor_name': 'quad_0',
            'uav_num': 1,
            'is_360lidar': 1,
            'sensing_horizon': 30.0,
            'sensing_rate': 10.0,
            'estimation_rate': 10.0,
            'polar_resolution': 0.2,
            'yaw_fov': 360.0,
            'vertical_fov': 77.2,
            'min_raylength': 1.0,
            'livox_linestep': 1,
            'curvature_limit': 100.0,
            'hash_cubesize': 5.0,
            'use_avia_pattern': 1,
            'downsample_res': 0.1,
            'dynobj_enable': 0,
            'dynobject_size': 0.5,
            'dynobject_num': 5,
            'dyn_mode': 1,
            'dyn_velocity': 1.0,
            'use_uav_extra_model': 1,
            'collisioncheck_enable': 0,
            'collision_range': 0.5,
            'output_pcd': 0,
            'uav_num': 1,
        }]
    )

    # Odom visualization node for 3D drone model and trajectory
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        parameters=[{
            'mesh_resource': 'package://odom_visualization/meshes/yunque.dae',
            'color_r': 1.0,
            'color_g': 0.0,
            'color_b': 0.0,
            'color_a': 1.0,
            'robot_scale': 2.0,
            'frame_id': 'map',
            'quadrotor_name': 'quadrotor',
        }],
        remappings=[
            ('odom', 'odom'),
        ]
    )

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
        odom_topic_arg,
        map_generator_node,
        odom_visualization_node,  # 3D drone model and trajectory

        lidar_node,  # LiDAR simulation - re-enabled
    ])
