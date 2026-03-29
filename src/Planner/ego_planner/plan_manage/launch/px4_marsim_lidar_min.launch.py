#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id')
    odom_topic = LaunchConfiguration('odom_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    frame_id = LaunchConfiguration('frame_id')

    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')
    planning_horizon = LaunchConfiguration('planning_horizon')

    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')

    flight_type = LaunchConfiguration('flight_type')
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs')


    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', odom_topic),
            ('grid_map/odom', odom_topic),
            ('grid_map/cloud', cloud_topic),
            ('/move_base_simple/goal', '/goal_pose'),
        ],
        parameters=[
            # visualization frame (used by traj_utils/PlanningVisualization)
            {'frame_id': frame_id},

            # fsm
            {'fsm/flight_type': flight_type},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},

            # grid map (PointCloud2 mode)
            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.099},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.5},
            {'grid_map/virtual_ceil_height': 2.9},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},
            {'grid_map/frame_id': frame_id},

            # planner manager
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': use_distinctive_trajs},
            {'manager/drone_id': drone_id},

            # optimization
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': 0.5},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},

            # bspline
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            # prediction (kept for compatibility)
            {'prediction/obj_num': 10},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0},
        ],
    )

    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        parameters=[
            {'traj_server/time_forward': 1.0},
            {'traj_server/is_to_px4': True}
        ],
        remappings=[
            ('position_cmd', '/planner_position_cmd'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='0', description='Drone id stored in planner messages (not used for input topics here)'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry topic (nav_msgs/Odometry) already started per README'),
        DeclareLaunchArgument('cloud_topic', default_value='/cloud', description='LiDAR PointCloud2 topic already started per README'),
        DeclareLaunchArgument('max_vel', default_value='2.0', description='EGO planner max velocity'),
        DeclareLaunchArgument('max_acc', default_value='3.0', description='EGO planner max acceleration'),
        DeclareLaunchArgument('planning_horizon', default_value='7.5', description='EGO planner planning horizon'),
        DeclareLaunchArgument('flight_type', default_value='1', description='EGO planner flight_type'),
        DeclareLaunchArgument('use_distinctive_trajs', default_value='True', description='EGO planner use distinctive trajectories'),
        DeclareLaunchArgument('map_size_x', default_value='42.0', description='Local map size X'),
        DeclareLaunchArgument('map_size_y', default_value='30.0', description='Local map size Y'),
        DeclareLaunchArgument('map_size_z', default_value='5.0', description='Local map size Z'),
        DeclareLaunchArgument('frame_id', default_value='map', description='Global frame id (should match odom header.frame_id)'),
        ego_planner_node,
        traj_server_node,
    ])
