#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_description = get_package_share_directory('tugbot_description')
    
    rviz_path = os.path.join(pkg_ros_description, 'rviz/model.rviz')
    urdf_path = os.path.join(pkg_ros_description, 'urdf/model.urdf.xacro')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_model = LaunchConfiguration('urdf_model')
    
    return LaunchDescription([                
        DeclareLaunchArgument(
            name='urdf_model', 
            default_value=urdf_path, 
            description='Absolute path to robot urdf file'),
                
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='false',
            description='Whether to start RVIZ'),
                DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 
            'robot_description': Command(['xacro ', urdf_model])}],
            arguments=[urdf_path]),
                
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]),
        
        # Node(
        #     condition=IfCondition(use_rviz),
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map2o',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        #     ),
        
        # Node(
        #     condition=IfCondition(use_rviz),
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map2o',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint ']
        #     ),
    ])
