#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():    
    pkg_ros_cartographer = get_package_share_directory('tugbot_cartographer')
    
    rviz_path = os.path.join(pkg_ros_cartographer, 'rviz/cartographer.rviz')
    config_path = os.path.join(pkg_ros_cartographer, 'config')
        
    # change resolution and period time to change the look of the map
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.01')
    
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', config_path,'-configuration_basename', 'config.lua'],
        remappings=[
            ('/scan', '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan'),
        ]
    )
        
    grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )
        
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path]
    )
        
    return LaunchDescription([
        cartographer,
        grid,
        rviz,
    ])