import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_nav = get_package_share_directory('tugbot_navigation')
    pkg_ros_bringup = get_package_share_directory('nav2_bringup')
        
    nav2_path = os.path.join(pkg_ros_bringup, 'launch')
    rviz_path = os.path.join(pkg_ros_nav, 'rviz', 'nav.rviz')
    map_path = os.path.join(pkg_ros_nav, 'maps', 'my_map.yaml')
    param_path = os.path.join(pkg_ros_nav, 'params', 'param.yaml')
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        rviz,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_path, '/bringup_launch.py']),
            launch_arguments={
                'namespace': namespace,
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': param_path,
                }.items(),
        ),

        
        # Node(
        #     package='teleop',
        #     executable='controller',
        #     name='bot_controller',
        #     output='screen'
        #     ),
    ])