# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
            
    pkg_ros_description = get_package_share_directory('tugbot_description')
    pkg_ros_odom = get_package_share_directory('rf2o_laser_odometry')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sdf_path = os.path.join(pkg_ros_description, 'sdf', 'tugbot_world.sdf')
    urdf_path = os.path.join(pkg_ros_description, 'urdf/model.urdf.xacro')
    odom_path = os.path.join(pkg_ros_odom, 'launch')
    
    use_sim_time = LaunchConfiguration('use_sim_time')    
    use_rviz = LaunchConfiguration('use_rviz')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {sdf_path}'
        }.items(),
    )

    # RViz
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_description, 'rviz', 'simulator.rviz')],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/tugbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',],
        parameters=[{'qos_overrides./model/tugbot.subscriber.reliability': 'reliable'}],
        output='screen'
    )
    
    
    joint = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
        
    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': Command(['xacro ', urdf_path])}],
        arguments=[urdf_path]
    )
    
    odom = IncludeLaunchDescription(PythonLaunchDescriptionSource([odom_path, '/rf2o_laser_odometry.launch.py']))

    return LaunchDescription([        
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='false',
            description='Open RViz.'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        
        gz_sim,
        bridge,
        rviz,
        joint,
        robot_state,
        odom,
    ])