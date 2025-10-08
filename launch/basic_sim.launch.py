import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # robot_state_publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),

        # the diffdrive simulator node
        Node(
            package='en613_control',    
            executable='diffdrive_sim',     
            name='diffdrive_sim',
            output='screen',
            parameters=[
                # overrides:
                {'wheel_base': 0.825},
                {'wheel_radius': 0.4},
                {'publish_rate': 30.0},
                {'odom_frame': 'odom'},
                {'chassis_frame': 'chassis'}
            ]
        ),
    ])