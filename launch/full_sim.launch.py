import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file = os.path.join(
        get_package_share_directory('en613_control'),
        'urdf/basic_robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_config = os.path.join(
        get_package_share_directory('en613_control'),
        'rviz',
        'urdf_test.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        # State publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            output='screen',
        ),

        # Diff drive simulator
        Node(
            package='en613_control',
            executable='diffdrive_sim',
            name='diffdrive_sim',
            output='screen'
        ),

        # PID controller
        Node(
            package='en613_control',
            executable='diffdrive_pid',
            name='diffdrive_pid',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])
