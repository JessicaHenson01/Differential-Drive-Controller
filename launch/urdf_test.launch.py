import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'basic_robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('en613_control'),
        urdf_file_name)
    
    # Define the path to the RViz configuration file.
    # I'll create this file for you below. Let's assume it's in an 'rviz' subdirectory.
    rviz_config_file_name = 'urdf_test.rviz'
    rviz_config_path = os.path.join(
        get_package_share_directory('en613_control'),
        'rviz',
        rviz_config_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        # Joint State Publisher
        Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
        ),
        # Node(
        #     package='en613_control',
        #     executable='state_publisher',
        #     name='state_publisher',
        #     output='screen'),
        # RViz2
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] # Load our custom RViz config
        ),
    ])