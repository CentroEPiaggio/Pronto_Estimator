import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    

    xacro_pkg_value = LaunchConfiguration("xacro_pkg")
    xacro_name_value = LaunchConfiguration("xacro_name")
    config_name_value = LaunchConfiguration("config_name")

    package_share_path = FindPackageShare('pronto_ros2_node')

    xacro_pkg_arg = DeclareLaunchArgument("xacro_pkg",default_value="pronto_ros2_node")
    xacro_name_arg = DeclareLaunchArgument("xacro_name",default_value="mulinex")
    config_name_arg = DeclareLaunchArgument("config_name",default_value="pronto_mulinex.yaml")
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare(xacro_pkg_value),
       'urdf', xacro_name_value
    ])
    config_file_path = PathJoinSubstitution([
        package_share_path,
        'config',config_name_value
    ])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ======================================================================== #

    return LaunchDescription([
        
        xacro_name_arg,
        xacro_pkg_arg,
        config_name_arg,
        # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
        Node(
            package='pronto_ros2_node',
            executable='pronto_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'urdf_file': ParameterValue(Command(['xacro', ' ' ,xacro_file_path]), value_type=str)},
                config_file_path
            ],
        ),
        
        # # Launch RViz
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', config_file_path],
        # )
    ])