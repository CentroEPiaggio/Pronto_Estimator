import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import  LaunchConfiguration,PathJoinSubstitution,TextSubstitution
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
import subprocess
from launch.actions import DeclareLaunchArgument,EmitEvent, RegisterEventHandler,ExecuteProcess
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from datetime import datetime


def generate_launch_description():
    
    exp_name_value = LaunchConfiguration("exp_name")
    package_path = FindPackageShare("pronto_tuning")
    exp_name_arg = DeclareLaunchArgument("exp_name",default_value="Exp_1_cmd.yaml")


    #start qualisys
    qualysis = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("qualisys_driver")
                        , 'launch', 'qualisys.launch.py'])
                ))
    
    bag_base_path = "/home/ros/docker_pronto_ws/bags/"
    #starts controller 
    omni_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["omni_controller", "--controller-manager", "/controller_manager"],
    )   

    ordered_omni_controller_spawner = TimerAction(actions=[omni_controller_spawner],period=1.0)

    state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["state_broadcaster", "--controller-manager", "/controller_manager"],
        )   

    ordered_state_broadcaster_spawner = RegisterEventHandler(
        	
        OnProcessExit(target_action=omni_controller_spawner, on_exit=[state_broadcaster_spawner])
    )

    IMU_Broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["IMU_Broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    ordered_IMU_Broadcaster_spawner = RegisterEventHandler(
        	
        OnProcessExit(target_action=state_broadcaster_spawner, on_exit=[IMU_Broadcaster_spawner])
    )

    rf2o_odom = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("rf2o")
                        , 'launch', 'rf2o_laser_odometry.launch.py'])
                ))
    ordered_rf2o_odom = RegisterEventHandler(
        	
        OnProcessExit(target_action=IMU_Broadcaster_spawner, on_exit=[rf2o_odom])
    )

    #start bag 
    time = datetime.now()

    time_str = time.strftime("%Y_%m_%d_%H_%M_%S")
    
    bag_path = bag_base_path + "_Exp_" +time_str
    
    start_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','-a','-s','mcap', '-o', bag_path ],
     output='screen'
    )

    command_launch  = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("pronto_tuning")
                        , 'launch', 'command_tuning.launch.py'])
                ),
                launch_arguments={
                    "exp_name": exp_name_value
                }.items()
            )

    delayed_start_exp = TimerAction(
        actions=[command_launch], period=5.0
    )
    delayed_start = TimerAction(
        actions=[delayed_start_exp,start_bag], period=5.0
    )
    


    # ordered_command_launch = RegisterEventHandler(
        	
    #     OnProcessExit(target_action=IMU_Broadcaster_spawner, on_exit=[command_launch])
    # )
    
    
    return LaunchDescription([
        qualysis,
        exp_name_arg,
        ordered_omni_controller_spawner,
        ordered_state_broadcaster_spawner,
        ordered_IMU_Broadcaster_spawner,
        ordered_rf2o_odom,
        delayed_start
        # ordered_start_bag
        # ordered_command_launch
        # start_bag
    ])

