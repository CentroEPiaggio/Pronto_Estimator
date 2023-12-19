import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    # every parameter needs only the file name 
    # EXCEPT FOR URDF: pronto needs the entire urdf file path

    package_path = get_package_share_path("pronto_estimator_quadruped")

    rviz_config = LaunchConfiguration('rviz_config_file', default="pronto.rviz")
    rviz_config_path = PathJoinSubstitution([os.path.join(package_path, 'rviz'), rviz_config])

    estimator_config = LaunchConfiguration('estimator_config', default="pronto_estimator.yaml")
    estimator_config_path = PathJoinSubstitution([os.path.join(package_path, "config"), estimator_config])

    use_rviz = LaunchConfiguration('use_rviz', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    urdf_path = LaunchConfiguration('urdf_path', default="")


    return LaunchDescription([

        DeclareLaunchArgument('urdf_path', default_value=urdf_path),

        DeclareLaunchArgument('use_rviz', default_value='False'),

        # Launch Pronto node with parameters
        Node(
            package='pronto_estimator_quadruped',
            namespace='', # working only without namespace...why?
            executable='pronto_estimator_node',
            name='pronto_estimator',
            parameters=[estimator_config_path,
                        {'urdf_path' : LaunchConfiguration('urdf_path')}],  
            # prefix=['gdbserver localhost:3000'], # debug option
            output='screen'
        ),

        # Launch Rviz
        Node(
            condition = IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output= 'screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d',rviz_config_path],
        ),

    ])
