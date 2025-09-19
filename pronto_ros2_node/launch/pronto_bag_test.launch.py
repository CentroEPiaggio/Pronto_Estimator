from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    

    xacro_pkg_value = LaunchConfiguration("xacro_pkg")
    xacro_name_value = LaunchConfiguration("xacro_name")
    config_name_value = LaunchConfiguration("config_name")
    node_name_value = LaunchConfiguration("node_name", default="pronto_node")
    topic_name_value = LaunchConfiguration("topic_name", default="pose_est")
    v_topic_name_value = LaunchConfiguration("v_topic_name", default="twist_est")
    bag_name = LaunchConfiguration(
        "bag_name",
        default='2024-07-23/bags/2024_07_23_10_25_45-040_trot_forward/2024_07_23_10_25_45-040_trot_forward_0.mcap'
    )

    xacro_pkg_arg = DeclareLaunchArgument("xacro_pkg", default_value="pronto_ros2_node")
    xacro_name_arg = DeclareLaunchArgument("xacro_name", default_value="mulinex")
    config_name_arg = DeclareLaunchArgument("config_name", default_value="pronto_mulinex.yaml")
    node_name_arg = DeclareLaunchArgument("node_name", default_value="pronto_node")
    topic_name_arg = DeclareLaunchArgument("topic_name", default_value="pose_est")
    v_topic_name_arg = DeclareLaunchArgument("topic_name", default_value="twist_est")
    bag_name_arg = DeclareLaunchArgument(
        "bag_name",
        default_value='2024-07-23/bags/2024_07_23_10_25_45-040_trot_forward/2024_07_23_10_25_45-040_trot_forward_0.mcap'
    )
    
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare(xacro_pkg_value),
       'urdf', xacro_name_value
    ])
    config_file_path = PathJoinSubstitution([
        FindPackageShare('pronto_ros2_node'),
        'config', config_name_value
    ])
    bag_path = '/home/ros/docker_pronto_ws/bags/'
    bag_fullpath = PathJoinSubstitution([bag_path, bag_name])
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ======================================================================= #

    return LaunchDescription([
        xacro_name_arg,
        xacro_pkg_arg,
        config_name_arg,
        node_name_arg,
        topic_name_arg,
        v_topic_name_arg,
        bag_name_arg,
        Node(
            package='pronto_ros2_node',
            executable='pronto_node',
            name=node_name_value,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'urdf_file': ParameterValue(Command(['xacro ', xacro_file_path]), value_type=str)},
                {'pose_topic': ParameterValue(topic_name_value, value_type=str)},
                {'twist_topic': ParameterValue(v_topic_name_value, value_type=str)},
                config_file_path
            ],
        ),
        ExecuteProcess(
            cmd=['ros2 bag play', bag_fullpath],
            output='screen',
            shell=True,
        ),
    ])
