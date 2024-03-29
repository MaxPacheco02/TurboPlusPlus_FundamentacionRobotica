import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_dexterous = get_package_share_directory('dexterous')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
            pkg_dexterous,
            'worlds',
            'dexterous_world.sdf',
        ])}.items()
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                   '/j_f1@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f1_1@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f1_1_1@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f2@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f2_1@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f2_1_1@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f3@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f3_1@std_msgs/msg/Float64@gz.msgs.Double',
                   '/j_f3_1_1@std_msgs/msg/Float64@gz.msgs.Double',
                   ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
    ])