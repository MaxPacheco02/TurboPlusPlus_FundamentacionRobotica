from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='xarm',
        #     executable='trajectory_planner.py',
        # ),
        Node(
            name="foxglove_bridge",
            package="foxglove_bridge",
            executable="foxglove_bridge"
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xarm_planner'),
                'launch',
                'follow.launch.py'
            ])
        ]),
    )
    ])