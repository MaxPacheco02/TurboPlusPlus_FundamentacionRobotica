from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='courseworks_week2',
            executable='signal_generator.py',
        ),
        Node(
            package='courseworks_week2',
            executable='reconstruction.py',
        ),
        Node(
            name="foxglove_bridge",
            package="foxglove_bridge",
            executable="foxglove_bridge"
        ),
    ])