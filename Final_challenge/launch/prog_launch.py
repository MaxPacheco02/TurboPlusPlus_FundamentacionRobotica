import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('Final_challenge'),
        'config', 
        'params.yaml'
    )

    Final_challenge_node = Node(
        package = 'Final_challenge', 
        executable = 'final_challenge',
        output = 'screen', 
        name='final_challenge',
        parameters = [config] 
    )

    rqt_plot_node = Node(
        package = 'rqt_plot',
        executable = 'rqt_plot',
        output = 'screen',
    )

    l_d = LaunchDescription([Final_challenge_node , rqt_plot_node])
    return l_d