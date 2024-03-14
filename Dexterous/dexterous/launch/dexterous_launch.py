from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    dexterous_path = get_package_share_path('dexterous')
    hand_model_path = dexterous_path / 'urdf/hand.urdf'
    # pencil_model_path = dexterous_path / 'urdf/pencil.urdf'
    default_rviz_config_path = dexterous_path / 'rviz/urdf.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    hand_model_arg = DeclareLaunchArgument(name='hand_robot', default_value=str(hand_model_path),
                                      description='Absolute path to robot urdf file')
    # pencil_model_arg = DeclareLaunchArgument(name='pencil_model', default_value=str(pencil_model_path),
    #                                   description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description_hand = ParameterValue(Command(['xacro ', LaunchConfiguration('hand_robot')]),
                                       value_type=str)
    # robot_description_pencil = ParameterValue(Command(['xacro ', LaunchConfiguration('pencil_model')]),
    #                                    value_type=str)

    hand_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_hand}]
    )

    # pencil_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     namespace='pencil_robot',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_pencil}]
    # )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ["/j_s"]}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    foxglove_node = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge"
    )


    return LaunchDescription([
        gui_arg,
        hand_model_arg,
        # pencil_model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        hand_state_publisher_node,
        # pencil_state_publisher_node,
        rviz_node,
        foxglove_node,
    ])
