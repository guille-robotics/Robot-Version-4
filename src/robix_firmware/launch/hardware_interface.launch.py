import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description=Command (['ros2 param get --hide-type /robot_state_publisher robot_description'])

    #controller_params_file= os.path.join(get_package_share_directory('robix_controller'),'config','robix_controller.yaml')


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    #controller_manager = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    parameters=[
    #        {
    #         "use_sim_time": False},# False porque es para el Robot Real
    #    ],
    #    output="both",
    #)

    return LaunchDescription(
        [
            robot_state_publisher_node
        ]
    )