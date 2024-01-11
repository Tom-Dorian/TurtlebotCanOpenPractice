from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = "ros2_turtlebot_can_open"

    slave_config_n5 = PathJoinSubstitution(
        [FindPackageShare(package_name), "config/Turtlebot_PoistionControl", "cia402_slave_n5.eds"]
    )

    slave_config_n6 = PathJoinSubstitution(
        [FindPackageShare(package_name), "config/Turtlebot_PoistionControl", "cia402_slave_n6.eds"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "5",
            "node_name": "left_wheel_slave",
            "slave_config": slave_config_n5,
        }.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "6",
            "node_name": "right_wheel_slave",
            "slave_config": slave_config_n6,
        }.items(),
    )

    nodes_to_start = [
        slave_node_1,
        slave_node_2,
    ]

    return LaunchDescription(nodes_to_start)
