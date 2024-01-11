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
    xacro_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "urdf", "turtlebot3_burger.urdf.xacro"]
    )

    robot_description = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file]
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "launch", "basic.rviz"]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ],
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'robot_description': robot_description},
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[
            {'robot_description': robot_description},
        ]
    )
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
        rviz2,
        joint_state_publisher_node,
        robot_state_publisher,
    ]

    return LaunchDescription(nodes_to_start)
