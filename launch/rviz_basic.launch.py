from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name="ros2_turtlebot_can_open"
    ld = LaunchDescription()

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

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2)

    return ld
