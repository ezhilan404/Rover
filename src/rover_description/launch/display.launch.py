from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_directory('rover_description'), 'urdf', 'rover.urdf.xacro')
    robot_desc = xacro.process_file(urdf_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', os.path.join(get_package_share_directory('rover_description'), 'rviz', 'view_robot.rviz')]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        )

    ])
