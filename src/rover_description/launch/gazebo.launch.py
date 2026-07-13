from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_directory('rover_description'), 'urdf', 'rover.urdf.xacro')
    robot_desc = xacro.process_file(urdf_path).toxml()
    world_path = os.path.join(get_package_share_directory('rover_description'), 'worlds', 'rescue_world.sdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r ' + world_path}.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.1'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(get_package_share_directory('rover_description'), 'config', 'bridge.yaml')}],
            output='screen'
        )


    ])




