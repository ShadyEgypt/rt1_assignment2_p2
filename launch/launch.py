import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Path to the gazebo launch file
    gazebo_launch_file = os.path.join(
        FindPackageShare('robot_urdf').find('robot_urdf'),
        'launch',
        'gazebo.launch.py'
    )

    # Include the gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # Launch the controller node
    controller_node = Node(
        package='urdf_robot_controller',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    return launch.LaunchDescription([
        gazebo_launch,
        controller_node
    ])
