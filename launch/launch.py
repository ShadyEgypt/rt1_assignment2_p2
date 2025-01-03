import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    gazebo_launch_file = os.path.join(
        FindPackageShare('robot_urdf').find('robot_urdf'),
        'launch',
        'gazebo.launch.py'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    controller_node = Node(
        package='rt1_assignment2_p2',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    return launch.LaunchDescription([
        gazebo_launch,
        controller_node
    ])
