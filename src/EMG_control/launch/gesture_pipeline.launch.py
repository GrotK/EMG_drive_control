from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ścieżka do launcha turtlebota
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch'
    )

    return LaunchDescription([
        # Uruchomienie symulacji TurtleBota 3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_dir, 'turtlebot3_house.launch.py')
            )
        ),

        # EMG publisher
        Node(
            package='EMG_control',
            executable='emg_publisher',
            name='emg_publisher',
            output='screen'
        ),

        # EMG listener
        Node(
            package='EMG_control',
            executable='emg_listener',
            name='emg_listener',
            output='screen'
        ),

        # EMG turtle controller
        Node(
            package='EMG_control',
            executable='emg_turtle_controller',
            name='emg_turtle_controller',
            output='screen'
        )
    ])
