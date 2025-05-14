from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_world',
            name='gazebo_sim',
            output='screen'
        ),
        Node(
            package='EMG_control',
            executable='offline_publisher',
            name='emg_publisher',
            output='screen'
        ),
        Node(
            package='EMG_control',
            executable='main',
            name='emg_classifier',
            output='screen'
        ),
        Node(
            package='EMG_control',
            executable='gesture_to_cmd',
            name='gesture_controller',
            output='screen'
        )
    ])
