from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='the_zucc',
            executable='go',
            name='processWaypoints',
            output='screen'
        ),
        Node(
            package='the_zucc',
            executable='drive',
            name='goToGoal',
            output='screen'
        ),
        Node(
            package='the_zucc',
            executable='avoid',
            name='getObjectRange',
            output='screen'
        )
    ])
