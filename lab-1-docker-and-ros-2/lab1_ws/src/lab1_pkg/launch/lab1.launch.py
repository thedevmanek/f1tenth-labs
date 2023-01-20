from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker.py',
            name='talker',
            parameters=[
                {'v':2.0},
                {'d':2.9}
            ],
            output="screen",
            emulate_tty=True,
        ),
         Node(
            package='lab1_pkg',
            executable='relay.py',
            name='relay',
            output="screen",
            emulate_tty=True,
        )
    ])