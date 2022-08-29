from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_pkg',
            executable='talker',
            name='aab',
            output='screen',
            emulate_tty=True,
            parameters=[]
        )
    ])
