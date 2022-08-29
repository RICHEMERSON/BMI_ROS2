from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neural_data_generator',
            executable='br_ir_talker',
            name='blackrock_ir_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'timer_period': 0.5},
                {'chn_num': 280},
                {'smoothed_parameter': [10**(i/19-1) for i in range(20)]},
                {'deque_buffer_length': 20}
            ]
        )
    ])
