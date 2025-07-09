from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
import json

system_reader = {}
system_reader['blackrock'] = {}
system_reader['blackrock']['passive'] = 'blackrock_ir_talker'

def generate_launch_description():
    parameters = {}
    parameters['timer_period'] = 0.5
    parameters['chn_num'] = 280
    parameters['deque_buffer_length'] = 20
    parameters['smoothed_parameter'] = [10**(i/19-1) for i in range(20)]
    parameters['system'] = 0
    parameters['time_buffer_length'] = 1

    return LaunchDescription([
        Node(
            package='observation_reader',
            executable=system_reader['blackrock']['passive'],
            name='system{}_BlackrockPassiveIrPublisher'.format(str(parameters['system'])),
            output='screen',
            emulate_tty=True,
            parameters=[
                {'parameters': json.dumps(parameters)}
            ]
        )
    ])
