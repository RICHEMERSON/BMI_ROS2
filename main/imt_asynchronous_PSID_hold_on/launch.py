from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os, time


def generate_launch_description():
    # log dir (Linux path like other scenarios)
    base_dir = '/home/soma/data/bmi_data/leb'
    time_file = os.path.join(base_dir, f"{int(time.time()*10)}_imt_hold_on")
    os.makedirs(time_file, exist_ok=True)

    args = [
        DeclareLaunchArgument('system', default_value='0'),
        DeclareLaunchArgument('group', default_value='0'),
        DeclareLaunchArgument('state', default_value='0'),
        DeclareLaunchArgument('algorithm', default_value='refit_kf_lcy_2d'),
    ]

    system = LaunchConfiguration('system')
    group = LaunchConfiguration('group')
    state = LaunchConfiguration('state')
    algorithm = LaunchConfiguration('algorithm')

    nodes = []
    nodes.append(Node(
        package='decoding_element', executable='integrator_message_filters',
        name=['system_', system, '_group_', group, '_integrator'],
        parameters=[{'system': system, 'group': group, 'state': state}],
        output='log', arguments=['--log-file', os.path.join(time_file, 'integrator.log')]
    ))

    nodes.append(Node(
        package='decoding_element', executable='trainer_buffer',
        name=['system_', system, '_group_', group, '_trainer_buffer'],
        parameters=[{'system': system, 'group': group, 'state': state}],
        output='log', arguments=['--log-file', os.path.join(time_file, 'trainer_buffer.log')]
    ))

    nodes.append(Node(
        package='decoding_element', executable='trainer',
        name=['system_', system, '_group_', group, '_trainer'],
        parameters=[{'system': system, 'group': group, 'state': state, 'algorithm': algorithm}],
        output='log', arguments=['--log-file', os.path.join(time_file, 'trainer.log')]
    ))

    nodes.append(Node(
        package='decoding_element', executable='predictor',
        name=['system_', system, '_group_', group, '_predictor'],
        parameters=[{'system': system, 'group': group, 'state': state, 'algorithm': algorithm}],
        output='log', arguments=['--log-file', os.path.join(time_file, 'predictor.log')]
    ))

    nodes.append(Node(
        package='decoding_element', executable='predictor_buffer',
        name=['system_', system, '_group_', group, '_predictor_buffer'],
        parameters=[{'system': system, 'group': group, 'state': state}],
        output='log', arguments=['--log-file', os.path.join(time_file, 'predictor_buffer.log')]
    ))

    return LaunchDescription(args + nodes + [LogInfo(msg=['logs: ', time_file])])
