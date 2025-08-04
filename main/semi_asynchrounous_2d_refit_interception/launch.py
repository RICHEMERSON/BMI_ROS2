from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,TextSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os
import time

import os
import time
import pickle
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 创建唯一日志目录（带时间戳）
    time_file = '/home/soma/data/bmi_data/leb/' + str(int(time.time()*10)) + '_decoding_element/'
    os.makedirs(time_file, exist_ok=True)
    
    # 2. 声明可修改参数
    arguments = [
        DeclareLaunchArgument('system', default_value='5', description='system ID'),
        DeclareLaunchArgument('group', default_value='0', description='group ID'),
        DeclareLaunchArgument('state', default_value='0', description='state ID'),
        DeclareLaunchArgument('algorithm', default_value='refit_kf_lcy_2d', description='decoding algorithm'),
        DeclareLaunchArgument('log_dir', default_value=time_file, description='log path')
    ]
    
    # 3. 获取参数值
    system = LaunchConfiguration('system')
    group = LaunchConfiguration('group')
    state = LaunchConfiguration('state')
    algorithm = LaunchConfiguration('algorithm')
    log_dir = LaunchConfiguration('log_dir')
    
    # 4. 节点特定日志文件配置
    # integrator_log = [log_dir, TextSubstitution(text='system_'), system, TextSubstitution(text='_group_'), group, TextSubstitution(text='_integrator.log')]
    trainer_log = [log_dir, 'system_', system, '_group_', group, '_trainer.log']
    trainer_buffer_log = [log_dir, 'system_', system, '_group_', group, '_trainer_buffer.log']
    predictor_log = [log_dir, 'system_', system, '_group_', group, '_predictor.log']
    predictor_buffer_log = [log_dir, 'system_', system, '_group_', group, '_predictor_buffer.log']
    integrator_log = PathJoinSubstitution([log_dir,'integrator.log'])
    
    parameters = {}
    parameters['system'] = system
    parameters['group'] = group
    parameters['state'] = state
    parameters['AlignedData'] = 'observation'
    parameters['entry_observation_groups'] = ['/system_{}/passive_observation'.format(str(parameters['system']))]
    parameters['entry_state_groups'] = ['/system_{}/group_{}/state_{}/desired_state'.format(parameters['system'], parameters['group'], parameters['state'])]
    
    # 6. 节点配置（独立日志文件）
    integrator_node = Node(
        package='decoding_element',
        executable='integrator_message_filters',
        name=['system_', system, '_group_', group, '_integrator'],
        parameters=[{
            'system': system,
            'group': group,
            'state':state
        }],
        output='log',
        arguments=['--log-file', integrator_log]
    )

    trainer_buffer_node = Node(
        package='decoding_element',
        executable='trainer_buffer',
        name=['system_', system, '_group_', group, '_trainer_buffer'],
        parameters=[
            {
                'system': system,
                'group': group,
                'state':state
        }
        ],
        output='log',
        arguments=['--log-file', trainer_buffer_log]
    )
    
    trainer_node = Node(
        package='decoding_element',
        executable='trainer',
        name=['system_', system, '_group_', group, '_trainer'],
        parameters=[
            {
                'system': system,
                'group': group,
                'state':state,
                'algorithm': algorithm
        }
        ],
        output='log',
        arguments=['--log-file', trainer_log]
    )
    
    predictor_node = Node(
        package='decoding_element',
        executable='predictor',
        name=['system_', system, '_group_', group, '_predictor'],
        parameters=[{
                'system': system,
                'group': group,
                'state':state,
                'algorithm': algorithm
        }],
        output='log',
        arguments=['--log-file', predictor_log]
    )

    predictor_buffer_node = Node(
        package='decoding_element',
        executable='predictor_buffer',
        name=['system_', system, '_group_', group, '_predictor_buffer'],
        parameters=[{
                'system': system,
                'group': group,
                'state':state
        }],
        output='log',
        arguments=['--log-file', predictor_buffer_log]
    )
    
    # 7. 日志位置提示
    log_info = LogInfo(msg=['所有节点日志存储在: ', log_dir])
    # print(['aaa',log_info])
    log_info = LogInfo(msg=['integrator节点日志存储在: ', integrator_log])
    # log_info = LogInfo(msg=['trainer日志存储在: ', trainer_log])
    # log_info = LogInfo(msg=['predictor节点日志存储在: ', predictor_log])
    
    return LaunchDescription(
        arguments + [
            integrator_node,
            trainer_buffer_node,
            trainer_node,
            predictor_node,
            predictor_buffer_node,
            log_info
        ]
    )