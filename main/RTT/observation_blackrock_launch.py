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
        DeclareLaunchArgument('time_buffer_length', default_value='1'),
        DeclareLaunchArgument('chn_num', default_value='256'),
        DeclareLaunchArgument('deque_buffer_length', default_value='20'),
        DeclareLaunchArgument('smoothed_parameter', default_value=str([10**(i/19-1) for i in range(20)])),
        DeclareLaunchArgument('timer_period', default_value='0.02'),
        DeclareLaunchArgument('subcriber', default_value='blackrock_ir_talker'),
        DeclareLaunchArgument('log_dir', default_value=time_file)
    ]
    
    # 3. 获取参数值
    system = LaunchConfiguration('system')
    time_buffer_length = LaunchConfiguration('time_buffer_length')
    chn_num = LaunchConfiguration('chn_num')
    smoothed_parameter = LaunchConfiguration('smoothed_parameter')
    timer_period = LaunchConfiguration('timer_period')
    subcriber = LaunchConfiguration('subcriber')
    deque_buffer_length = LaunchConfiguration('deque_buffer_length')
    log_dir = LaunchConfiguration('log_dir')
    
    observation_log = PathJoinSubstitution([log_dir,'observation.log'])

    observation_node = Node(
        package='observation_reader',
        executable=subcriber,
        name=['system_', system, '_BlackrockPassiveIrPublisher'],
        parameters=[{
                'system': system,
                'time_buffer_length': time_buffer_length,
                'chn_num':chn_num,
                'smoothed_parameter': smoothed_parameter,
                'timer_period' : timer_period,
                'deque_buffer_length' : deque_buffer_length 
        }],
        output='log',
        arguments=['--log-file', observation_log]
    )
    
    # 7. 日志位置提示
    log_info = LogInfo(msg=['所有节点日志存储在: ', log_dir])
    # print(['aaa',log_info])
    # log_info = LogInfo(msg=['integrator节点日志存储在: ', integrator_log])
    # log_info = LogInfo(msg=['trainer日志存储在: ', trainer_log])
    # log_info = LogInfo(msg=['predictor节点日志存储在: ', predictor_log])


    
    return LaunchDescription(
        arguments + [
            observation_node,
            log_info
        ]
    )