import os
import subprocess
import json
import time
import pickle
import signal
import os
import re
from datetime import datetime

def generate_next_folder(base_path):
    """
    在指定路径下生成符合格式的递增文件夹名称
    :param base_path: 基础路径
    :return: 新文件夹完整路径
    """
    # 获取当前日期（格式：yyyymmdd）
    today = datetime.now().strftime("%Y%m%d")
    
    # 定义文件夹名称正则表达式（匹配：日期_bmi_三位数字）
    pattern = re.compile(fr'^{today}_interception_(\d{{3}})$')
    
    # 遍历目标目录下的所有文件夹
    max_num = 0
    for folder_name in os.listdir(base_path):
        folder_path = os.path.join(base_path, folder_name)
        
        # 只处理目录
        if os.path.isdir(folder_path):
            match = pattern.match(folder_name)
            if match:
                # 提取并比较序号
                current_num = int(match.group(1))
                if current_num > max_num:
                    max_num = current_num

    # 计算新序号（自动补零到3位）
    new_num = max_num + 1
    new_folder_name = f"{today}_interception_{new_num:03d}"
    new_folder_path = os.path.join(base_path, new_folder_name)

    # 检查是否存在并创建目录
    try:
        if not os.path.exists(new_folder_path):
            os.makedirs(new_folder_path)
            print(f"成功创建文件夹: {new_folder_path}")
        else:
            print(f"文件夹已存在: {new_folder_path}")
    except PermissionError:
        print(f"错误：无权限创建文件夹 {new_folder_path}")
    except Exception as e:
        print(f"创建文件夹时发生错误: {str(e)}")
    
    return new_folder_path

target_path = '/home/soma/data/bmi_data/leb'
if os.path.exists(target_path):
    folder_name = generate_next_folder(target_path)
else:
    print("错误：指定路径不存在")

save_path = os.path.join(target_path,folder_name)

system_reader = {}
system_reader['blackrock'] = {}
system_reader['blackrock']['passive'] = 'blackrock_ir_talker'
# system_reader['blackrock']['passive'] = 'blackrock_ir_talker_count'

parameters = {}
parameters['timer_period'] = 0.02
parameters['chn_num'] = 256
parameters['deque_buffer_length'] = 20
parameters['smoothed_parameter'] = [10**(i/19-1) for i in range(20)]
parameters['system'] = 0
parameters['time_buffer_length'] = 1

def raise_observation_reader_node(recording_system, node_mode, log_file, **kwargs):
 
    bash_excution = 'ros2 run observation_reader {} '.format(system_reader[recording_system][node_mode])
    node_name = '--ros-args -r __node:=system{}_BlackrockPassiveIrPublisher '.format(str(parameters['system']))
    par = "--param "+"parameters:={}".format(list(pickle.dumps(kwargs))).replace(' ', '')
    bash_excution = bash_excution + node_name + par+' 2>{}'.format(log_file)
    p = subprocess.Popen(bash_excution,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")
    return p
time_dir = save_path+'/'+str(int(time.time()*10))+'_observation'+'/'
if not os.path.exists(time_dir):
    os.mkdir(time_dir)

log_file = time_dir+'system{}_observation.log'.format(str(parameters['system']))
observe_p = raise_observation_reader_node('blackrock', 'passive', log_file, **parameters)
time.sleep(1)
print('observation start')


time_file = save_path + '/'+str(int(time.time()*10))+'_decoding_element'+'/'
if not os.path.exists(time_file):
    os.mkdir(time_file)

parameters = {}
parameters['system'] = 0
parameters['group'] = 0
parameters['state'] = 0
parameters['AlignedData'] = 'observation'
parameters['entry_observation_groups'] = ['/system_{}/passive_observation'.format(str(parameters['system']))]
parameters['entry_state_groups'] = ['/system_{}/group_{}/state_{}/desired_state'.format(parameters['system'], parameters['group'], parameters['state'])]

def raise_data_integrator_node(**kwargs):
    
    log_file = time_file+'system{}_group{}_integrator.log'.format(str(parameters['system']), str(parameters['group']))
    bash_excution = 'ros2 run decoding_element integrator '
    # bash_excution = 'ros2 run decoding_element integrator_message_filters '
    node_name = '--ros-args -r __node:=system{}_group{}_integrator '.format(str(parameters['system']), str(parameters['group']))
    par = "--param "+"parameters:={} ".format(list(pickle.dumps(kwargs))).replace(' ', '')
    bash_excution = bash_excution + node_name + par+' 2>{}'.format(log_file)
    p = subprocess.Popen(bash_excution,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")
    
    return p

integrator_p = raise_data_integrator_node(**parameters)


parameters = {}
parameters['system'] = 0
parameters['group'] = 0
parameters['decoding_element'] = 'decoder'
parameters['algorithm'] = 'refit_kf_lcy_2d'
# parameters['algorithm'] = 'PSID_2D'
# parameters['algorithm'] = 'NDT2D'

def raise_trainer_node(**kwargs):
    
    log_file = time_file+'system{}_group{}_trainer.log'.format(str(parameters['system']), str(parameters['group']))
    bash_excution = 'ros2 run decoding_element trainer '
    node_name = '--ros-args -r __node:=system{}_group{}_trainer '.format(str(parameters['system']), str(parameters['group']))
    par = "--param "+"parameters:={} ".format(list(pickle.dumps(kwargs))).replace(' ', '')
    bash_excution = bash_excution + node_name + par+' 2>{}'.format(log_file)
    p = subprocess.Popen(bash_excution,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")
    
    return p

trainer_p = raise_trainer_node(**parameters)

parameters = {}
parameters['system'] = 0
parameters['group'] = 0
parameters['wait'] = False
parameters['algorithm'] = 'refit_kf_lcy_2d'
# parameters['algorithm'] = 'PSID_2D'
# parameters['algorithm'] = 'NDT2D'

def raise_predictor_node(**kwargs):
    
    log_file = time_file+'system{}_group{}_predictor.log'.format(str(parameters['system']), str(parameters['group']))
    bash_excution = 'ros2 run decoding_element predictor '
    node_name = '--ros-args -r __node:=system{}_group{}_predictor '.format(str(parameters['system']), str(parameters['group']))
    par = "--param "+"parameters:={} ".format(list(pickle.dumps(kwargs))).replace(' ', '')
    bash_excution = bash_excution + node_name + par+' 2>{}'.format(log_file)
    p = subprocess.Popen(bash_excution,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")
    
    return p

predictor_p = raise_predictor_node(**parameters)

time.time(1)

bash_excution = f'ros2 run state_reader psychopy_2D_semi_interception 2>{save_path}/$(date +%Y%m%d)_$(date +%H%M%S)_behavior.log'
state_p = subprocess.Popen(bash_excution,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")

observe_p.wait()
trainer_p.wait()
integrator_p.wait()
predictor_p.wait()
state_p.wait()