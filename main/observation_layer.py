import os
import subprocess
import json
import time
import pickle
import signal

system_reader = {}
system_reader['blackrock'] = {}
system_reader['blackrock']['passive'] = 'blackrock_ir_talker'

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
    p = subprocess.run(bash_excution,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")

time_dir = '/share/'+str(int(time.time()*10))+'/'
if not os.path.exists(time_dir):
    os.mkdir(time_dir)

log_file = time_dir+'system{}_observation.log'.format(str(parameters['system']))
raise_observation_reader_node('blackrock', 'passive', log_file, **parameters)

