import os
import subprocess
import json
import time
import pickle
import signal

time_file = '/share/'+str(int(time.time()*10))+'_decoding_element'+'/'
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
parameters['algorithm'] = 'PSID'

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
        
def raise_predictor_node(**kwargs):
    
    log_file = time_file+'system{}_group{}_predictor.log'.format(str(parameters['system']), str(parameters['group']))
    bash_excution = 'ros2 run decoding_element predictor '
    node_name = '--ros-args -r __node:=system{}_group{}_predictor '.format(str(parameters['system']), str(parameters['group']))
    par = "--param "+"parameters:={} ".format(list(pickle.dumps(kwargs))).replace(' ', '')
    bash_excution = bash_excution + node_name + par+' 2>{}'.format(log_file)
    p = subprocess.Popen(bash_excution,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")
    
    return p

predictor_p = raise_predictor_node(**parameters)

trainer_p.wait()
integrator_p.wait()
predictor_p.wait()
