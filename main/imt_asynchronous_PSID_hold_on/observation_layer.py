import os
import subprocess
import time
import pickle

# Linux-friendly base data directory
BASE_DIR = '/home/soma/data/bmi_data/leb'
os.makedirs(BASE_DIR, exist_ok=True)

parameters = {
    'timer_period': 0.02,
    'chn_num': 256,
    'deque_buffer_length': 20,
    'smoothed_parameter': [10**(i/19-1) for i in range(20)],
    'system': 0,
    'time_buffer_length': 1,
}

SYSTEM_READER = {
    'blackrock': {
        'passive': 'blackrock_ir_talker'
    }
}

def raise_observation_reader_node(recording_system: str, node_mode: str, log_file: str, **kwargs):
    cmd = f"ros2 run observation_reader {SYSTEM_READER[recording_system][node_mode]} "
    node_name = f"--ros-args -r __node:=system{parameters['system']}_BlackrockPassiveIrPublisher "
    # match existing pattern: pass pickled parameters array
    par = ("--param " + "parameters:={}".format(list(pickle.dumps(kwargs)))).replace(' ', '')
    full = cmd + node_name + par + f" 2>{log_file}"
    return subprocess.Popen(full, shell=True)

# per-run log dir
run_dir = os.path.join(BASE_DIR, f"{int(time.time()*10)}_observation")
os.makedirs(run_dir, exist_ok=True)
log_file = os.path.join(run_dir, f"system{parameters['system']}_observation.log")

if __name__ == '__main__':
    raise_observation_reader_node('blackrock', 'passive', log_file, **parameters)
