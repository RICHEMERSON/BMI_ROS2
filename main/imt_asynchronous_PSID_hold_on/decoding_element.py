import os
import subprocess
import time
import pickle

# Linux data dir
BASE_DIR = '/home/soma/data/bmi_data/leb'
os.makedirs(BASE_DIR, exist_ok=True)

ALG = 'refit_kf_lcy_2d'

# integrator params
integrator_params = {
    'system': 0,
    'group': 0,
    'state': 0,
}

# trainer params
trainer_params = {
    'system': 0,
    'group': 0,
    'state': 0,
    'algorithm': ALG,
}

# predictor params
predictor_params = {
    'system': 0,
    'group': 0,
    'state': 0,
    'wait': False,
    'algorithm': ALG,
}

def run_node(pkg_exec: str, node_label: str, params: dict, log_path: str):
    cmd = f"ros2 run decoding_element {pkg_exec} "
    node_name = f"--ros-args -r __node:=system{params['system']}_group{params['group']}_{node_label} "
    par = ("--param " + "parameters:{}".format(list(pickle.dumps(params)))).replace(' ', '')
    full = cmd + node_name + par + f" 2>{log_path}"
    return subprocess.Popen(full, shell=True)

run_dir = os.path.join(BASE_DIR, f"{int(time.time()*10)}_decoding_element")
os.makedirs(run_dir, exist_ok=True)

if __name__ == '__main__':
    p_integrator = run_node('integrator', 'integrator', integrator_params, os.path.join(run_dir, 'integrator.log'))
    p_trainer = run_node('trainer', 'trainer', trainer_params, os.path.join(run_dir, 'trainer.log'))
    p_predictor = run_node('predictor', 'predictor', predictor_params, os.path.join(run_dir, 'predictor.log'))

    p_trainer.wait()
    p_integrator.wait()
    p_predictor.wait()
