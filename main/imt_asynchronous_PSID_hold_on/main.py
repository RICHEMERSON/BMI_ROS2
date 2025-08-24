import os
import subprocess
import time
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BASE_LOG = '/home/soma/data/bmi_data/leb'
os.makedirs(BASE_LOG, exist_ok=True)

# Session folder
session_dir = os.path.join(BASE_LOG, f"{int(time.time()*10)}_imt_hold_on")
os.makedirs(session_dir, exist_ok=True)

obs_log = os.path.join(session_dir, 'observation.log')
decode_log = os.path.join(session_dir, 'decoding_element.log')
state_log_tpl = os.path.join(BASE_LOG, '$(date +%Y%m%d)_$(date +%H%M%S)_behavior.log')

# Parameters (tweak as needed)
system = 0
group = 0
state = 0
algorithm = 'refit_kf_lcy_2d'

if __name__ == '__main__':
    # 1) Observation reader
    obs_py = os.path.join(HERE, 'observation_layer.py')
    obs_p = subprocess.Popen([sys.executable, obs_py], stdout=subprocess.DEVNULL, stderr=open(obs_log, 'w'))
    time.sleep(1.0)

    # 2) Decoding pipeline via ROS2 launch (includes integrator/trainer/predictor and buffers)
    launch_file = os.path.join(HERE, 'launch.py')
    launch_cmd = [
        'ros2', 'launch', launch_file,
        f"system:={system}", f"group:={group}", f"state:={state}", f"algorithm:={algorithm}"
    ]
    dec_p = subprocess.Popen(' '.join(launch_cmd), shell=True, stdout=subprocess.DEVNULL, stderr=open(decode_log, 'w'))
    time.sleep(2.0)

    # 3) State/behavior presenter using BMI (calls /predictor/decoding_service)
    # Choose a presenter that already uses decoding service
    state_cmd = f'ros2 run state_reader psychopy_2D_interception 2>{state_log_tpl}'
    st_p = subprocess.Popen(state_cmd, shell=True, stdout=subprocess.DEVNULL)

    # Wait for processes (presenter usually controls lifetime)
    st_p.wait()
    dec_p.terminate()
    obs_p.terminate()
