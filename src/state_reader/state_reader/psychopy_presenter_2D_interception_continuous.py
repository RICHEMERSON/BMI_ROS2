# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import numpy as np
import time
from interfaces.msg import State
from interfaces.srv import DecodingService
from collections import deque
from queue import Queue
from psychopy import visual, core
from psychopy import data
import pickle
from multiprocessing import shared_memory
import datetime
import os
import json
import serial
from uldaq import (get_daq_device_inventory, DaqDevice, 
                   InterfaceType, Range, AOutFlag, DigitalPortType, DigitalDirection)

devices = get_daq_device_inventory(InterfaceType.USB)
if not devices:
    raise RuntimeError('No DAQ devices found')

daq_device = DaqDevice(devices[0])
daq_device.connect()

# # Get AoDevice and DioDevice objects
ao_device = daq_device.get_ao_device()
dio_device = daq_device.get_dio_device()

# # Configure the AUXPORT as an output port
voltage=5
dio_device.d_config_port(DigitalPortType.AUXPORT, DigitalDirection.OUTPUT)
ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=0) 

#%% set udp protocal
import socket
bhvip='192.168.137.3'
bhvport=8866
bhvaddr=(bhvip,bhvport)
udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp_socket.settimeout(None)

#%% task parameter
SurroundRadius = 10
TouchErr = 3
mywin = visual.Window(size=(1280, 1024),monitor='testMonitor',color=(-1, -1, -1), units="cm",screen=1,fullscr=True)


## object statement map
object_map = {}
object_map['circle'] = visual.Circle

## object configuration
object_dict = {}
object_dict['circle'] = {}
object_dict['circle']['SurroundTarget'] = {}
object_dict['circle']['SurroundTarget']['size'] = 3
object_dict['circle']['SurroundTarget']['pos'] = [0,10]
object_dict['circle']['SurroundTarget']['lineColor'] = (-1,-1,-1)
object_dict['circle']['SurroundTarget']['fillColor'] = (-1,1,-1)

object_dict['circle']['Cursor'] = {}
object_dict['circle']['Cursor']['size'] = 1
object_dict['circle']['Cursor']['pos'] = [0,0]
object_dict['circle']['Cursor']['lineColor'] = (-1,-1,-1)
object_dict['circle']['Cursor']['fillColor'] = (-1,-1,-1)

object_dict['circle']['Ring'] = {}
object_dict['circle']['Ring']['size'] = 4
object_dict['circle']['Ring']['pos'] = [0,0]
object_dict['circle']['Ring']['lineColor'] = (1,1,1)
object_dict['circle']['Ring']['fillColor'] = (-1,-1,-1)
object_dict['circle']['Ring']['lineWidth'] = 10

object_instance = {}
for object_type in object_dict:
    for object_name in object_dict[object_type]:
        assert not object_name in object_instance, 'object name should only appear once'
        object_instance[object_name] = object_map[object_type](
        win=mywin, name = object_name, **object_dict[object_type][object_name]
        )

## task configuration
# conds = data.createFactorialTrialList({'Reach Direction':np.array([0, 1/4, 1/2, 3/4, 1, 5/4, 3/2, 7/4])*np.pi})
r_sq2 = 6
conds = data.createFactorialTrialList({'Reach con':[[0,r_sq2],[r_sq2,0],[0,-r_sq2],[-r_sq2,0],[r_sq2,r_sq2],
                                                    [r_sq2,-r_sq2],[-r_sq2,-r_sq2],[-r_sq2,r_sq2]]})
BMIExp = data.TrialHandler(trialList=conds, nReps=1000, name='Calibrate',method='random',
                           dataTypes = ['Condition','EventMarker','TrialStartTime','AnalogData',
                                        'UserVars','TrialError'])
UserVars = {}
UserVars['TargetPos'] = []

## set timer
WithinTrailTimer = core.Clock()
TaskTimer = core.Clock()
TrialTimeCounter = core.Clock()
BellCurveCounter = core.Clock()

#%% initialize shared memory parameter
sm_dict = {}
sm_name = ['AssistCoefficient', 'FFCoefficient', 'DecoderCoefficient', 'TrainingFlag', 'MovingTargetFlag', 'GenerationSpeed', 'AssistCoefficient1', 'AssistCoefficient2', 'RingFlag', 'TargetSpeed']
initialize_value = [6,1,0,1,1,0,3.5,0.5,0,0,1/12]
#%% initialize shared memory parameter
try:  
    smooth_coefficient = shared_memory.ShareableList([1],name='SmoothCoefficient') 
except FileExistsError:         
    smooth_coefficient = shared_memory.ShareableList(name='SmoothCoefficient')         
    smooth_coefficient.shm.unlink()          
    smooth_coefficient = shared_memory.ShareableList([1],name='SmoothCoefficient')

try:
    
    assist_coefficient = shared_memory.ShareableList([6],name='AssistCoefficient')
    ff_coefficient = shared_memory.ShareableList([1],name='FFCoefficient')
    decoder_coefficient = shared_memory.ShareableList([0.3],name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList([1],name='TrainingFlag')
    PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    moving_target_flag = shared_memory.ShareableList([0],name='MovingTargetFlag')
    
    GenerationSpeed = shared_memory.ShareableList([2.818],name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList([1],name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList([0],name='AssistCoefficient2')
    ring_flag = shared_memory.ShareableList([0],name='RingFlag')
    target_speed = shared_memory.ShareableList([1/12],name='TargetSpeed')

except FileExistsError:
    
    assist_coefficient = shared_memory.ShareableList(name='AssistCoefficient')
    decoder_coefficient = shared_memory.ShareableList(name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList(name='TrainingFlag')
    moving_target_flag = shared_memory.ShareableList(name='MovingTargetFlag')
    ff_coefficient = shared_memory.ShareableList(name='FFCoefficient')
    PosFlag = shared_memory.ShareableList(name='PosFlag')
    ring_flag = shared_memory.ShareableList(name='RingFlag')
    
    GenerationSpeed = shared_memory.ShareableList(name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList(name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList(name='AssistCoefficient2')
    target_speed = shared_memory.ShareableList(name='TargetSpeed')
    
    assist_coefficient.shm.unlink()
    decoder_coefficient.shm.unlink()
    training_flag.shm.unlink()
    moving_target_flag.shm.unlink()
    ff_coefficient.shm.unlink()
    PosFlag.shm.unlink()
    ring_flag.shm.unlink()
    
    GenerationSpeed.shm.unlink()
    AssistCoefficient1.shm.unlink()
    AssistCoefficient2.shm.unlink()
    target_speed.shm.unlink()
    
    assist_coefficient = shared_memory.ShareableList([6],name='AssistCoefficient')
    decoder_coefficient = shared_memory.ShareableList([0.3],name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList([1],name='TrainingFlag')
    moving_target_flag = shared_memory.ShareableList([0],name='TrainingFlag')
    ff_coefficient = shared_memory.ShareableList([1],name='FFCoefficient')
    PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    ring_flag = shared_memory.ShareableList([0],name='RingFlag')
    
    GenerationSpeed = shared_memory.ShareableList([2.818],name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList([1],name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList([0],name='AssistCoefficient2')
    target_speed = shared_memory.ShareableList([1/12],name='TargetSpeed')

AssistCoefficient1[0] = 0
GenerationSpeed[0] = 3.5

#%% filename
psychopy_filename = '/share/{}_'.format('monkey_name')+str(int(time.time()*10))+'_center_out_2D.psydat'

def reward_ao(ao_device,reward_flag):
    import time
    AssistCoefficient1 = shared_memory.ShareableList(name='AssistCoefficient1')
    while True:
        if AssistCoefficient1[0]==1:
            ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=0)
            time.sleep(0.5)
            ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=voltage)
        ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=0)

AssistCoefficient1 = shared_memory.ShareableList(name='AssistCoefficient1')
decoder_coefficient[0]=0.3
assist_coefficient[0]=10

system = 5
class PsychopyPresenter(Node):

    def __init__(self):
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('psychopy_presenter')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = system
        parameters['group'] = 0
        parameters['state'] = 0
        parameters['decoding_element'] = 'decoder_0'
        
        self.state_publisher_ = self.create_publisher(State, '/system_{}/group_{}/state_{}/state'.format(
                                               parameters['system'], parameters['group'], parameters['state']), 1
                                               )
        self.desired_state_publisher_ = self.create_publisher(State, '/system_{}/group_{}/state_{}/desired_state'.format(
                                               parameters['system'], parameters['group'], parameters['state']), 1
                                               )
        
        self.cli = self.create_client(
                                      DecodingService, '/system_{}/group_{}/predictor/decoding_service'.format(parameters['system'], parameters['group'])
                                     )
                                     
        # while not self.cli.wait_for_service(timeout_sec=0.02):
        #    self.get_logger().info('service not available, waiting again...')
        
        while not self.cli.wait_for_service(timeout_sec=None):
            if not rclpy.ok():
                self.get_logger().info('service not available, waiting again...')
                raise RuntimeError
            
        self.req = DecodingService.Request()

        #%% logging parameters
        for par in parameters:
            self.get_logger().info('Parameters: {}: {}'.format(par, str(parameters[par])))
        

    def send_request(self, re):
        self.req.req = re
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

import threading

def main(args=None):
    # reward_flag = False
    # thread1 = threading.Thread(target=reward_ao,args=(ao_device,reward_flag))
    # thread1.start()
    
    def marker_publisher(marker):
        
        def eventmarker(marker):
            dio_device.d_out(DigitalPortType.AUXPORT, 255 & (marker + 192))
        #     print(j)
            time.sleep(0.002)
            dio_device.d_out(DigitalPortType.AUXPORT, 0)

        psychopy_presenter.get_logger().info('Publishing: Event marker: {}'.format(str(marker)))
        eventmarker(marker)
    
    def win_flip_info(object_instance, psychopy_presenter):
        object_attr = {}      
        for i in object_instance:
            object_attr[i] = {}
            attr = object_instance[i].__dict__
            
            for j in attr:
                if j not in ['win', '_initParams', '_vertices', '_verticesBase', '_rotationMatrix', '_borderBase', '_verticesBase','mouse']:
                    # object_attr[i][j] = attr[j]        
                    if isinstance(attr[j], np.ndarray):
                        if len(attr[j].squeeze().shape)>1:
                           continue
                        object_attr[i][j] = list(attr[j]) if len(attr[j].shape)!=0 else float(attr[j])                    
                    else:
                        object_attr[i][j] = attr[j] if '_' not in j else str(attr[j])
                        
        psychopy_presenter.get_logger().info('Publishing: frame info: {}'.format(json.dumps(object_attr)))
    
    #%% construct 
    rclpy.init(args=args)
    state_msg = State()
    desired_state = State()
    psychopy_presenter = PsychopyPresenter()
    
    psychopy_presenter.get_logger().info('Publishing: win info: {}'.format(str(mywin)))
    
    object_instance['SurroundTarget'].autoDraw = True
    object_instance['Cursor'].autoDraw = True
    object_instance['Cursor'].fillColor=(1,1,1)
    PosFlag[0]=1
    

    pr = [0,0]

    for trial_index, trial in enumerate(BMIExp):
        
        #%% task start
        #core.wait(0.5)#interval
        object_instance['SurroundTarget'].autoDraw = True
        marker_publisher(24)
        
        '''
        d = np.array(trial['Reach con'])-np.array(pr)
        if np.linalg.norm(d)>13:
            continue'''
        d1 = np.array(trial['Reach con'])-np.array(object_instance['Cursor'].pos)
        if np.linalg.norm(d1)<6:
            continue
        object_instance['SurroundTarget'].pos = trial['Reach con']
        pr = trial['Reach con']
        # win_flip_info(object_instance, psychopy_presenter)
        mywin.flip()
        win_flip_info(object_instance, psychopy_presenter)
        
        psychopy_presenter.get_logger().info('Publishing: Trial info: Target position: {}, Trial index: {}'.format(str(object_instance['SurroundTarget'].pos), str(trial_index)))
        marker_publisher(1)
        
        if training_flag[0]==1:
            tf = 1
        else:
            tf = 0
        
        WithinTrailTimer.reset()
        TrialTimeCounter.reset()
        t1 = TrialTimeCounter.getTime()
        decoding_time = 10
        reward_flag=1

        # t3 = TrialTimeCounter.getTime()
        while TrialTimeCounter.getTime()<decoding_time:
            t2 = TrialTimeCounter.getTime()
                                                               
            DiffDegree = np.math.atan2(object_instance['SurroundTarget'].pos[1]-object_instance['Cursor'].pos[1],
                                       object_instance['SurroundTarget'].pos[0]-object_instance['Cursor'].pos[0])
            
            assitant_vel = np.array([np.cos(DiffDegree),np.sin(DiffDegree)])
            decoding_results = np.array(psychopy_presenter.send_request(True).res)
            # decoding_results=0
            
            if np.linalg.norm(decoding_results) > 10:
                 decoding_results = decoding_results/np.linalg.norm(decoding_results) * 10
                 
            vel = decoding_results*decoder_coefficient[0]+assitant_vel*assist_coefficient[0]
            object_instance['Cursor'].pos = object_instance['Cursor'].pos+vel*(t2-t1)
            t1 = TrialTimeCounter.getTime()
            # psychopy_presenter.get_logger().info('Publishing: pos: {}'.format(str(object_instance['Cursor'].pos)))
            
            # t1 = TrialTimeCounter.getTime()

            if np.linalg.norm(object_instance['Cursor'].pos) > 15:
                 object_instance['Cursor'].pos = object_instance['Cursor'].pos/np.linalg.norm(object_instance['Cursor'].pos)*15
            
            # win_flip_info(object_instance, psychopy_presenter)
            mywin.flip()
            win_flip_info(object_instance, psychopy_presenter)
            psychopy_presenter.get_logger().info('Publishing: pos: {}, vel: {}, vec_len: {}, decoding results: {}, target pos: {}'.format(str(object_instance['Cursor'].pos),str(vel),str(len(vel)),str(decoding_results), str(object_instance['SurroundTarget'].pos)))
            
            #%% adaptive intention state
            vel_norm = np.linalg.norm(vel)
            desired_state.x_state = list(object_instance['Cursor'].pos) + [np.cos(DiffDegree)*vel_norm,np.sin(DiffDegree)*vel_norm] + [1.0]
            
            if tf==1:
                 psychopy_presenter.desired_state_publisher_.publish(desired_state)
            
            #%% actual state
            state_msg.x_state = list(object_instance['Cursor'].pos)
            psychopy_presenter.state_publisher_.publish(state_msg)
            psychopy_presenter.get_logger().info('Publishing: state: desired state: {}, state: {}'.format(str(desired_state.x_state), str(state_msg.x_state)))
            psychopy_presenter.get_logger().info('Publishing: coefficient: decoder coefficient: {}, assist coefficient: {}, PosFlag: {}, GenerationSpeed: {}, AssistCoefficient1: {}, AssistCoefficient2: {}, moving_target_flag: {}, training flag: {}, ring flag: {}'.format(
                        str(decoder_coefficient[0]), str(assist_coefficient[0]), str(PosFlag[0]), str(GenerationSpeed[0]), str(AssistCoefficient1[0]), str(AssistCoefficient2[0]), str(moving_target_flag[0]), str(tf), str(ring_flag[0]))
                        )
        
            if (np.linalg.norm(object_instance['Cursor'].pos-object_instance['SurroundTarget'].pos)<TouchErr) and (reward_flag==1):
                 marker_publisher(4)
                 psychopy_presenter.get_logger().info('Publishing: Trial error: 0')
                 reward_flag = 0
                 decoding_time = TrialTimeCounter.getTime() + 0.3
                 # TrialTimeCounter.reset()
                 object_instance['SurroundTarget'].autoDraw = False
                 mywin.flip()
                 ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=voltage)

        ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=0)
                 
        marker_publisher(5)         
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
        try:
            pass
        except KeyboardInterrupt:
            psychopy_presenter.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
