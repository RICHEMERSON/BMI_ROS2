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
# from shared_memory_support import Shared_Array
from psychopy import visual, core, event
from psychopy import data
import pickle
from multiprocessing import shared_memory
import datetime
import os
import json
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

def AssistGenerator(t,tau=0.55):
    return (t/tau)**2*np.exp(-((t/tau)**2)/2)

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
object_dict['circle']['SurroundTarget']['size'] = 2
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

object_dict['circle']['Center'] = {}
object_dict['circle']['Center']['size'] = 2
object_dict['circle']['Center']['pos'] = [0,0]
object_dict['circle']['Center']['lineColor'] = (-1,-1,-1)
object_dict['circle']['Center']['fillColor'] = (-1,1,-1)

object_instance = {}
for object_type in object_dict:
    for object_name in object_dict[object_type]:
        assert not object_name in object_instance, 'object name should only appear once'
        object_instance[object_name] = object_map[object_type](
        win=mywin, name = object_name, **object_dict[object_type][object_name]
        )

fullscr_area = visual.Circle(mywin, radius=100, name='background', pos=(0, 0), lineColor='red', fillColor='red', opacity=0)
center_radius = visual.Circle(mywin, radius=2.5, name='center_radius', pos=(0, 0), lineColor='red', fillColor='red', opacity=0)
target_radius = visual.Circle(mywin, radius=2.5, name='target_radius', pos=(0, 0), lineColor='green', fillColor='green', opacity=0)
mywin.mouseVisible = False

## task configuration
conds = data.createFactorialTrialList({'Reach Direction':np.array([0, 1/8, 1/4, 3/8, 1/2, 5/8,  3/4, 7/8,  1, 9/8, 5/4, 11/8, 3/2, 13/8, 7/4, 15/8])*np.pi})
BMIExp = data.TrialHandler(trialList=conds, nReps=4000, name='Calibrate',method='random',
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

try:  
    DecoderSel = shared_memory.ShareableList([0],name='DecoderSel') 
except FileExistsError:         
    DecoderSel = shared_memory.ShareableList(name='DecoderSel')         
    DecoderSel.shm.unlink()
    DecoderSel = shared_memory.ShareableList([0],name='DecoderSel')
    
#%% initialize shared memory parameter
try:  
    smooth_coefficient = shared_memory.ShareableList([1],name='SmoothCoefficient') 
except FileExistsError:         
    smooth_coefficient = shared_memory.ShareableList(name='SmoothCoefficient')         
    smooth_coefficient.shm.unlink()
    smooth_coefficient = shared_memory.ShareableList([1],name='SmoothCoefficient')

try:  
    select_pos = shared_memory.ShareableList([0],name='SelectPos') 
except FileExistsError:         
    select_pos = shared_memory.ShareableList(name='SelectPos')         
    select_pos.shm.unlink()
    select_pos = shared_memory.ShareableList([0],name='SelectPos')

try:
    
    assist_coefficient = shared_memory.ShareableList([6],name='AssistCoefficient')
    ff_coefficient = shared_memory.ShareableList([1],name='FFCoefficient')
    decoder_coefficient = shared_memory.ShareableList([0],name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList([1],name='TrainingFlag')
    PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    moving_target_flag = shared_memory.ShareableList([0],name='MovingTargetFlag')
    
    GenerationSpeed = shared_memory.ShareableList([2.818],name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList([1],name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList([0],name='AssistCoefficient2')
    ring_flag = shared_memory.ShareableList([0],name='RingFlag')
    target_speed = shared_memory.ShareableList([2/3],name='TargetSpeed')

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
    decoder_coefficient = shared_memory.ShareableList([0],name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList([1],name='TrainingFlag')
    moving_target_flag = shared_memory.ShareableList([0],name='TrainingFlag')
    ff_coefficient = shared_memory.ShareableList([1],name='FFCoefficient')
    PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    ring_flag = shared_memory.ShareableList([0],name='RingFlag')
    
    GenerationSpeed = shared_memory.ShareableList([2.818],name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList([1],name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList([0],name='AssistCoefficient2')
    target_speed = shared_memory.ShareableList([2/3],name='TargetSpeed')

AssistCoefficient1[0] = 0.5
GenerationSpeed[0] = 4

#%% filename
psychopy_filename = '/share/{}_'.format('monkey_name')+str(int(time.time()*10))+'_center_out_2D.psydat'

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
        parameters['system'] = 5
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
                                     
        while not self.cli.wait_for_service(timeout_sec=None):
            if not rclpy.ok():
                self.get_logger().info('service not available, waiting again...')
                raise RuntimeError
            
        self.req = DecodingService.Request()

    def send_request(self, re):
        self.req.req = re
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    # ser = serial.Serial("/dev/ttyUSB0", 115200)

    def marker_publisher(marker):
        psychopy_presenter.get_logger().info('Publishing: Event marker: {}'.format(str(marker)))
        
        # ser.close()
    
    def win_flip_info(object_instance, psychopy_presenter):
        mywin.flip()
        object_attr = {}      
        for i in object_instance:
            object_attr[i] = {}
            attr = object_instance[i].__dict__
            
            for j in attr:
                if j not in ['win', '_initParams', '_vertices', '_verticesBase', '_rotationMatrix', '_borderBase', '_verticesBase','Mouse','mouse']:
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
    PosFlag[0]=0
    mouse = event.Mouse()

    psychopy_presenter.get_logger().info('Publishing: win info: {}'.format(str(mywin)))
    
    for trial_index, trial in enumerate(BMIExp):
        #%% Target off
        object_instance['Cursor'].autoDraw = False
        object_instance['SurroundTarget'].autoDraw = False
        object_instance['Center'].autoDraw = False
        object_instance['Cursor'].fillColor=(0.1,0.1,0.1)
        object_instance['Ring'].fillColor = (-1,-1,-1)
        object_instance['SurroundTarget'].fillColor=(-1,1,-1)

        mywin.flip()
        # win_flip_info(object_instance, psychopy_presenter)
        time.sleep(1)

        if ring_flag[0]==0:
            go = False
        else:
            go = np.random.choice([True,False], p=[0.5,0.5])

        # psychopy_presenter = PsychopyPresenter()

        object_instance['SurroundTarget'].pos = [SurroundRadius*np.cos(trial['Reach Direction']),
                              SurroundRadius*np.sin(trial['Reach Direction'])]
        
        #%% center on
        object_instance['Center'].autoDraw = True
        win_flip_info(object_instance, psychopy_presenter)
        marker_publisher(1)

        #%% check center touch
        TrialTimeCounter.reset()
        touch_center_flag = False
        while TrialTimeCounter.getTime()<5:
            if mouse.isPressedIn(fullscr_area, buttons=[0]):
                touch_center_flag = True
                marker_publisher(12) # trial end
                break
        if not touch_center_flag:
            marker_publisher(5) # no touch,trial end
            continue
        else:
            TrialTimeCounter.reset()
            hold_center_flag = True
            while TrialTimeCounter.getTime()<1:
                if not mouse.isPressedIn(center_radius, buttons=[0]):
                    hold_center_flag = False
                    break

        if not hold_center_flag:
            marker_publisher(5) # hold center break or wrong touch center,trial end
            continue

        if go:

            object_instance['SurroundTarget'].fillColor=(-1,1,-1)
            object_instance['SurroundTarget'].autoDraw = True
            win_flip_info(object_instance, psychopy_presenter)
            marker_publisher(21) # target on

            TrialTimeCounter.reset()
            hold_center_flag = True
            while TrialTimeCounter.getTime()<1:
                if not mouse.isPressedIn(center_radius, buttons=[0]):
                    hold_center_flag = False
                    break

            if not hold_center_flag:
                marker_publisher(5) # hold center break,trial end
                continue

            object_instance['Center'].autoDraw = False
            win_flip_info(object_instance, psychopy_presenter)
            marker_publisher(3) # GO

            TrialTimeCounter.reset()
            hold_center_flag = True
            while TrialTimeCounter.getTime()<2:
                buttons = mouse.getPressed()
                if buttons[0] == 0:
                    hold_center_flag= False
                    marker_publisher(4) # leave screen
                    break
            if hold_center_flag:
                marker_publisher(5) # no leave screen,trial end
                continue
            
            target_radius.pos = object_instance['SurroundTarget'].pos
            touch_target_flag = False
            while TrialTimeCounter.getTime()<2:
                if mouse.isPressedIn(fullscr_area, buttons=[0]):
                    if mouse.isPressedIn(target_radius, buttons=[0]):
                        touch_target_flag = True
                    else:
                        touch_target_flag = False
                    marker_publisher(6) # touch target
                    break

            if not touch_target_flag:
                marker_publisher(5) # no touch screen,trial end
                continue
            
            hold_target_flag = True
            TrialTimeCounter.reset()
            while TrialTimeCounter.getTime()<1:
                if not mouse.isPressedIn(target_radius, buttons=[0]):
                    hold_target_flag = False
                    break

            if not hold_target_flag:
                marker_publisher(5) # wrong touch or hold target break,trial end
                continue

            marker_publisher(20) # success, trial end
            ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=voltage)
            time.sleep(0.6)
            ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=0)

            marker_publisher(5)
        
        #%% BMIdecoding
        if not go:

            object_instance['SurroundTarget'].fillColor=(1,-1,-1)
            object_instance['SurroundTarget'].autoDraw = True
            object_instance['Cursor'].autoDraw = True
            Xpos = 0 #PosX SpeedX 1
            Xvel = 0 #PosX SpeedX 1
            object_instance['Cursor'].pos = [Xpos,0]

            win_flip_info(object_instance, psychopy_presenter)
            marker_publisher(10)
            TrialTimeCounter.reset()
            
            PosFlag[0]=1

            if training_flag[0]==1:
                tf = 1
            else:
                tf = 0
            
            t1 = TrialTimeCounter.getTime()
            decoding_time = 5

            while TrialTimeCounter.getTime()<decoding_time:

                if not mouse.isPressedIn(center_radius, buttons=[0]):
                    break

                t2 = TrialTimeCounter.getTime()                                                               
                DiffDegree = np.math.atan2(object_instance['SurroundTarget'].pos[1]-object_instance['Cursor'].pos[1],
                                        object_instance['SurroundTarget'].pos[0]-object_instance['Cursor'].pos[0])
                
                assitant_vel = np.array([np.cos(DiffDegree),np.sin(DiffDegree)])
                decoding_results = np.array(psychopy_presenter.send_request(True).res)
                    
                vel = decoding_results*decoder_coefficient[0]+assitant_vel*assist_coefficient[0]
                object_instance['Cursor'].pos = object_instance['Cursor'].pos+vel*(t2-t1)
                t1 = TrialTimeCounter.getTime()
                mywin.flip()
                win_flip_info(object_instance, psychopy_presenter)
                
                #%% adaptive intention state
                vel_norm = np.linalg.norm(vel)
                desired_state.x_state = list(object_instance['Cursor'].pos) + [np.cos(DiffDegree)*vel_norm,np.sin(DiffDegree)*vel_norm] + [1.0]
                
                if tf==1:
                    psychopy_presenter.desired_state_publisher_.publish(desired_state)
                    psychopy_presenter.get_logger().info('send data')
                
                #%% actual state
                state_msg.x_state = list(object_instance['Cursor'].pos)
                # psychopy_presenter.state_publisher_.publish(state_msg)

                psychopy_presenter.get_logger().info('Publishing: state: desired state: {}, state: {}'.format(str(desired_state.x_state), str(state_msg.x_state)))
                psychopy_presenter.get_logger().info('Publishing: coefficient: decoder coefficient: {}, assist coefficient: {}, PosFlag: {}, GenerationSpeed: {}, AssistCoefficient1: {}, AssistCoefficient2: {}, moving_target_flag: {}, training flag: {}, ring flag: {}'.format(
                            str(decoder_coefficient[0]), str(assist_coefficient[0]), str(PosFlag[0]), str(GenerationSpeed[0]), str(AssistCoefficient1[0]), str(AssistCoefficient2[0]), str(moving_target_flag[0]), str(tf), str(ring_flag[0]))
                            )
            
                if np.linalg.norm(object_instance['Cursor'].pos-object_instance['SurroundTarget'].pos)<TouchErr:
                    marker_publisher(11)
                    BMIExp.addData('TrialError', 0)
                    psychopy_presenter.get_logger().info('Publishing: Trial error: 0')
                    object_instance['SurroundTarget'].fillColor=(1,-1,-1)
                    mywin.flip()
                    ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=voltage)
                    time.sleep(0.6)
                    ao_device.a_out(0, Range.UNI5VOLTS, AOutFlag.DEFAULT, data=0)
                    object_instance['Cursor'].autoDraw = False
                    object_instance['SurroundTarget'].autoDraw = False
                    object_instance['Cursor'].fillColor=(0.1,0.1,0.1)
                    object_instance['Ring'].fillColor = (-1,-1,-1)
                    object_instance['SurroundTarget'].fillColor=(-1,1,-1)
                    win_flip_info(object_instance, psychopy_presenter)
                    mywin.flip()
                    
                    break
                
                if np.linalg.norm(object_instance['Cursor'].pos) > 15:
                    break
            
            PosFlag[0]=0
            marker_publisher(12)
        # win_flip_info(object_instance, psychopy_presenter)
        # mywin.flip()
        # win_flip_info(object_instance, psychopy_presenter)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psychopy_presenter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
