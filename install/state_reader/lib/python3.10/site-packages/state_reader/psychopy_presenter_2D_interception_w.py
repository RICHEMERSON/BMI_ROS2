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
from shared_memory_support import Shared_Array
from psychopy import visual, core
from psychopy import data
import pickle
from multiprocessing import shared_memory
import datetime
import os

def AssistGenerator(t,tau=0.55):
    return (t/tau)**2*np.exp(-((t/tau)**2)/2)

#%% set udp protocal
import socket
bhvip='192.168.137.3'
bhvport=8866
bhvaddr=(bhvip,bhvport)
udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp_socket.settimeout(None)

#%% task parameter
SurroundRadius = 10
TouchErr = 3.5
mywin = visual.Window(size=(1280, 1024),monitor='testMonitor',color=(-1, -1, -1), units="deg",screen=1,fullscr=True)

## object statement map
object_map = {}
object_map['circle'] = visual.Circle

## object configuration
object_dict = {}
object_dict['circle'] = {}
object_dict['circle']['SurroundTarget'] = {}
object_dict['circle']['SurroundTarget']['size'] = 2.5
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
conds = data.createFactorialTrialList({'Reach Direction':np.array([0, 1/4, 1/2, 3/4, 1, 5/4, 3/2, 7/4])*np.pi})
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
try:
    
    assist_coefficient = shared_memory.ShareableList([2],name='AssistCoefficient')
    ff_coefficient = shared_memory.ShareableList([1],name='FFCoefficient')
    decoder_coefficient = shared_memory.ShareableList([0],name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList([1],name='TrainingFlag')
    PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    moving_target_flag = shared_memory.ShareableList([0],name='MovingTargetFlag')
    
    GenerationSpeed = shared_memory.ShareableList([2.818],name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList([1],name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList([0],name='AssistCoefficient2')

except FileExistsError:
    
    assist_coefficient = shared_memory.ShareableList(name='AssistCoefficient')
    decoder_coefficient = shared_memory.ShareableList(name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList(name='TrainingFlag')
    moving_target_flag = shared_memory.ShareableList(name='MovingTargetFlag')
    ff_coefficient = shared_memory.ShareableList(name='FFCoefficient')
    PosFlag = shared_memory.ShareableList(name='PosFlag')
    
    GenerationSpeed = shared_memory.ShareableList(name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList(name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList(name='AssistCoefficient2')
    
    assist_coefficient.shm.unlink()
    decoder_coefficient.shm.unlink()
    training_flag.shm.unlink()
    moving_target_flag.shm.unlink()
    ff_coefficient.shm.unlink()
    PosFlag.shm.unlink()
    
    GenerationSpeed.shm.unlink()
    AssistCoefficient1.shm.unlink()
    AssistCoefficient2.shm.unlink()
    
    assist_coefficient = shared_memory.ShareableList([2],name='AssistCoefficient')
    decoder_coefficient = shared_memory.ShareableList([0],name='DecoderCoefficient')
    training_flag = shared_memory.ShareableList([1],name='TrainingFlag')
    moving_target_flag = shared_memory.ShareableList([0],name='TrainingFlag')
    ff_coefficient = shared_memory.ShareableList([1],name='FFCoefficient')
    PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    
    GenerationSpeed = shared_memory.ShareableList([2.818],name='GenerationSpeed')
    AssistCoefficient1 = shared_memory.ShareableList([1],name='AssistCoefficient1')
    AssistCoefficient2 = shared_memory.ShareableList([0],name='AssistCoefficient2')

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
        parameters['system'] = 0
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
                                     
        while not self.cli.wait_for_service(timeout_sec=0.02):
            self.get_logger().info('service not available, waiting again...')
        self.req = DecodingService.Request()

    def send_request(self, re):
        self.req.req = re
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):

    def marker_publisher(marker):
        TrialEventMarker.append([marker,WithinTrailTimer.getTime()])
        psychopy_presenter.get_logger().info('Publishing: Event marker: {}'.format(str(marker)))
        udp_socket.sendto(str(6000+marker).encode("gbk"),bhvaddr)
    
    #%% construct 
    rclpy.init(args=args)
    state_msg = State()
    desired_state = State()
    psychopy_presenter = PsychopyPresenter()
    PosFlag[0]=0
    InitialSpeed = 14
    
    for trial in BMIExp:
        
        #%% task start
        core.wait(2)#interval
        BMIExp.addData('TrialError', 1)
        BMIExp.addData('TrialStartTime', TaskTimer.getTime())
        WithinTrailTimer.reset()
        
        #%% initialize save variable
        TrialEventMarker = []
        CursorTrajectory = []
        UserVars['TargetPos'] = []
        marker_publisher(24)
        
        #%% Target on
        object_instance['SurroundTarget'].autoDraw = True
        object_instance['Cursor'].autoDraw = True
        Xpos = 0 #PosX SpeedX 1
        Xvel = 0 #PosX SpeedX 1
        object_instance['Cursor'].pos = [Xpos,0]
        object_instance['SurroundTarget'].pos = [SurroundRadius*np.cos(trial['Reach Direction']),
                              SurroundRadius*np.sin(trial['Reach Direction'])]
    
        UserVars['TargetPos'] = object_instance['SurroundTarget'].pos
        mywin.flip()
        psychopy_presenter.get_logger().info('Publishing: Target position: {}'.format(str(object_instance['SurroundTarget'].pos)))
        marker_publisher(1)
        
        if decoder_coefficient[0]<0.5 and assist_coefficient[0]==0:
            object_instance['Ring'].autoDraw = True
        
        TrialTimeCounter.reset()
        InitialDegree = trial['Reach Direction']
        TargetDegreeSpeed = np.random.choice([1/6,0,-1/6])*np.pi
        UserVars['TargetDegreeSpeed'] = TargetDegreeSpeed
        #%% Delay period
        while TrialTimeCounter.getTime()<1:
            if moving_target_flag==0:
                InitialDegree = InitialDegree+TargetDegreeSpeed*0.01667
                object_instance['SurroundTarget'].pos = [SurroundRadius*np.cos(InitialDegree),
                                                               SurroundRadius*np.sin(InitialDegree)]
            mywin.flip()
        ## GO signal
        object_instance['Cursor'].fillColor=(1,1,1)
        mywin.flip()
        marker_publisher(2)
        TrialTimeCounter.reset()
        while TrialTimeCounter.getTime()<0.2:
            if moving_target_flag==0:
                InitialDegree = InitialDegree+TargetDegreeSpeed*0.01667
                object_instance['SurroundTarget'].pos = [SurroundRadius*np.cos(InitialDegree),
                                                         SurroundRadius*np.sin(InitialDegree)]
            mywin.flip()
        
        #%% BMIdecoding
        marker_publisher(3)
        TrialTimeCounter.reset()
        
        PosFlag[0]=1
        
        feedforward_flag = False
        OnceFeedForwardFlag = False
        decoding_wait_time = 2
        
        TrialTime = 0
        
        if training_flag[0]==1:
            tf = 1
        else:
            tf = 0
        
        while TrialTimeCounter.getTime()<10:
        
            if moving_target_flag==0:
                InitialDegree = InitialDegree+TargetDegreeSpeed*0.01667
                object_instance['SurroundTarget'].pos = [SurroundRadius*np.cos(InitialDegree),
                                                         SurroundRadius*np.sin(InitialDegree)]
            
            DiffDegree = np.math.atan2(object_instance['SurroundTarget'].pos[1]-object_instance['Cursor'].pos[1],
                                       object_instance['SurroundTarget'].pos[0]-object_instance['Cursor'].pos[0])
            
            assitant_vel = np.array([np.cos(DiffDegree),np.sin(DiffDegree)])
            decoding_results = np.array(psychopy_presenter.send_request(True).res)
            
            vel = decoding_results*decoder_coefficient[0]+assitant_vel*assist_coefficient[0]           
            
            
            if np.linalg.norm(object_instance['Cursor'].pos)>2.5 and not feedforward_flag and decoder_coefficient[0]<0.5 and assist_coefficient[0]==0:
                    
                FeedforwardSpeed = np.linalg.norm(decoding_results) + 5
                decoding_wait_time = TrialTimeCounter.getTime()+1.5
                speed_degree = np.math.atan2(Cursor.pos[1],Cursor.pos[0])
                
                BellCurveCounter = 0
                feedforward_flag=True
                Ring.autoDraw = False
                FeedforwardFlag = True
                object_instance['Ring'].fillColor = (-1,-1,-1)
            
            '''
            if feedforward_flag and np.linalg.norm(object_instance['Cursor'].pos)<10:
                BellCurveCounter = BellCurveCounter+0.01667
                
                Xvel = np.cos(DiffDegree)*InitialSpeed*AssistGenerator(TrialTime)*AssistCoefficient1[0]+\
                        np.cos(speed_degree)*FeedforwardSpeed*AssistGenerator(BellCurveCounter+0.15,tau=0.25)*GenerationSpeed[0]+\
                        np.cos(DiffDegree)*AssistCoefficient2[0]
                
                Yvel = np.sin(DiffDegree)*InitialSpeed*AssistGenerator(TrialTime)*AssistCoefficient1[0]+\
                        np.sin(speed_degree)*FeedforwardSpeed*AssistGenerator(BellCurveCounter+0.15,tau=0.25)*GenerationSpeed[0]+\
                        np.sin(DiffDegree)*AssistCoefficient2[0]
                
                vel = np.array([Xvel, Yvel])           
                Cursor.fillColor=(1,-1,-1)'''
            
            # psychopy_presenter.get_logger().info(str(np.cos(trial['Reach Direction'])*np.abs(Xvel)))
            # Xvel = np.cos(trial['Reach Direction'])*assist_coefficient[0]
            
            object_instance['Cursor'].pos = object_instance['Cursor'].pos+vel*0.01667
            psychopy_presenter.get_logger().info('Publishing: pos: {}'.format(str(object_instance['Cursor'].pos)))

            mywin.flip()
            
            #%% adaptive intention state
            vel_norm = np.linalg.norm(vel)
            desired_state.x_state = list(object_instance['Cursor'].pos) + [np.cos(DiffDegree)*vel_norm,np.sin(DiffDegree)*vel_norm] + [1.0]
            
            if tf==1:
                 psychopy_presenter.desired_state_publisher_.publish(desired_state)
            
            #%% actual state
            state_msg.x_state = list(object_instance['Cursor'].pos)
            psychopy_presenter.state_publisher_.publish(state_msg)
            psychopy_presenter.get_logger().info('Publishing: state: desired state: {}, state: {}'.format(str(desired_state.x_state), str(state_msg.x_state)))
            psychopy_presenter.get_logger().info('Publishing: coefficient: decoder coefficient: {}, assist coefficient: {}'.format(str(decoder_coefficient[0]), str(assist_coefficient[0])))
            psychopy_presenter.get_logger().info('Publishing: coefficient: target trajectory: {}, cursor trajectory: {}, degree speed: {}'.format(str(object_instance['SurroundTarget'].pos), str(object_instance['Cursor'].pos), str(TargetDegreeSpeed)))
            
            CursorTrajectory = CursorTrajectory+[object_instance['Cursor'].pos]
        
            if np.linalg.norm(object_instance['Cursor'].pos-object_instance['SurroundTarget'].pos)<TouchErr:
                 marker_publisher(4)
                 BMIExp.addData('TrialError', 0)
                 break
        
        TrialTime = TrialTime+0.01667
        PosFlag[0]=0
        
        #%% Target off
        object_instance['Cursor'].autoDraw = False
        object_instance['SurroundTarget'].autoDraw = False
        object_instance['Ring'].autoDraw = False
        object_instance['Cursor'].fillColor=(0.1,0.1,0.1)
        mywin.flip()
        marker_publisher(5)
    
        BMIExp.addData('EventMarker', TrialEventMarker)
        BMIExp.addData('AnalogData', CursorTrajectory)
        BMIExp.addData('UserVars', UserVars)
        if os.path.exists(psychopy_filename):
            os.remove(psychopy_filename)
        
        BMIExp.saveAsPickle(psychopy_filename)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psychopy_presenter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
