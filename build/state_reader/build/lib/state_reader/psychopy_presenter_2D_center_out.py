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
from interfaces.srv import Decoder
from collections import deque
from queue import Queue
from shared_memory_support import Shared_Array
from psychopy import visual, core
from psychopy import data
import pickle
from multiprocessing import shared_memory
import datetime
import os

#%% set udp protocal
import socket
bhvip='192.168.137.3'
bhvport=8866
bhvaddr=(bhvip,bhvport)
udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp_socket.settimeout(None)

#%% task parameter
SurroundRadius = 10
TouchErr = 2.5
mywin = visual.Window(size=(1280, 1024),monitor='testMonitor',color=(-1, -1, -1), units="deg",screen=1,fullscr=False)

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

object_instance = {}
for object_type in object_dict:
    for object_name in object_dict[object_type]:
        assert not object_name in object_instance, 'object name should only appear once'
        object_instance[object_name] = object_map[object_type](
        win=mywin, name = object_name, **object_dict[object_type][object_name]
        )

## task configuration
conds = data.createFactorialTrialList({'Reach Direction':np.array([1,0])*np.pi})
BMIExp = data.TrialHandler(trialList=conds, nReps=10, name='Calibrate',method='random',
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
    decoder_coefficient = shared_memory.ShareableList([0],name='DecoderCoefficient')
except FileExistsError:
    assist_coefficient = shared_memory.ShareableList(name='AssistCoefficient')
    decoder_coefficient = shared_memory.ShareableList(name='AssistCoefficient')
    assist_coefficient.shm.unlink()
    decoder_coefficient.shm.unlink()
    assist_coefficient = shared_memory.ShareableList([2],name='AssistCoefficient')
    decoder_coefficient = shared_memory.ShareableList([0],name='DecoderCoefficient')
    

#%% filename
psychopy_filename = '/share/{}_'.format('monkey_name')+str(int(time.time()*10))+'_center_out_2D_PSID.psydat'

class PsychopyPresenter(Node):

    def __init__(self):
        super().__init__('psychopy_presenter')
        self.state_publisher_ = self.create_publisher(State, 'state', 10)
        self.intention_state_publisher_ = self.create_publisher(State, 'intention_state', 10)
        
        '''
        self.cli = self.create_client(Decoder, 'Decoder')
        while not self.cli.wait_for_service(timeout_sec=0.02):
            self.get_logger().info('service not available, waiting again...')
        self.req = Decoder.Request()'''

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
    intention_state = State()
    psychopy_presenter = PsychopyPresenter()
    
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
        
        #%% Delay period
        core.wait(1)
        ## GO signal
        object_instance['Cursor'].fillColor=(1,1,1)
        mywin.flip()
        marker_publisher(2)
        core.wait(0.2)
        
        #%% BMIdecoding
        marker_publisher(3)
        TrialTimeCounter.reset()
        while TrialTimeCounter.getTime()<10:
            # Xvel = np.array(psychopy_presenter.send_request(True))*DecoderCoefficient[0]+np.cos(trial['Reach Direction'])*AssistCoefficient[0]
            Xvel = np.cos(trial['Reach Direction'])*assist_coefficient[0]
            Xpos = Xpos+Xvel*0.01667
            object_instance['Cursor'].pos = [Xpos,0]
            mywin.flip()
            
            #%% adaptive intention state
            intention_state.state = [np.cos(trial['Reach Direction'])*np.abs(Xvel), 0.0, 1.0]
            psychopy_presenter.intention_state_publisher_.publish(intention_state)
            
            #%% actual state
            state_msg.state = list(object_instance['Cursor'].pos)
            psychopy_presenter.state_publisher_.publish(state_msg)
            psychopy_presenter.get_logger().info('Publishing: state: [intention state: {}, state: {}]'.format(str(intention_state.state), str(state_msg.state)))
            psychopy_presenter.get_logger().info('Publishing: coefficient: [decoder coefficient: {}, assist coefficient: {}]'.format(str(decoder_coefficient[0]), str(assist_coefficient[0])))
            
            CursorTrajectory = CursorTrajectory+[object_instance['Cursor'].pos]
        
            if np.linalg.norm(object_instance['Cursor'].pos-object_instance['SurroundTarget'].pos)<TouchErr:
                 marker_publisher(4)
                 BMIExp.addData('TrialError', 0)
                 break
        
        #%% Target off
        object_instance['Cursor'].autoDraw = False
        object_instance['SurroundTarget'].autoDraw = False
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
