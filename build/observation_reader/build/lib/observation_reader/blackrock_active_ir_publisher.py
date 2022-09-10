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
from cerebus import cbpy
import time
from interfaces.srv import ActiveObservation
from collections import deque
import json

def deque_convolve(neural_buffer_deque, smoothed_parameter):
    return list((np.array(neural_buffer_deque)*smoothed_parameter).sum(1))

parameters = {}
parameters['timer_period'] = 0.5
parameters['chn_num'] = 280
parameters['time_buffer_length'] = 20
parameters['smoothed_parameter'] = list(range(280))
parameters['system'] = 0
parameters['spiketrain_window'] = 0.5

class BlackrockActiveIrPublisher(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'blackrock_ir_publisher' field
        super().__init__('/system/blackrock_active_ir_publisher')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------        
        
        #%% declare parameters
        self.declare_parameter('parameters', json.dumps(parameters))
        
        #%% get parameters
        parameters = json.loads(self.get_parameter('parameters').get_parameter_value().string_value)
        
        #%% logging parameters
        for par in parameters:
            self.get_logger().info('Parameters: {}: {}'.format(par, str(parameters[par])))
        
        #+-----------------------------------------------------------------------
        # initialize communication module and use callback function
        #+-----------------------------------------------------------------------

        self.srv = self.create_service(
                ActiveObservation, 
                '/system_{}/active_observation'.format(str(parameters['system'])), 
                self.neural_array_callback
                )
        
        self.timer = self.create_timer(parameters['timer_period'], self.build_spiketrain_callback)
        
        #+-----------------------------------------------------------------------
        # connect to blackrock
        #+-----------------------------------------------------------------------
        
        while True:
            con_params = cbpy.defaultConParams()
            
            re,con = cbpy.open(instance = 0,
                               connection = 'default',
                               parameter = con_params)
            
            buffer_par = {'double':True,'event_length':15000}
            
            cbpy.trial_config(
                  instance = 0,
                  reset = True,
                  buffer_parameter = buffer_par,
                  nocontinuous = True,
                  nocomment = True)
                  
            time.sleep(2)
            ree,event = cbpy.trial_event(instance = 0,reset = True)
            
            time.sleep(0.1)
            cbpy_close_marker = 0
            try:
                print(event[-1][1]['events'])
                break
            except IndexError:
                print("catch data wrong")
                while cbpy_close_marker!=2:
                    cbpy_close_marker = cbpy.close()
                    time.sleep(0.2)
                continue
            time.sleep(1)
            
            print("fail")
            while cbpy_close_marker!=2:
                cbpy_close_marker = cbpy.close()
                time.sleep(0.2)
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------   
        
        self.neural_buffer = np.zeros((parameters['chn_num'], )) # shared recieved neural data
        self._neural_buffer_deque = [deque() for _ in range(parameters['chn_num'])] # a deque buffer for saving spiketrain
        self._time_buffer_deque = deque(maxlen = parameters['time_buffer_length'])
        
        # modify the smoothed parameter
        self._smoothed_parameter = np.tile(np.array(parameters['smoothed_parameter']),(parameters['chn_num'],1))
        
        #%% set '0' time stamp
        ree,event = cbpy.trial_event(instance = 0, reset = True, reset_clock=True)
        self._time_start = time.time()
        
        # share 'spiketrain_window' paramter
        self._spiketrain_window = parameters['spiketrain_window']
	
    def build_spiketrain_callback(self):
    	
    	#%% read data from br
        ree,event = cbpy.trial_event(instance = 0,reset = True)
        self.neural_buffer[:] = 0 # 0 padding
        
        #%% set actual spike count to appointed chn
        if len(event)!=0:
            for i in event: 
                self._neural_buffer_deque[i[0]-1] = self._neural_buffer_deque[i[0]-1] + i[1]['timestamps'][0] # extent deque list
                
                ## left pop, if the first spike time stamp is too far from the last
                while (self._neural_buffer_deque[i[0]-1][-1]-self._neural_buffer_deque[i[0]-1][0]) < self._spiketrain_window:
                    self._neural_buffer_deque[i[0]-1].popleft()
        
        self.get_logger().info('Publishing: neural_buffer_deque: {}'.format(str(self._neural_buffer_deque)))
    
    def neural_array_callback(self, request, response):
        
        #%% read ramaining spike train, if the request happens at the interval of the timer  
        self.build_spiketrain_callback()
        
        #%% set '0' time stamp from a request
        request_time = request.time-self._time_start
        left_border = request_time-request.window_width
        
        #%% get spike count
        active_observation = [np.histogram(
                                           chn_data-left_border if len(chn_data) !=0 else chn_data,
                                           bins=request.bin_num,
                                           range=(0,request.window_width)
                              ) for chn_data in self._neural_buffer_deque]
        
        #%% send a response from time buffer
        self._time_buffer_deque.append(deque_convolve(active_observation, self._smoothed_parameter))
        self._neural_array = np.array(self._time_buffer_deque).T
        response.y_observation = list(self._neural_array.flatten())
        response.y_shape = list(self._neural_array.shape) 
        
        self.get_logger().info('Servicing: ActiveObservation: [incoming request : {}, outcoming response : {}]'.format(str(request), str(response)))

def main(args=None):
    rclpy.init(args=args)
    blackrock_active_ir_publisher = BlackrockActiveIrPublisher()
    rclpy.spin(blackrock_active_ir_publisher)
    blackrock_active_ir_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
