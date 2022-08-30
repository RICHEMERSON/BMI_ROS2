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
from std_msgs.msg import String
from cerebus import cbpy
import time
from interfaces.msg import PassiveObservation
from collections import deque

def deque_convolve(neural_buffer_deque, smoothed_parameter):
    return list((np.array(neural_buffer_deque).T*smoothed_parameter).sum(1))

class BlackrockPassiveIrPublisher(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------        
        
        #%% declare parameters
        self.declare_parameter('timer_period', 0.5)
        self.declare_parameter('chn_num', 280)
        self.declare_parameter('deque_buffer_length', 20)
        self.declare_parameter('smoothed_parameter', list(range(280)))
        self.declare_parameter('system', 0)
        self.declare_parameter('time_buffer_length', 20)
        
        #%% get parameters
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        chn_num = self.get_parameter('chn_num').get_parameter_value().integer_value
        deque_buffer_length = self.get_parameter('deque_buffer_length').get_parameter_value().integer_value
        smoothed_parameter = self.get_parameter('smoothed_parameter').get_parameter_value().double_array_value
        system = self.get_parameter('system').get_parameter_value().integer_value
        time_buffer_length = self.get_parameter('time_buffer_length').get_parameter_value().integer_value
        
        #%% logging parameters
        self.get_logger().info('Parameters: chn_num: {}'.format(str(chn_num)))
        self.get_logger().info('Parameters: timer_period: {}'.format(str(timer_period)))
        self.get_logger().info('Parameters: deque_buffer_length: {}'.format(str(deque_buffer_length)))
        self.get_logger().info('Parameters: smoothed_parameter: {}'.format(str(smoothed_parameter)))
        self.get_logger().info('Parameters: system: {}'.format(str(system)))
        self.get_logger().info('Parameters: time_buffer_length: {}'.format(str(time_buffer_length)))
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'blackrock_ir_publisher' field
        super().__init__('/system_{}/blackrock_passive_ir_publisher'.format(str(system)))
        
        #+-----------------------------------------------------------------------
        # initialize communication module and use callback function
        #+-----------------------------------------------------------------------
        
        self._neural_buffer_deque = deque(maxlen=deque_buffer_length)
        self.publisher_ = self.create_publisher(
                PassiveObservation, '/system_{}/passive_observation'.format(str(system)), 1
                )
        self.msg = PassiveObservation()
        
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
        
        self.neural_buffer = np.zeros((chn_num, )) # shared recieved neural data
        self._neural_buffer_deque = deque(maxlen=20) # a deque buffer for preprocessing
        self._time_buffer_deque = deque(maxlen = time_buffer_length)
        
        # modify the smoothed parameter
        self._smoothed_parameter = np.tile(np.array(smoothed_parameter),(chn_num,1))
	
	#%% build the ros timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
	
    def timer_callback(self):
    	
    	#%% read data from br
        ree,event = cbpy.trial_event(instance = 0,reset = True)
        self.neural_buffer[:] = 0
        if len(event)!=0:
            for i in event: self.neural_buffer[i[0]-1] = i[1]['timestamps'][0].shape[0]
        
        #%% smooth data to get ir
        self._neural_buffer_deque.append(self.neural_buffer.copy())
        self._time_buffer_deque.append(deque_convolve(self._neural_buffer_deque, self._smoothed_parameter))
        self._neural_array = np.array(self._time_buffer_deque).T
        
        #%% broadcast neural data (ir)
        self.msg.y_observation = list(self._neural_array.flatten())
        self.msg.y_shape = list(self._neural_array.shape) 
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: Observation: {}'.format(str(self.msg)))

def main(args=None):
    rclpy.init(args=args)
    blackrock_passive_ir_publisher = BlackrockPassiveIrPublisher()
    rclpy.spin(blackrock_ir_publisher)
    blackrock_ir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
