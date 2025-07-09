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
from interfaces.msg import Ir
from collections import deque

def deque_convolve(neural_buffer_deque, smoothed_parameter):
    return list((np.array(neural_buffer_deque).T*smoothed_parameter).sum(1))

def GetNeuralData(NeuralVector,spikes):
    NeuralVector[:]=0
    while True:
        try:
            RecievedData = spikes.receive(noblock=True)
            NeuralVector[RecievedData['nTrodeId']-1]=NeuralVector[RecievedData['nTrodeId']-1]+1 
        except:
            break

class BlackrockIrPublisher(Node):

    def __init__(self):
        super().__init__('SpikeGadgt_ir_reader')
        
        #%% declare parameters
        self.declare_parameter('timer_period', 0.5)
        self.declare_parameter('chn_num', 280)
        self.declare_parameter('deque_buffer_length', 20)
        self.declare_parameter('select_trial', list(range(280))) # the default value is depend on 'chn_num'
        
        #%% get parameters
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        chn_num = self.get_parameter('chn_num').get_parameter_value().integer_value
        deque_buffer_length = self.get_parameter('deque_buffer_length').get_parameter_value().integer_value
        smoothed_parameter = self.get_parameter('smoothed_parameter').get_parameter_value().double_array_value
        
        #%% logging parameters
        self.get_logger().info('Parameters: chn_num: {}'.format(str(chn_num)))
        self.get_logger().info('Parameters: timer_period: {}'.format(str(timer_period)))
        self.get_logger().info('Parameters: deque_buffer_length: {}'.format(str(deque_buffer_length)))
        self.get_logger().info('Parameters: smoothed_parameter: {}'.format(str(smoothed_parameter)))
        
        self._neural_buffer_deque = deque(maxlen=deque_buffer_length)
        self.publisher_ = self.create_publisher(Ir, 'neural_data/ir', 10)
        self.msg = Ir()
        
        #%% connect to sg
        self._spikes = socket.SourceSubscriber('source.spikes')
        
        # modify the smoothed parameter
        self._smoothed_parameter = np.tile(np.array(smoothed_parameter),(chn_num,1))
	
	#%% build the ros timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
	
    def timer_callback(self):
    	#%% read data from br
        self.neural_buffer[:] = 0
        GetNeuralData(self.neural_buffer,self._spikes)
        
        #%% smooth data to get ir
        self._neural_buffer_deque.append(self.neural_buffer)
        
        #%% broadcast neural data (ir)
        self.msg.ir = deque_convolve(self._neural_buffer_deque, self._smoothed_parameter)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: {}'.format(str(self.msg.ir)))



def main(args=None):
    rclpy.init(args=args)

    blackrock_ir_publisher = BlackrockIrPublisher()

    rclpy.spin(blackrock_ir_publisher)

    blackrock_ir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
