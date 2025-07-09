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
import time
from interfaces.msg import PassiveObservation
from collections import deque
import pickle
from trodesnetwork import socket

def deque_convolve(neural_buffer_deque, smoothed_parameter):
    return list((smoothed_parameter*np.array(neural_buffer_deque).T).sum(1))
    
import threading

import numba as nb

@nb.jit()
def intan_reader(spikeArray, chunksToRead, NeuralVector):
    spikeIndex = 0
    wrong = 0
    for chunk in range(chunksToRead):
        magicNumber, spikeIndex = readUint32(spikeArray, spikeIndex)     
        #assert magicNumber == 0x3ae2710f, 'Incorrect spike magic number'
        if magicNumber != 0x3ae2710f:
            wrong = 0
            for i in range(len(spikeArray)):
                magicNumber, spikeIndex = readUint32(spikeArray, spikeIndex)
                if magicNumber == 0x3ae2710f:
                    break
            
        nativeChannelName, spikeIndex = readChar5(spikeArray, spikeIndex)
        chnInt = int(nativeChannelName[2::])
        #singleTimestamp, spikeIndex = readUint32(spikeArray, spikeIndex)     
        #singleID, spikeIndex = readUint8(spikeArray, spikeIndex)
        NeuralVector[chnInt]=NeuralVector[chnInt]+1
    return wrong

class IntanIrPublisher(Node):

    def __init__(self):
    
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        print('initialize reader node')
        #%% use super to initialize ros node with 'blackrock_ir_publisher' field
        super().__init__('system_SpikeGadgtIrPublisher')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------        
        
        print('initialize/get parameters')
        parameters = {}
        parameters['timer_period'] = 0.5
        parameters['chn_num'] = 1024
        parameters['deque_buffer_length'] = 20
        parameters['smoothed_parameter'] = list(range(1024))
        parameters['system'] = 0
        parameters['time_buffer_length'] = 20
        
        #%% declare parameters
        self.declare_parameter('parameters', list(pickle.dumps(parameters)))
        
        #%% get parameters
        parameters = pickle.loads(bytes(list(self.get_parameter('parameters').get_parameter_value().integer_array_value)))
        
        #%% logging parameters
        for par in parameters:
            self.get_logger().info('Parameters: {}: {}'.format(par, str(parameters[par])))
        
        #+-----------------------------------------------------------------------
        # initialize communication module and use callback function
        #+-----------------------------------------------------------------------

        print('set callback')
        self.publisher_ = self.create_publisher(
                PassiveObservation, '/system_{}/passive_observation'.format(str(parameters['system'])), 1
                )
        self.timer = self.create_timer(parameters['timer_period'], self.neural_array_callback)
        self.msg = PassiveObservation()
        
        #+-----------------------------------------------------------------------
        # connect to blackrock
        #+-----------------------------------------------------------------------
        
        print('build connection')
        NeuralVector = np.zeros((parameters['chn_num'],))
        self._neural_buffer = NeuralVector
        spikes = socket.SourceSubscriber('source.spikes')

        def subscribe_spikes_thread():
           while True:
               spikeBytesToRead = spkwaveform.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
               spikeArray = spkwaveform.recv(spikeBytesToRead)
               spikeBytesToRead = len(spikeArray)
               w = intan_reader(spikeArray, chunksToRead, NeuralVector)
               if w==0: 
                   self.get_logger().info('wrong spike')
                   
        t1 = threading.Thread(target=subscribe_spikes_thread) #Insert into a thread
        t1.start()
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------   
        
        print('set attribution')
        self._neural_buffer = np.zeros((parameters['chn_num'], )) # shared recieved neural data
        self._neural_buffer_deque = deque(maxlen=parameters['deque_buffer_length']) # a deque buffer for preprocessing
        self._time_buffer_deque = deque(maxlen = parameters['time_buffer_length'])
        
        # modify the smoothed parameter
        self._smoothed_parameter = np.tile(np.array(parameters['smoothed_parameter']),(parameters['chn_num'],1))
        
        # save neural data for convolution
        self._neural_buffer_deque = deque(maxlen=parameters['deque_buffer_length'])
	
    def neural_array_callback(self):
        
        #%% smooth data to get ir
        self._neural_buffer_deque.append(self._neural_buffer.copy())
        if len(self._neural_buffer_deque)==self._neural_buffer_deque.maxlen:
            self._time_buffer_deque.append(deque_convolve(self._neural_buffer_deque, self._smoothed_parameter))
            self._neural_array = np.array(self._time_buffer_deque).T
        
            #%% broadcast neural data (ir)
            self.msg.y_observation = list(self._neural_array.flatten())
            self.msg.y_shape = list(self._neural_array.shape) 
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: PassiveObservation: {}'.format(str(self.msg)))
	
        self._neural_buffer[:] = 0
	
def main(args=None):
    rclpy.init(args=args)
    intan_passive_ir_publisher = IntanPassiveIrPublisher()
    rclpy.spin(intan_passive_ir_publisher)
    intan_passive_ir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
