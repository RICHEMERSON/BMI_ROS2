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
import json
from interfaces.msg import Sample, DecoderElement
from collections import deque
from multiprocessing import Queue
from sklearn.linear_model import LinearRegression
from multiprocessing import Process
import pickle
from neural_decoding.decoder.decoder import decoder
import traceback

def trainer(decoding_element, sample_buffer, de_buffer, error_buffer):
      
      while True: 
         
         if not sample_buffer.empty():
             
             try:
             
                 while not sample_buffer.empty():
                     sample = sample_buffer.get()
                     decoding_element.update(np.array(sample.y_observation), np.array(sample.x_state))
             
                 decoding_element.fit()
                 _decoding_element_msg = list(pickle.dumps(decoding_element))
                 de_buffer.put(_decoding_element_msg)
             
             except Exception as e:
                 error_buffer.put(traceback.format_exc().replace('\n','\o'))
         

class DecodingElementTrainer(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('decoding_element_trainer')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['decoding_element'] = 'decoder'
        parameters['algorithm'] = 'wiener_filter_synchronous'
        
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
        
        #%% initialize subscriber
        self.subscription_sample = self.create_subscription(
            Sample, '/system_{}/group_{}/integrator/integrated_data'.format(parameters['system'], parameters['group']), self.sample_listener_callback, 1
            )
        self.subscription_sample  # prevent unused variable warning
        
        #%% initialize timer & publisher
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.decoding_element_talker_callback)
        self.publisher_ = self.create_publisher(DecoderElement, '/system_{}/group_{}/trainer/decoding_element'.format(parameters['system'], parameters['group']), 1)
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------    
        
        self._decoding_element = decoder[parameters['algorithm']]()
        self._sample_buffer = Queue()
        self._de_buffer = Queue()
        self._error_buffer = Queue()
        
        self.counter = 0
        
        #%% build sub process for consideration of GIL
        trainer_process = Process(target=trainer, args=(self._decoding_element, self._sample_buffer, self._de_buffer, self._error_buffer))
        trainer_process.daemon = True
        trainer_process.start()
        
    def decoding_element_talker_callback(self):
        _decoding_element_msg = DecoderElement()
        self.counter = self.counter+1
        
        while not self._de_buffer.empty():
            _decoding_element_msg.de = self._de_buffer.get()
        
        while not self._error_buffer.empty():
            self.get_logger().error(self._error_buffer.get())
        
        if len(_decoding_element_msg.de)!=0:
            self.publisher_.publish(_decoding_element_msg)
            
        if len(_decoding_element_msg.de)!=0 and (self.counter%50)==0:
            self.get_logger().info('Publishing: decoding element: {}'.format(str(_decoding_element_msg.de)))

    def sample_listener_callback(self, msg):
        self.get_logger().info('Recieving: integrated data: x_state : {}, y_observation : {}'.format(str(msg.x_state), str(msg.y_observation)))
        self._sample_buffer.put(msg)            

def main(args=None):
    rclpy.init(args=args)
    decoding_element_trainer = DecodingElementTrainer()
    rclpy.spin(decoding_element_trainer)
    decoding_element_trainer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
