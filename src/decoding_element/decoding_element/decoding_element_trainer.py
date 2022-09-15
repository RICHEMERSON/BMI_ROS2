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

def trainer(decoding_element, sample_buffer, de_buffer):

      _x_state = deque(maxlen = 100000)
      _y_observation = deque(maxlen = 100000)
      
      while True: 
         
         if not sample_buffer.empty():
             
             while not sample_buffer.empty():
                 sample = sample_buffer.get()
                 _x_state.append(sample.x_state)
                 _y_observation.append(sample.y_observation)
             
             if len(_y_observation)<10:
                 continue
             
             decoding_element.fit(np.array(_y_observation), np.array(_x_state))
             _decoding_element_msg = list(pickle.dumps(decoding_element))
             de_buffer.put(_decoding_element_msg)
         

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
        parameters['algorithm'] = 'wiener_filter'
        
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
        
        self._decoding_element = LinearRegression()
        self._sample_buffer = Queue()
        self._de_buffer = Queue()
        
        #%% build sub process for consideration of GIL
        trainer_process = Process(target=trainer, args=(self._decoding_element, self._sample_buffer, self._de_buffer))
        trainer_process.daemon = True
        trainer_process.start()
        
    def decoding_element_talker_callback(self):
        _decoding_element_msg = DecoderElement()
        while not self._de_buffer.empty():
            _decoding_element_msg.de = self._de_buffer.get()
            self.publisher_.publish(_decoding_element_msg)
            self.get_logger().info('Publishing: decoding element: {}'.format(str(_decoding_element_msg.de)))

    def sample_listener_callback(self, msg):
        # self.get_logger().info('Recieving: integrated data: [x_state : {}, y_observation : {}]'.format(str(msg.x_state), str(msg.y_observation)))
        self._sample_buffer.put(msg)            

def main(args=None):
    rclpy.init(args=args)
    decoding_element_trainer = DecodingElementTrainer()
    rclpy.spin(decoding_element_trainer)
    decoding_element_trainer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
