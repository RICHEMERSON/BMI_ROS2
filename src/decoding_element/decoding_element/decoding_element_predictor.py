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
from interfaces.msg import State, DecoderElement, PassiveObservation
from collections import deque
from multiprocessing import Queue
from sklearn.linear_model import MultiTaskLasso
import pickle
from interfaces.srv import DecodingService
import json
from multiprocessing import Process

def model_inference(observation_q, state_q, decoding_element_q):
    decoding_element = None
    while True:
        if not decoding_element_q.empty():
            while not decoding_element_q.empty():
                decoding_element = decoding_element_q.get()
    
        if decoding_element is not None:
            decoding_state = list(decoding_element.predict(observation_q.get()))
            state_q.put(decoding_state)

class DecodingElementPredictor(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('decoding_element_predictor')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['wait'] = False
        
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
        self.subscription_decoding_element = self.create_subscription(
            DecoderElement, '/system_{}/group_{}/trainer/decoding_element'.format(parameters['system'], parameters['group']), self.decoding_element_listener_callback, 1
            )
        self.subscription_decoding_element  # prevent unused variable warning
        
        self.subscription_observation = self.create_subscription(
            PassiveObservation, '/system_{}/group_{}/integrator/observation'.format(parameters['system'], parameters['group']), self.neural_data_listener_callback, 1
            )
        self.subscription_observation  # prevent unused variable warning
        
        #%% initialize service
        self.srv = self.create_service(
                DecodingService, '/system_{}/group_{}/predictor/decoding_service'.format(parameters['system'], parameters['group']), self.decoding_element_predict_callback
                )
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------    
        
        self._decoding_state = None
        self._decoding_element = None
        self._neural_data = []
        self._wait = parameters['wait']
        
        self._observation_q = Queue()
        self._state_q = Queue()
        self._decoding_element_q = Queue()
        
        #%% build sub process for consideration of GIL
        predictor_process = Process(target=model_inference, args=(self._observation_q, self._state_q, self._decoding_element_q))
        predictor_process.daemon = True
        predictor_process.start()
    
    def decoding_element_listener_callback(self, msg):
    
        # self.get_logger().info('Recieving: decoding element: {}'.format(str(msg.de)))
        self._decoding_element = pickle.loads(bytes(list(msg.de)))
        self._decoding_element_q.put(self._decoding_element)
    
    def neural_data_listener_callback(self, msg):
        
        _decoding_state = State()
        # self.get_logger().info('Recieving: neural data: {}'.format(str(msg.y_observation)))
        self._observation_q.put(np.array(msg.y_observation)[np.newaxis,:])

        while not self._state_q.empty():
            self._decoding_state = self._state_q.get()
        
        self.get_logger().info('Publishing: decoding state: {}'.format(str(self._decoding_state)))
    
    def decoding_element_predict_callback(self, request, response):
        
        response.res = [0.0] if self._decoding_state is None else self._decoding_state  
        
        if self._wait and self._decoding_element is not None:
            response.res = list(self._decoding_element.predict(np.array(msg.y_observation)[np.newaxis,:]))
                  
        self.get_logger().info('Servicing: decoding element: [incoming request : {}, outcoming response : {}]'.format(str(request.req), str(response.res)))

def main(args=None):
    rclpy.init(args=args)
    decoding_element_predictor = DecodingElementPredictor()
    rclpy.spin(decoding_element_predictor)
    decoding_element_trainer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
