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
import traceback
from neural_decoding.decoder.decoder import decoder
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
        parameters['algorithm'] = 'refit_kf_lcy_2d'
        
        self.declare_parameter('system', 0)
        self.declare_parameter('group', 0)
        self.declare_parameter('state', 0)
        self.declare_parameter('algorithm', 'refit_kf_lcy_2d')

        parameters['system'] = self.get_parameter('system').get_parameter_value().integer_value
        parameters['group'] = self.get_parameter('group').get_parameter_value().integer_value
        parameters['state'] = self.get_parameter('state').get_parameter_value().integer_value
        parameters['algorithm'] = self.get_parameter('algorithm').get_parameter_value().string_value
        
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
        
        self._par = None
        self._decoding_element = decoder[parameters['algorithm']]()
        self._neural_data = []
        self._wait = parameters['wait'] 
        self._decoding_state = [0.0]
    
    def decoding_element_listener_callback(self, msg):
    
        self._decoding_element_de = msg.de
        self._par = pickle.loads(bytes(list(msg.de)))
        self._decoding_element.model_update(self._par)
    
    def neural_data_listener_callback(self, msg):
        decoding_input = np.array(msg.y_observation)[np.newaxis,:]
        try:
            self._decoding_state = list(self._decoding_element.predict(decoding_input).squeeze())
        except Exception as e:
            self.get_logger().error(traceback.format_exc().replace('\n','\o'))
    
    def decoding_element_predict_callback(self, request, response):      
        response.res = self._decoding_state        
        self.get_logger().info('Publishing: outcoming response : {}'.format(str(response.res)))
        return response

def main(args=None):
    rclpy.init(args=args)
    decoding_element_predictor = DecodingElementPredictor()
    rclpy.spin(decoding_element_predictor)
    decoding_element_predictor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
