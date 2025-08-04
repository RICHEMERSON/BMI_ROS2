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
import pickle
from interfaces.srv import DecodingService
import json
from multiprocessing import Process
import traceback

class PredictorBufferNode(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('predictor_buffer_node')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['wait'] = False
        
        self.declare_parameter('system', 0)
        self.declare_parameter('group', 0)
        self.declare_parameter('state', 0)

        parameters['system'] = self.get_parameter('system').get_parameter_value().integer_value
        parameters['group'] = self.get_parameter('group').get_parameter_value().integer_value
        parameters['state'] = self.get_parameter('state').get_parameter_value().integer_value
        
        #%% logging parameters
        for par in parameters:
            self.get_logger().info('Parameters: {}: {}'.format(par, str(parameters[par])))
        
        #+-----------------------------------------------------------------------
        # initialize communication module and use callback function
        #+-----------------------------------------------------------------------
        
        #%% initialize subscriber
        self.subscription = self.create_subscription(
            State, '/system_{}/group_{}/predictor/state'.format(parameters['system'], parameters['group']), self.state_callback, 1
        )

        #%% initialize service
        self.srv = self.create_service(
                DecodingService, '/system_{}/group_{}/predictor/decoding_service'.format(parameters['system'], parameters['group']), self.handle_request
                )

        self._decoding_state = [0.0]
    
    def state_callback(self, msg):
        self._decoding_state = list(msg.x_state)

    def handle_request(self, request, response):
        # 返回缓冲区中的所有数据
        response.res = self._decoding_state
        self.get_logger().info('Sent {} predictions.'.format(len(response.res)))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PredictorBufferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()