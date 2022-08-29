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
from interfaces.srv import PassiveObservation, State
from collections import deque
from queue import Queue

parameters = {}
parameters['system'] = 0
parameters['group'] = 0

class PassiveDataIntegrator(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('/system/group/passive_data_integrator')
    
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
        
        #%% initialize subscriber
        self.subscription_neural_data = self.create_subscription(
            Ir, '/system_{}/observation'.format(str(system)), self.neural_data_listener_callback, 1
            )
        self.subscription_neural_data  # prevent unused variable warning
        
        self.subscription_state = self.create_subscription(
            State, '/system_{}/group_{}/state'.format(str(system),str(group)), self.state_listener_callback, 1
            )
        self.subscription_state  # prevent unused variable warning
        
        #%% initialize publisher
        self.publisher_ = self.create_publisher(
                Sample, '/system_{}/group_{}/integrated_data'.format(str(system),str(group)), 100
                )
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------    
        
        self._neural_buffer = None # share recieved neural data
        self._sample = Sample() # construct msg for publisher

    def neural_data_listener_callback(self, msg):
        
        self.get_logger().info('Recieving: neural data: {}'.format(str(msg.ir)))
        
        self._neural_buffer = np.array(msg.ir)[self._selected_chn_index] # bind the recieved neural data to attribute
        self.get_logger().info('Recieving: grouped neural data: {}'.format(str(list(self._neural_buffer))))

    def state_listener_callback(self, msg):
        
        self.get_logger().info('Recieving: state data: {}'.format(str(msg.state)))
        
        # send integrated data to decoding element trainer
        if self._neural_buffer is not None:
                self._sample.x_state = msg.state
                self._sample.y_observation = self._neural_buffer[-1]
                self.publisher_.publish(self._sample)
                self.get_logger().info('Publishing: integrated data: [x_state : {}, y_observation : {}]'.format(str(self._sample.x_state), str(self._sample.y_observation)))

def main(args=None):
    rclpy.init(args=args)
    passive_data_integrator = PassiveDataIntegrator()
    rclpy.spin(passive_data_integrator)
    passive_data_integrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
