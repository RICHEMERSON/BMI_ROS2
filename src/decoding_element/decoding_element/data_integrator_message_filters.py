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
import message_filters
import numpy as np
from interfaces.msg import PassiveObservation, State, Sample
#from cerebus import cbpy
import time
from collections import deque
from queue import Queue
import pickle
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class PassiveDataIntegratorMessageFilters(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('PassiveDataIntegratorMessageFilters')
    
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['state'] = 0

        self.sample_msg = Sample()
        
        #%% declare parameters    
        # self.declare_parameter('parameters', list(pickle.dumps(parameters)))
        self.declare_parameter('system', 0)
        self.declare_parameter('group', 0)
        self.declare_parameter('state', 0)

        parameters['system'] = self.get_parameter('system').get_parameter_value().integer_value
        parameters['group'] = self.get_parameter('group').get_parameter_value().integer_value
        parameters['state'] = self.get_parameter('state').get_parameter_value().integer_value

        #%% get parameters
        # parameters = pickle.loads(bytes(list(self.get_parameter('parameters').get_parameter_value().integer_array_value)))
        
        #%% logging parameters
        for par in parameters:
            self.get_logger().info('Parameters: {}: {}'.format(par, str(parameters[par])))
        
        #+-----------------------------------------------------------------------
        # initialize communication module and use callback function
        #+-----------------------------------------------------------------------

        #%% initialize publisher
        
        # integrate state and observation
        self.subscription_observation = message_filters.Subscriber(self,
            PassiveObservation, '/system_{}/passive_observation'.format(str(parameters['system']))
            )
        self.subscription_observation  # prevent unused variable warning
        
        self.subscription_state = message_filters.Subscriber(self,
            State, '/system_{}/group_{}/state_{}/desired_state'.format(parameters['system'], parameters['group'], parameters['state'])
            )
        self.subscription_state  # prevent unused variable warning

        self.ts = message_filters.ApproximateTimeSynchronizer([self.subscription_observation, self.subscription_state], 10, 0.1, allow_headerless=True) # allow_headerless=True,可以不使用时间戳
        self.ts.registerCallback(self.callback)

        # integrate state and observation
        self.subscription_observation_predictor = self.create_subscription(
            PassiveObservation, '/system_{}/passive_observation'.format(str(parameters['system']),str(parameters['group'])), self.neural_data_listener_callback, 1
            )
        self.subscription_observation_predictor  # prevent unused variable warning


        #%% initialize publisher
        self.publisher_integrate_data_ = self.create_publisher(
                Sample, '/system_{}/group_{}/integrator/integrated_data'.format(parameters['system'], parameters['group']), 1
                )
        
        self.publisher_observation_ = self.create_publisher(
                PassiveObservation, '/system_{}/group_{}/integrator/observation'.format(parameters['system'], parameters['group']), 1
        )

    def callback(self,passiveObservation,state):
        # self.publisher_observation_.publish(passiveObservation)
        self.sample_msg.x_state = state.x_state
        self.sample_msg.y_observation = passiveObservation.y_observation
        self.publisher_integrate_data_.publish(self.sample_msg)
        self.get_logger().info('Publishing: integrated_data: {}'.format(str(self.sample_msg)))

    def neural_data_listener_callback(self, msg):
        self.publisher_observation_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    passive_data_integrator_filters = PassiveDataIntegratorMessageFilters()  
    rclpy.spin(passive_data_integrator_filters)
    passive_data_integrator_filters.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
