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
from interfaces.srv import PassiveObservation, State, Sample
from collections import deque
from queue import Queue

#%% default parameters
parameters = {}
parameters['system'] = 0
parameters['group'] = 0
parameters['AlignedData'] = 'observation'

#%% default entry group topic
entry_observation_groups = ['/system_{}/active_observation'.format(str(parameters['system'])]

#%% default entry state topic
entry_state_groups = ['/system_{}/group_{}/state'.format(str(parameters['system']),str(parameters['group']))]

#%% subscriber template
interface_dict = {}
interface_dict['observation'] = PassiveObservation()
interface_dict['state'] = State()

subscribe_template = """

self.subscription_{} = self.create_subscription(
            PassiveObservation if '{}' == 'observation else State, entry_{}_groups[{}], self.{}_listener_callback, 1
            )
self.subscription_{}  # prevent unused variable warning

self.publisher_{} = self.create_publisher(
                PassiveObservation if '{}' == 'observation else State, entry_{}_groups[{}], 1
                )
"""
        
subscribe_function_template = """

def {}_listener_callback(self, msg):
    parameters = json.loads(self.get_parameter('parameters').get_parameter_value().string_value)
    self.get_logger().info('Recieving: {}: '+str(msg))
    msg_data = msg.y_observation if '{}' == 'observation else msg.x_state
    selected_msg_data = list(np.array(msg_data)[np.array(parameters['select_index'][{}][{}])])
    msg = PassiveObservation() if {} == 'observation' else State()
    if {} == 'observation':
        msg.y_observation = selected_msg_data
    else:
        msg.x_state = selected_msg_data
    self.publisher_{}.publish(msg)"""

def _declare_subscriber(index, data_type):
    
    group_name = data_type+str(index)
    return subscribe_template.format(
           group_name, data_type, data_type, index, group_name, 
           group_name, group_name, data_type, data_type, index
           )

def _declare_subscriber_callback(index, data_type):
    
    group_name = data_type+str(index)
    return subscribe_function_template.format(
           group_name, group_name, data_type, data_type, index, data_type, data_type, group_name
           )

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
        self.declare_parameter('entry_observation_groups', entry_observation_groups)
        self.declare_parameter('entry_state_groups', entry_state_groups)
        
        #%% get parameters
        parameters = json.loads(self.get_parameter('parameters').get_parameter_value().string_value)
        entry_observation_groups = self.get_parameter('entry_observation_groups').get_parameter_value().string_array_value
        entry_state_groups = self.get_parameter('entry_state_groups').get_parameter_value().string_array_value
        
        #%% logging parameters
        for par in parameters:
            self.get_logger().info('Parameters: {}: {}'.format(par, str(parameters[par])))
        
        for groups in entry_observation_groups:
            self.get_logger().info('Parameters: entry observation group: {}'.format(groups))
        
        for groups in entry_state_groups:
            self.get_logger().info('Parameters: entry state group: {}'.format(groups))
        
        #+-----------------------------------------------------------------------
        # initialize communication module and use callback function
        #+-----------------------------------------------------------------------
        
        #%% initialize subscriber
        
        for index in range(len(entry_observation_groups)):
            exec(_declare_subscriber(index, 'observation'))
        self._shared_observation_buffer = [[] for _ in range(len(entry_observation_groups))]
        
        for index in range(len(entry_state_groups)):
            exec(_declare_subscriber(index, 'state'))
        self._shared_state_buffer = [[] for _ in range(len(entry_state_groups))]
        
        #%% initialize publisher
        
        # integrate multi source state and observation
        self.observation_publisher_ = self.create_publisher(
                PassiveObservation, '/system_{}/group_{}/integrator/observation'.format(str(parameters['system']),str(parameters['group'])), 1
                )
        self._observation = None
        self._state = None
        
        self.state_publisher_ = self.create_publisher(
                State, '/system_{}/group_{}/integrator/state'.format(str(parameters['system']),str(parameters['group'])), 1
                )
        
        # integrate state and observation
        self.subscription_observation = self.create_subscription(
            PassiveObservation, '/system_{}/group_{}/integrator/observation'.format(str(parameters['system']),str(parameters['group'])), self.neural_data_listener_callback, 1
            )
        self.subscription_observation  # prevent unused variable warning
        
        self.subscription_state = self.create_subscription(
            State, '/system_{}/group_{}/integrator/state'.format(parameters['system'], parameters['group']), self.state_listener_callback, 1
            )
        self.subscription_state  # prevent unused variable warning
        
        #%% initialize publisher
        self.publisher_ = self.create_publisher(
                Sample, '/system_{}/group_{}/integrator/integrated_data'.format(parameters['system'], parameters['group']), 1
                )
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------    
        
        self._neural_buffer = None # share recieved neural data
        self._sample = Sample() # construct msg for publisher

    def neural_data_listener_callback(self, msg):
        
        self.get_logger().info('Recieving: integrator observation: {}'.format(str(msg.observation)))
        self._observation = msg.observation
        
        # send integrated data to decoding element trainer
        if parameters['AlignedData']=='observation':
                if self._observation is not None:
                        self._sample.x_state = msg.state
                        self._sample.y_observation = self._observation
                        self.publisher_.publish(self._sample)
                        self.get_logger().info('Publishing: integrated data: [x_state : {}, y_observation : {}]'.format(str(self._sample.x_state), str(self._sample.y_observation)))

    def state_listener_callback(self, msg):
        
        self.get_logger().info('Recieving: integrator state data: {}'.format(str(msg.state)))
        self._state = msg.state
        
        # send integrated data to decoding element trainer
        if parameters['AlignedData']=='state':
                if self._observation is not None:
                        self._sample.x_state = msg.state
                        self._sample.y_observation = self._observation
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
