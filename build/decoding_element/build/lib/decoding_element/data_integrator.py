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
from interfaces.msg import PassiveObservation, State, Sample
from cerebus import cbpy
import time
from collections import deque
from queue import Queue
import pickle

#%% subscriber template
interface_dict = {}
interface_dict['observation'] = PassiveObservation()
interface_dict['state'] = State()

def _declare_subscriber(index, data_type):
    index_str = str(index)
    group_name = data_type+index_str
    subscribe_template = f"""

passive_data_integrator.subscription_{group_name} = passive_data_integrator.create_subscription(
            PassiveObservation if '{data_type}' == 'observation' else State, passive_data_integrator._entry_{data_type}_groups[{index_str}], passive_data_integrator.{group_name}_listener_callback, 1
            )
passive_data_integrator.subscription_{group_name}  # prevent unused variable warning

passive_data_integrator.publisher_{group_name} = passive_data_integrator.create_publisher(
                PassiveObservation if '{data_type}' == 'observation' else State, passive_data_integrator._entry_{data_type}_groups[{index_str}], 1
                )
    """

    return subscribe_template

def _declare_subscriber_callback(index, data_type):
    
    index_str = str(index)
    group_name = data_type+index_str
    subscribe_function_template = f"""

def {group_name}_listener_callback(self, msg):
    # self.get_logger().info('Recieving: {group_name}: '+str(msg))
    msg_data = msg.y_observation if '{data_type}' == 'observation' else msg.x_state
    self._shared_{data_type}_buffer[{index}].append(msg_data)
    
    concatenate_buffer = []
    po_msg = interface_dict['{data_type}']
    
    for index, i in enumerate(self._shared_{data_type}_buffer):
        if len(i)==0:
            concatenate_buffer.clear()
            break
        else:
            concatenate_buffer = concatenate_buffer + \
                       list(np.array(i[-1])[self._parameters['entry_group_index'][index]]) \
                            if 'entry_group_index' in self._parameters else list(np.array(i[-1]))
    
    if len(concatenate_buffer)!=0:
    
        if '{data_type}' == 'observation':
            po_msg.y_observation = list(concatenate_buffer)
            po_msg.y_shape = [len(concatenate_buffer),1]
        else:
            po_msg.x_state = list(concatenate_buffer)
            
        self.{data_type}_publisher_.publish(po_msg)
        # self.get_logger().info('Recieving: grouped {group_name}: '+str(po_msg))
        for i in self._shared_{data_type}_buffer: 
            i.clear()  
    """
    
    return subscribe_function_template

def _synchro_concatenation_subscriber(self, parameters, shared_buffer, group_name, data_type):
    
    print('OK')
    concatenate_buffer = []
    po_msg = interface_dict[data_type]
    
    for index, i in enumerate(shared_buffer):
        if len(i)==0:
            concatenate_buffer.clear()
            break
        else:
            group_index_key = 'entry_{}_group_index'.format(data_type)
            concatenate_buffer = concatenate_buffer + \
                       np.array(i[-1])[parameters[group_index_key]] \
                            if group_index_key in parameters else np.array(i[-1])
            
    if len(concatenate_buffer)!=0:
    
        if data_type is 'observation':
            po_msg.y_observation = list(concatenate_buffer)
            po_msg.y_shape = [len(concatenate_buffer),1]
        else:
            po_msg.x_state = list(concatenate_buffer)
            
        exec('self.{}_publisher_.publish(po_msg)'.format(group_name))
        self.get_logger().info('Recieving: grouped {}: {}'.format(group_name, str(po_msg)))
        for i in concatenate_buffer: i.clear()

class PassiveDataIntegrator(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('passive_data_integrator')
    
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['state'] = 0
        parameters['AlignedData'] = 'observation'
        parameters['entry_observation_groups'] = ['/system_{}/passive_observation'.format(str(parameters['system']))]
        parameters['entry_state_groups'] = ['/system_{}/group_{}/state_{}/desired_state'.format(parameters['system'], parameters['group'], parameters['state'])]

        #%% default entry group topic
        entry_observation_groups = parameters['entry_observation_groups']

        #%% default entry state topic
        entry_state_groups = parameters['entry_state_groups']
        
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
        self.observation_publisher_ = self.create_publisher(
                PassiveObservation, '/system_{}/group_{}/integrator/observation'.format(parameters['system'], parameters['group']), 1
                )
        self.state_publisher_ = self.create_publisher(
                State, '/system_{}/group_{}/integrator/state'.format(parameters['system'], parameters['group']), 1
                )
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------    
        
        self._neural_buffer = None # share recieved neural data
        self._sample = Sample() # construct msg for publisher
        self._shared_observation_buffer = [[] for _ in range(len(entry_observation_groups))]
        self._shared_state_buffer = [[] for _ in range(len(entry_state_groups))]
        self._parameters = parameters
        self._entry_observation_groups = entry_observation_groups
        self._entry_state_groups = entry_state_groups

    def neural_data_listener_callback(self, msg):
        
        # self.get_logger().info('Recieving: integrator observation: {}'.format(str(msg)))
        self._observation = msg.y_observation
        
        # send integrated data to decoding element trainer
        if self._parameters['AlignedData']=='observation':
                if self._state is not None:
                        self._sample.x_state = self._state
                        self._sample.y_observation = self._observation
                        self.publisher_.publish(self._sample)
                        self.get_logger().info('Publishing: integrated data: [x_state : {}, y_observation : {}]'.format(str(self._sample.x_state), str(self._sample.y_observation)))
                        self._state = None

    def state_listener_callback(self, msg):
        
        # self.get_logger().info('Recieving: integrator state data: {}'.format(str(msg)))
        self._state = msg.x_state
        
        # send integrated data to decoding element trainer
        if self._parameters['AlignedData']=='state':
                if self._observation is not None:
                        self._sample.x_state = self._state
                        self._sample.y_observation = self._observation
                        self.publisher_.publish(self._sample)
                        self.get_logger().info('Publishing: integrated data: [x_state : {}, y_observation : {}]'.format(str(self._sample.x_state), str(self._sample.y_observation)))
                        self._observation = None

def main(args=None):
    
    rclpy.init(args=args)
    passive_data_integrator = PassiveDataIntegrator()
    
    for index in range(len(passive_data_integrator._entry_observation_groups)):
        group_name = 'observation'+str(index)
        exec(_declare_subscriber_callback(index, 'observation'))
        exec(f'PassiveDataIntegrator.{group_name}_listener_callback = {group_name}_listener_callback')
        exec(_declare_subscriber(index, 'observation'))
        
    for index in range(len(passive_data_integrator._entry_state_groups)):
        group_name = 'state'+str(index)
        exec(_declare_subscriber_callback(index, 'state'))
        exec(f'PassiveDataIntegrator.{group_name}_listener_callback = {group_name}_listener_callback')
        exec(_declare_subscriber(index, 'state'))
    
    rclpy.spin(passive_data_integrator)
    passive_data_integrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
