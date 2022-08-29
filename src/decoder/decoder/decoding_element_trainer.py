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
from interfaces.msg import DecoderElement
from collections import deque
from queue import Queue
from sklearn.linear_model import MultiTaskLasso

#%% default parameters
parameters = {}
parameters['system'] = 0
parameters['group'] = 0
parameters['decoding_element'] = 'decoder'
parameters['algorithm'] = 'wiener_filter'

class DecodingElementTrainer(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('/system/group/decoding_element_trainer')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% declare parameters    
        self.declare_parameter('parameters', json.dumps(parameters))
        
        #%% get parameters
        parameters = json.loads(self.get_parameter('parameters').get_parameter_value().string_value)
        
        self.subscription_sample = self.create_subscription(
            Sample, '/system_{}/group_{}/integrator/state', self.sample_listener_callback, 1
            )
        self.subscription_sample  # prevent unused variable warning
        
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.decoding_element_talker_callback)
        self.publisher_ = self.create_publisher(DecoderElement, '/system_{}/group_{}/trainer/decoding_element', 1)
        
        self._sample_buffer = Queue()
        self._x_state = deque(maxlen = 100000)
        self._y_observation = deque(maxlen = 100000)
        self._decoding_element = MultiTaskLasso()
        self._decoding_element_msg = DecoderElement()
        
    def decoding_element_talker_callback(self):
        
        if not self._sample_buffer.empty():
            
            while not self._sample_buffer.empty():
                sample = self._sample_buffer.get()
                self._x_state.append(sample.x_state)
                self._y_observation.append(sample.y_observation)
            self._decoding_element.fit(np.array(self._y_observation), np.array(self._x_state))
            self._decoding_element_msg.de = list(pickle.dumps(self._decoding_element))
            self.publisher_.publish(self._decoding_element_msg)
            self.get_logger().info('Publishing: decoding element: {}'.format(str(self._decoding_element_msg.de)))

    def sample_listener_callback(self, msg):
    
        self.get_logger().info('Recieving: integrated data: [x_state : {}, y_observation : {}]'.format(str(msg.x_state), str(msg.y_observation)))
        self._sample_buffer.put(msg)            

def main(args=None):
    rclpy.init(args=args)
    decoding_element_trainer = DecodingElementTrainer()
    rclpy.spin(decoding_element_trainer)
    decoding_element_trainer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
