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
from interfaces.msg import DecoderElement, Ir
from collections import deque
from queue import Queue
from sklearn.linear_model import MultiTaskLasso
import pickle
from interfaces.srv import Decoder

class DecodingElementPredictor(Node):

    def __init__(self):
    
        super().__init__('decoding_element_predictor')
        self.subscription = self.create_subscription(
            DecoderElement, 'trained_decoding_element', self.decoding_element_listener_callback, 10
            )
        self.subscription  # prevent unused variable warning
        
        self.subscription_neural_data = self.create_subscription(
            Ir, '/neural_data/ir', self.neural_data_listener_callback, 10
            )
        self.subscription_neural_data  # prevent unused variable warning
        
        self.decoding_element = None
        
        self.srv = self.create_service(Decoder, 'Decoder', self.decoding_element_predict_callback)
        self._neural_data = []
    
    def decoding_element_listener_callback(self, msg):
    
        self.get_logger().info('Recieving: decoding element: {}'.format(str(msg.de)))
        self.decoding_element = pickle.loads(bytes(list(msg.de)))
    
    def neural_data_listener_callback(self, msg):
        
        self.get_logger().info('Recieving: neural data: {}'.format(str(msg.ir)))
        self._neural_buffer = msg.ir
    
    def decoding_element_predict_callback(self, request, response):
        
        if self.decoding_element is None or len(self._neural_data)==0:
            response.res = [0.0]
        else:
            response.res = self.decoding_element.predict(np.array(self._neural_data))
            
        self.get_logger().info('Servicing: Decoder: [incoming request : {}, outcoming response : {}]'.format(str(request.req), str(response.res)))
                
            

def main(args=None):
    rclpy.init(args=args)
    decoding_element_trainer = DecodingElementTrainer()
    rclpy.spin(decoding_element_trainer)
    decoding_element_trainer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
