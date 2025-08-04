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
from collections import deque
from interfaces.srv import RequestData
from interfaces.msg import Sample
import numpy as np

class DataBufferNode(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('data_buffer_node')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['decoding_element'] = 'decoder'

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
        self.subscription_sample = self.create_subscription(
            Sample, '/system_{}/group_{}/integrator/integrated_data'.format(parameters['system'], parameters['group']), self.sample_listener_callback, 1
            )
        self.subscription_sample  # prevent unused variable warning
        # 创建服务接口供训练节点请求数据
        self.service = self.create_service(RequestData, 'request_data', self.handle_request_data)
        
    def data_callback(self, msg):
        # 将接收到的数据存入缓冲队列
        x_state = np.array(msg.x_state).copy()
        y_observation = np.array(msg.y_observation).copy()
        self.buffer.append({'x_state': x_state, 'y_observation': y_observation})
        # self.get_logger().info('Received data: x_state={}, y_observation={}'.format(x_state, y_observation))

    def handle_request_data(self, request, response):
        # 返回缓冲区中的所有数据
        if len(self.buffer) > 0:
            response.data = list(self.buffer)
            self.buffer.clear()  # 清空缓冲区
        else:
            response.data = []
        self.get_logger().info('Sent {} samples to trainer node.'.format(len(response.data)))
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = DataBufferNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
