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
from interfaces.srv import RequestData
from sklearn.linear_model import LinearRegression
import numpy as np
import pickle
from interfaces.msg import DecoderElement
from neural_decoding.decoder.decoder import decoder
         

class DecodingElementTrainer(Node):

    def __init__(self):
        
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('decoding_element_trainer')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['decoding_element'] = 'decoder'
        parameters['algorithm'] = 'refit_kf_lcy_2d'

        self.declare_parameter('system', 0)
        self.declare_parameter('group', 0)
        self.declare_parameter('state', 0)
        self.declare_parameter('algorithm', 'refit_kf_lcy_2d')

        parameters['system'] = self.get_parameter('system').get_parameter_value().integer_value
        parameters['group'] = self.get_parameter('group').get_parameter_value().integer_value
        parameters['state'] = self.get_parameter('state').get_parameter_value().integer_value
        parameters['algorithm'] = self.get_parameter('algorithm').get_parameter_value().string_value
        
        #%% declare parameters    
        # self.declare_parameter('parameters', list(pickle.dumps(parameters)))
        
        #%% get parameters
        # parameters = pickle.loads(bytes(list(self.get_parameter('parameters').get_parameter_value().integer_array_value)))
        
        #%% logging parameters
        for par in parameters:
            self.get_logger().info('Parameters: {}: {}'.format(par, str(parameters[par])))
        
        #+-----------------------------------------------------------------------
        # initialize communication module and use callback function
        #+-----------------------------------------------------------------------
        
        #%% initialize timer & publisher
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.decoding_element_talker_callback)
        self.publisher_ = self.create_publisher(DecoderElement, '/system_{}/group_{}/trainer/decoding_element'.format(parameters['system'], parameters['group']), 1)
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------    
        
        self._decoding_element = decoder[parameters['algorithm']]()
        
    def decoding_element_talker_callback(self):
        # 请求数据缓冲节点的数据
        client = self.create_client(RequestData, 'request_data')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Data Buffer Node service not available.')
            return

        request = RequestData.Request()
        future = client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if len(response.data) > 0:
                # 提取数据并训练模型
                # x_states = np.array([sample.x_state for sample in response.data])
                # y_observations = np.array([sample.y_observation for sample in response.data])
                # self.get_logger().info('{}'.format([np.array(sample.x_state).shape for sample in response.data]))
                for data_i in response.data:
                    self._decoding_element.update(np.array(data_i.y_observation), np.array(data_i.x_state))

                self._decoding_element.fit()
                self.get_logger().info('Model trained with {} samples.'.format(len(response.data)))

                # 发布模型参数
                params = self._decoding_element.model_par()
                if params is not None:
                    msg = DecoderElement()
                    msg.de = list(pickle.dumps(params))
                    self.publisher_.publish(msg)
                    self.get_logger().info('Published model parameters.')
            else:
                self.get_logger().info('No data available for training.')
        except Exception as e:
            self.get_logger().error('Failed to train model: {}'.format(str(e)))
            
def main(args=None):
    rclpy.init(args=args)
    decoding_element_trainer = DecodingElementTrainer()
    rclpy.spin(decoding_element_trainer)
    decoding_element_trainer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
