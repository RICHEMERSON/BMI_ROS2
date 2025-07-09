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
# from neural_decoding.decoder.decoder import decoder

####################### TCP#######################################################################################################################
import socket
def recv_all(sock):
    buffer = b''
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buffer += chunk
        try:
            json.loads(buffer.decode('utf-8'))  
            break
        except json.JSONDecodeError:
            continue
    return buffer

def tcp_send_predict_request(neural_input, server_ip='localhost', port=10030): 
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        s.connect((server_ip, port)) 
        print('CONNECT SUCCESS')
        payload = json.dumps({
            'neural_input': neural_input.tolist(),
            'timestamp': time.time(),
            'shape': neural_input.shape
        })
        s.sendall(payload.encode('utf-8'))
        print(f"[Predictor] Sent neural data to controller at {server_ip}:{port}")
        
        
        result = recv_all(s)
        response = json.loads(result.decode('utf-8'))
        s.close()
        
       
        prediction = response.get('prediction', None)
        if prediction is not None:
            print(f"[Predictor] Received prediction: {prediction}")
            return prediction
        else:
            raise RuntimeError(f"No prediction in response: {response}")
            
    except Exception as e:
        raise RuntimeError(f"TCP connection error: {e}")
        
####################################################################################################################################################################  

def model_inference(observation_q, state_q, error_buffer):

    par = None
    # loop_time = []
    while True:
        t1=time.time()
        # while not decoding_element_q.empty():
        #     par = decoding_element_q.get()
        #     decoding_element.model_update(par)          
    
        # if not par is None:
        try:    
                print('##########################')
                decoding_input = observation_q.get()
                print(decoding_input)
                # start = time.time()
                # decoding_state = decoding_element.predict(decoding_input)
                decoding_state = tcp_send_predict_request(decoding_input)
                # loop_time.append(time.time() - start)
                state_q.put([decoding_state, decoding_input])
        except Exception as e:
                error_buffer.put(traceback.format_exc().replace('\n','\o'))
                # pass

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
        
        self._decoding_state = None
        self._par = None
        # self._decoding_element = decoder[parameters['algorithm']]()
        self._neural_data = []
        self._wait = parameters['wait']
        
        self._observation_q = Queue()
        self._state_q = Queue()
        # self._decoding_element_q = Queue()
        self._error_buffer = Queue()
        
        # self._decoding_element_de = None
        
        #%% build sub process for consideration of GIL
        predictor_process = Process(target=model_inference, args=(self._observation_q, self._state_q, 
                                                                  self._error_buffer))
        predictor_process.daemon = True
        predictor_process.start()
    
    def decoding_element_listener_callback(self, msg):
        pass
        # self.get_logger().info('Recieving: decoding element: {}'.format(str(msg.de)))
        # self._decoding_element_de = msg.de
        # self._par = pickle.loads(bytes(list(msg.de)))
        # self._decoding_element.model_update(self._par)
        # self._decoding_element_q.put(self._par)
    
    def neural_data_listener_callback(self, msg):
        
        # _decoding_state = State()
        # self.get_logger().info('Recieving: neural data: {}'.format(str(msg.y_observation)))
        self._observation_q.put(np.array(msg.y_observation)[np.newaxis,:])
        readout = None

        while not self._state_q.empty():
            readout = self._state_q.get()
            self._decoding_state = readout[0]
            # par = list(pickle.dumps(readout[-1]))
        
        while not self._error_buffer.empty():
            self.get_logger().error(self._error_buffer.get())
        
        if not readout is None:
            # self.get_logger().info('Publishing: decoding state: {}, input: {}'.format(str(readout[0]), str(list(readout[1].squeeze()))))
            pass
    
    def decoding_element_predict_callback(self, request, response):
        
        response.res = [0.0] if self._decoding_state is None else list(self._decoding_state.squeeze())
        
        if not self._decoding_state is None:
            # self.get_logger().info('Servicing: decoding element: incoming request : {}, outcoming response : {}'.format(str(request.req), str(response.res)))
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
