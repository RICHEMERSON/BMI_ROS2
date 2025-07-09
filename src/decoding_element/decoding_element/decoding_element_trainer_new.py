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
#from cerebus import cbpy#############################################################################
import time
import json
from interfaces.msg import Sample, DecoderElement
from collections import deque
from multiprocessing import Queue
from sklearn.linear_model import LinearRegression
from multiprocessing import Process
import pickle
from neural_decoding.decoder.decoder import decoder
import traceback      

###################################################################################################################################
import socket
import threading
import queue
class DummyMsg:
    def __init__(self):
        self.x_state = [1.0, 2.0]
        self.y_observation = [[0.5, 0.3], [0.6, 0.4]]

####################################################################################################################################
#def trainer(decoding_element, sample_buffer, de_buffer, error_buffer):
#      print("[TRAINER PROCESS] Trainer process started")
#      while True: 
#         
#         if not sample_buffer.empty():
             
#             try:
#                 #print("[TRAINER PROCESS] Found samples in buffer")
#                 print(f'[TRAINER PROCESS] Processing sample - x_state: {sample.x_state}, y_observation shape: {len(sample.y_observation)}')###########
#                 while not sample_buffer.empty():
#                     sample = sample_buffer.get()
#                     print('[TRAINER] Got one sample from buffer.')
#                     decoding_element.update(np.array(sample.y_observation), np.array(sample.x_state))
             
#                decoding_element.fit()
#                 par = decoding_element.model_par()
                 
#                 if not par is None:
#                       print("[TRAINER PROCESS] Model fitted, sending parameters")###############
#                       _decoding_element_msg = list(pickle.dumps(par))
#                       de_buffer.put(_decoding_element_msg)
             
#             except Exception as e:
#                 print(f"[TRAINER PROCESS] Error: {str(e)}")###########
#                 error_buffer.put(traceback.format_exc().replace('\n','\o'))
#            #else:#############
#              #time.sleep(0.1)   ################  
########################################### new trainer version####################################################################
def trainer(decoding_element, sample_buffer, de_buffer, error_buffer):
    print("[TRAINER PROCESS] Trainer process started")
    while True: 
        if not sample_buffer.empty():
            try:
                # ���Ȼ�ȡsample
                sample = sample_buffer.get()
                print(f'[TRAINER PROCESS] Received sample data: x_state={sample.x_state}')
                
                # Ȼ��������
                decoding_element.update(np.array(sample.y_observation), np.array(sample.x_state))
                decoding_element.fit()
                par = decoding_element.model_par()
                
                if par is not None:
                    print("[TRAINER PROCESS] Model fitted, sending parameters")
                    _decoding_element_msg = list(pickle.dumps(par))
                    de_buffer.put(_decoding_element_msg)
            
            except Exception as e:
                print(f"[TRAINER PROCESS] Error: {str(e)}")
                error_buffer.put(traceback.format_exc().replace('\n','\o'))
        else:
            time.sleep(0.1)                  
#################################################end#############################################################################
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
        self.subscription_sample = self.create_subscription(
            Sample, '/system_{}/group_{}/integrator/integrated_data'.format(parameters['system'], parameters['group']), self.sample_listener_callback, 1
            )
        self.subscription_sample  # prevent unused variable warning
        
        # #%% initialize timer & publisher
        # timer_period = 0.02
        # self.timer = self.create_timer(timer_period, self.decoding_element_talker_callback)
        # self.publisher_ = self.create_publisher(DecoderElement, '/system_{}/group_{}/trainer/decoding_element'.format(parameters['system'], parameters['group']), 1)
        
        #+-----------------------------------------------------------------------
        # declare protect/private attribute for callback function
        #+-----------------------------------------------------------------------    
        
        self._decoding_element = decoder[parameters['algorithm']]()
        #self._sample_buffer = Queue()# replace bottom two lines
        #self._trainer_buffer = Queue()##################
        #self._tcp_buffer = Queue()#######################
        #self._tcp_buffer = queue.Queue()  ##############
        self._trainer_buffer = Queue()###########################
        self._tcp_buffer = queue.Queue()########################

        self._de_buffer = Queue()
        self._error_buffer = Queue()
        
        #print('excution start mei ne')
        #threading.Thread(target=self.ndt2_tcp_client_loop, daemon=True).start()####################################################
        ###########################################################################################################################
        #for _ in range(6):##################
          #self._sample_buffer.put(DummyMsg())################
        #def generate_dummy_data(self):##################
          #while True:################
            #dummy = DummyMsg()###############
            #self._trainer_buffer.put(dummy)#####################
            #self._tcp_buffer.put(dummy)###############
            #time.sleep(0.1) ######################################
        #for _ in range(6):###############
          #dummy = DummyMsg()############
          #self._trainer_buffer.put(dummy)##############
          #self._tcp_buffer.put(dummy)####################

        ###########################################################################################################################
        print('Starting trainer node...')
        threading.Thread(target=self.ndt2_tcp_client_loop, daemon=True).start()
        self.counter = 0
        
        #%% build sub process for consideration of GIL
        #trainer_process = Process(target=trainer, args=(self._decoding_element, self._sample_buffer, self._de_buffer, self._error_buffer))# replace bottom line ##################
        #trainer_process = Process(target=trainer, args=(self._decoding_element, self._trainer_buffer, self._de_buffer, self._error_buffer))##############################
        
        # trainer_process = Process(target=trainer, args=(self._decoding_element, self._trainer_buffer, self._de_buffer, self._error_buffer))
        # trainer_process.daemon = True
        # trainer_process.start()
        
    ###################################################################################################################    
#    def ndt2_tcp_client_loop(self):  
#        while True:
#            if not self._tcp_buffer.empty():
#                msg = self._tcp_buffer.get()
#                print('[TCP CLIENT] Attempting to send data via TCP...')
#                try:
#                    # Convert numpy arrays to lists for JSON serialization
#                    data = {
#                        'x_state': msg.x_state.tolist() if hasattr(msg.x_state, 'tolist') else list(msg.x_state),
#                        'y_observation': msg.y_observation.tolist() if hasattr(msg.y_observation, 'tolist') else list(msg.y_observation),
#                        'timestamp': time.time()
#                    }
#                    payload = json.dumps(data)
#
#                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#                        s.connect(('localhost', 8002))
#                        s.sendall(payload.encode('utf-8'))
#                        print('[TCP CLIENT] Data sent successfully')
#                        
#                        # Wait for response
#                        response = s.recv(8192)
#                        response_data = json.loads(response.decode('utf-8'))
#                        print('[TCP CLIENT] Received response:', response_data)
#
#                except Exception as e:
#                    print(f'[TCP CLIENT ERROR] Detailed error: {str(e)}')
#                    traceback.print_exc()
#            else:
#               time.sleep(0.1)
#               
##############################################################################################################################
    def ndt2_tcp_client_loop(self):
        print('[TCP CLIENT] Starting TCP client...')
        
        while True:
            try:
              
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect(('localhost', 10032))
                    print('[TCP CLIENT] Connected to server')
                    
                    while True:  
                        if not self._tcp_buffer.empty():
                            msg = self._tcp_buffer.get()
                            try:
                                # Convert numpy arrays to lists for JSON serialization
                                data = {
                                    'x_state': msg.x_state.tolist() if hasattr(msg.x_state, 'tolist') else list(msg.x_state),
                                    'y_observation': msg.y_observation.tolist() if hasattr(msg.y_observation, 'tolist') else list(msg.y_observation),
                                    'timestamp': time.time()
                                }
                                payload = json.dumps(data)
                                
                                print('[TCP CLIENT] Sending data...')
                                s.sendall(payload.encode('utf-8'))
                                
                                # Wait for response
                                response = s.recv(8192)
                                if response:
                                    response_data = json.loads(response.decode('utf-8'))
                                    print('[TCP CLIENT] Data sent successfully')
                                    
                            except Exception as e:
                                print(f'[TCP CLIENT ERROR] Error sending data: {e}')
                                break  
                                
                        time.sleep(0.001)  
                        
            except Exception as e:
                print(f'[TCP CLIENT ERROR] Connection error: {e}')
                print('[TCP CLIENT] Reconnecting in 1 second...')
                time.sleep(1)              
########################################################new ndt2 tcp client loop###############################################
#    def ndt2_tcp_client_loop(self):
#      print('[TCP CLIENT] Starting TCP client...')
#      s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#      connected = False
    
#      while True:
#        if not connected:
#            try:
#                s.connect(('localhost', 8002))
#                connected = True
#                print('[TCP CLIENT] Connected to server')
#            except Exception as e:
#                print(f'[TCP CLIENT] Connection failed: {str(e)}')
#                time.sleep(1) 
#                continue
#
#        if not self._tcp_buffer.empty():
#            msg = self._tcp_buffer.get()
#            try:
#                # Convert numpy arrays to lists for JSON serialization
#                data = {
#                    'x_state': msg.x_state.tolist() if hasattr(msg.x_state, 'tolist') else list(msg.x_state),
#                    'y_observation': msg.y_observation.tolist() if hasattr(msg.y_observation, 'tolist') else list(msg.y_observation),
#                    'timestamp': time.time()
#                }
#                payload = json.dumps(data)
#                
#                s.sendall(payload.encode('utf-8'))
#                print('[TCP CLIENT] Data sent successfully')
#                
#                response = s.recv(8192)
#                response_data = json.loads(response.decode('utf-8'))
#                print('[TCP CLIENT] Received response:', response_data)
                
#           except Exception as e:
#                print(f'[TCP CLIENT ERROR] Error during communication: {str(e)}')
#                connected = False
#                s.close()
#                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
#        else:
#            time.sleep(0.1)             
#               #########################################################################################################################
        
    def decoding_element_talker_callback(self):
        #################################################xia mian shi xin jia de 
        if self._de_buffer.empty():
          return
        ####################################################shang mian shi xin jia de 
        _decoding_element_msg = DecoderElement()
        self.counter = self.counter+1
        
        while not self._de_buffer.empty():
            _decoding_element_msg.de = self._de_buffer.get()
        
        while not self._error_buffer.empty():
            self.get_logger().error(self._error_buffer.get())
        
        if len(_decoding_element_msg.de)!=0:
            self.publisher_.publish(_decoding_element_msg)
            
        if len(_decoding_element_msg.de)!=0 and (self.counter%10)==0:
        # if len(_decoding_element_msg.de)!=0:
            self.get_logger().info('Publishing: decoding element: {}'.format(str(_decoding_element_msg.de)))

    def sample_listener_callback(self, msg):
        print(f"[TRAINER NODE] Received integrated data - x_state: {msg.x_state}, y_observation length: {len(msg.y_observation)}")#####
        self.get_logger().info('Recieving: integrated data: x_state : {}, y_observation : {}'.format(str(msg.x_state), str(msg.y_observation)))
        #self._sample_buffer.put(msg) #gai wei xia mian liang hang le 
        #self._trainer_buffer.put(msg)###################################################################
        #self._tcp_buffer.put(msg)    ###########################################################    
        self._trainer_buffer.put(msg)############
        print("[TRAINER NODE] Data put into trainer buffer")############
        self._tcp_buffer.put(msg) #############   
        print("[TRAINER NODE] Data put into TCP buffer")   #################

def main(args=None):
    rclpy.init(args=args)
    decoding_element_trainer = DecodingElementTrainer()
    rclpy.spin(decoding_element_trainer)
    decoding_element_trainer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
