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
from interfaces.msg import State
from interfaces.srv import DecodingService
from collections import deque
from queue import Queue
from shared_memory_support import Shared_Array
from psychopy import visual, core
from psychopy import data
import pickle
from multiprocessing import shared_memory
import datetime
import os
import socket

class MonkeylogicServer(Node):

    def __init__(self):
        #+-----------------------------------------------------------------------
        # initialize node
        #+-----------------------------------------------------------------------
        
        #%% use super to initialize ros node with 'passive_data_integrator' field
        super().__init__('monkeylogic_server')
        
        #+-----------------------------------------------------------------------
        # set parameters
        #+-----------------------------------------------------------------------
        
        #%% default parameters
        parameters = {}
        parameters['system'] = 0
        parameters['group'] = 0
        parameters['state'] = 0
        parameters['decoding_element'] = 'decoder_0'
        
        self.state_publisher_ = self.create_publisher(State, '/system_{}/group_{}/state_{}/state'.format(
                                               parameters['system'], parameters['group'], parameters['state']), 1
                                               )
        self.desired_state_publisher_ = self.create_publisher(State, '/system_{}/group_{}/state_{}/desired_state'.format(
                                               parameters['system'], parameters['group'], parameters['state']), 1
                                               )
        
        '''
        self.cli = self.create_client(DecodingService, '/system_{}/group_{}/state_{}/{}'.format(
                                     parameters['system'], parameters['group'], parameters['state'],parameters['decoding_element'])
                                     )
                                     
        while not self.cli.wait_for_service(timeout_sec=0.02):
            self.get_logger().info('service not available, waiting again...')
        self.req = DecodingService.Request()'''

    def send_request(self, re):
        self.req.req = re
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    
    rclpy.init(args=args)

    bhvip='192.168.137.2'
    bhvport=7700
    bhvaddr=(bhvip,bhvport)
    udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    udp_socket.bind(bhvaddr)
    state_msg = State()
    monkeylogic_server = MonkeylogicServer()
    
    bhvip='192.168.137.2'
    bhvport=7711
    bhvaddr=(bhvip,bhvport)
    udp_socket_pos=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    udp_socket_pos.bind(bhvaddr)
    
    bhvip='192.168.137.3'
    bhvport=8822
    bhvaddr=(bhvip,bhvport)
    udp_socket_decode=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    
    while True:
        mo_trial = float(udp_socket.recv(1024).decode())
        monkeylogic_server.get_logger().info('Publishing: MO: {}'.format(mo_trial))
        state_msg.x_state = [mo_trial, 0.0, 0.0]
        monkeylogic_server.desired_state_publisher_.publish(state_msg)
        
        state_msg.x_state = [float(i) for i in udp_socket_pos.recv(1024).decode().split()]
        monkeylogic_server.get_logger().info('Publishing: pos: {}'.format(state_msg)) 
        monkeylogic_server.desired_state_publisher_.publish(state_msg)
        # udp_socket_decode.sendto(str(monkeylogic_server.send_request(True)).encode("utf-8"), bhvaddr)
        
        


if __name__ == '__main__':
    main()
