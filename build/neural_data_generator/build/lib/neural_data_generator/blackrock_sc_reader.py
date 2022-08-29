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
from interfaces.msg import Ir

class BlackrockIrPublisher(Node):

    def __init__(self):
        super().__init__('blackrock_ir_publisher')
        
        self.declare_parameter('timer_period', 0.02)
        self.declare_parameter('chn_num', 280)
        
        my_param = self.get_parameter('timer_period').get_parameter_value()
        self.get_logger().info('Publishing: {}'.format(str(my_param.double_value)))
        my_param = self.get_parameter('chn_num').get_parameter_value()
        self.get_logger().info('Publishing: {}'.format(str(my_param.integer_value)))
        
        self.publisher_ = self.create_publisher(Ir, 'topic', 10)
        self.msg = Ir()

        self._chn_num = 280
        self.msg.ir = [0.0 for i in range(self._chn_num)]
        timer_period = 0.02  # seconds
        
        # connect to blackrock
        while True:
            con_params = cbpy.defaultConParams()
            re,con = cbpy.open(instance = 0,
                               connection = 'default',
                               parameter = con_params)
            
            buffer_par = {'double':True,'event_length':15000}
            cbpy.trial_config(instance = 0,reset = True,buffer_parameter = buffer_par,nocontinuous = True,nocomment = True)
            time.sleep(2)
            ree,event = cbpy.trial_event(instance = 0,reset = True)
            
            time.sleep(0.1)
            cbpy_close_marker = 0
            try:
                print(event[-1][1]['events'])
                break
            except IndexError:
                print("catch data wrong")
                while cbpy_close_marker!=2:
                    cbpy_close_marker = cbpy.close()
                    time.sleep(0.2)
                continue
            time.sleep(1)
            
            print("fail")
            while cbpy_close_marker!=2:
                cbpy_close_marker = cbpy.close()
                time.sleep(0.2)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ree,event = cbpy.trial_event(instance = 0,reset = True)
        self.msg.ir = [0.0 for i in range(self._chn_num)]
        if len(event)!=0:
            for i in event: self.msg.ir[i[0]-1] = i[1]['timestamps'][0].shape[0]    
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: {}'.format(str(self.msg.ir)))



def main(args=None):
    rclpy.init(args=args)

    blackrock_ir_publisher = BlackrockIrPublisher()

    rclpy.spin(blackrock_ir_publisher)

    blackrock_ir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
