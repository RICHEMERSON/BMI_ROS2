import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from interfaces.msg import DecoderElement
from sklearn.linear_model import LinearRegression
import pickle
import time


def timer_callback2(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.get_logger().info('Publishing: "%s"' % msg.data)

class MinimalPublisher(Node):

    def __init__(self):
            
        super().__init__('minimal_publisher')
        
        self.declare_parameter('parameters', 'asd')
        parameters = self.get_parameter('parameters').get_parameter_value()
        print(parameters)
        
        self.__node = 'node_2'
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_byte_ = self.create_publisher(DecoderElement, 'topic_byte', 10)
        timer_period = 0.5  # seconds
        self.timer1 = self.create_timer(timer_period, self.timer_callback)
        exec('self.timer2 = self.create_timer(timer_period, self.timer_callback2)')
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().error('aaa')
        self.i += 1
        
        lr = LinearRegression()
        msg_byte = DecoderElement()
        msg_byte.de = list(pickle.dumps(lr))
        self.publisher_byte_.publish(msg_byte)

exec('MinimalPublisher.timer_callback2 = timer_callback2')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
