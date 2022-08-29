import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from interfaces.msg import DecoderElement
from sklearn.linear_model import LinearRegression
import pickle

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.subscription_byte = self.create_subscription(
            DecoderElement,
            'topic_byte',
            self.listener_byte_callback,
            10)
        self.subscription_byte  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    
    def listener_byte_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.de)
        decoder_element = pickle.loads(bytes(list(msg.de)))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
