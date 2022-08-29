import sys

from interfaces.srv import Decoder
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Decoder, 'Decoder')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Decoder.Request()

    def send_request(self, re):
        self.req.req=re
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(True)
    minimal_client.get_logger().info(
        'Service: Decoder: req : {}, res : {}'.format(str(True),str(response.res))
        )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
