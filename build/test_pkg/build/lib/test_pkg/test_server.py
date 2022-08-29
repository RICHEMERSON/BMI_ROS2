from interfaces.srv import Decoder

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Decoder, 'Decoder', self.Decoder_callback)

    def Decoder_callback(self, request, response):
        self.get_logger().info('Request: re: {}'.format(str(request.req)))
        response.res = [1.0,2.0,3.0]

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
