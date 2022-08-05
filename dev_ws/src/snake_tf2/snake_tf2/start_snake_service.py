from std_srvs.srv import Empty
from snake_interfaces.srv import TurtlePos
from snake_tf2.snake_tf2_listener import FrameListener

import rclpy
from rclpy.node import Node

import math

class StartSnakeService(Node):

    def __init__(self):
        super().__init__('start_snake_service')
        self.srv = self.create_service(
            Empty, 'start_turtlesim_snake', self.start_turtlesim_snake)
        self.get_logger().info('Start snake service created')

    def start_turtlesim_snake(self, request, response):
        self.get_logger().info(
            'Start snake service CALLED -> Starting normal execution')
        self.get_logger().info(
            f'request: {request} | response: {response}')
        return response 


def main(args=None):
    rclpy.init(args=args)

    start_service = StartSnakeService()

    rclpy.spin_once(start_service)


if __name__ == '__main__':
    main()