from snake_interfaces.srv import TurtlePos
from snake_tf2.snake_tf2_listener import FrameListener

import rclpy
from rclpy.node import Node

import math

class StartSnakeServiceLocation(Node):

    def __init__(self):
        super().__init__('start_located_snake_service')
        self.srv = self.create_service(
            TurtlePos, 'start_turtlesim_snake', self.start_turtlesim_snake)
        self.get_logger().info('Start located snake service created')

    def start_turtlesim_snake(self, request, response):
        response.success= True
        self.get_logger().info(
            'Start located snake service CALLED -> Starting normal execution')
        self.get_logger().info(
            f'request: X={request.pos_x} | Y={request.pos_y} | angle={request.angle*360/2/math.pi} | response: {response}'
            )
        
        if request.pos_x and request.pos_y and request.angle:
            self.get_logger().info(
                'Sending service request for deployment ->->->'
            )
            snake_game_node = FrameListener(request.pos_x, request.pos_y, request.angle)
        else:
            snake_game_node = FrameListener()
        try:
            rclpy.spin(snake_game_node)
        except KeyboardInterrupt:
            pass
        
        return response 


def main(args=None):
    rclpy.init(args=args)
    
    location_service = StartSnakeServiceLocation()

    rclpy.spin_once(location_service)


if __name__ == '__main__':
    main()