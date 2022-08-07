import rclpy

from snake_tf2.start_snake_service_location import StartSnakeServiceLocation
from snake_tf2.snake_tf2_listener import FrameListener

def main():
    rclpy.init()
    start_node = StartSnakeServiceLocation(True)
    try:
        rclpy.spin_once(start_node)
    except KeyboardInterrupt:
        pass
    
    # Not necessary, listener node is spinning inside
    # node = FrameListener()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass

    rclpy.shutdown()