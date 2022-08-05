import rclpy

from snake_tf2.start_snake_service import StartSnakeService
from snake_tf2.snake_tf2_listener import FrameListener

def main():
    rclpy.init()
    start_node = StartSnakeService()
    try:
        rclpy.spin_once(start_node)
    except KeyboardInterrupt:
        pass
    
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()