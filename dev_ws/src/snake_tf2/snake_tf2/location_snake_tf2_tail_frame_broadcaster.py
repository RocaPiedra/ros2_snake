import rclpy

from snake_tf2.start_snake_service_location import StartSnakeServiceLocation
from snake_tf2.snake_tf2_tail_frame_broadcaster import FixedFrameBroadcaster

def main():
    rclpy.init()
    start_node = StartSnakeServiceLocation()
    try:
        rclpy.spin_once(start_node)
    except KeyboardInterrupt:
        pass
    
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
