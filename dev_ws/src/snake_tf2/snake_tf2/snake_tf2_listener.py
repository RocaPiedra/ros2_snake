import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from turtlesim.srv import Spawn

import random

class FrameListener(Node):

    def __init__(self,
                start_x=None, start_y=None, start_theta=None):
        super().__init__('snake_tf2_frame_listener')

        try:
            # Declare and acquire `target_frame` parameter
            self.declare_parameter('target_frame', 'turtle1')
            
            # The parameter received should be the adequate tail (one tail per body)
            self.target_frame = self.get_parameter(
                'target_frame').get_parameter_value().string_value
        except:
            self.get_logger().info(
                'Listener was not launched from launch file'
            )
            self.target_frame = 'turtle1'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.body_spawned = False
        self.body_name = 'body1'
        self.body_list = []
        self.body_list.append(self.body_name)
        self.turtle_object_list = []
        self.start_position = [start_x, start_y, start_theta]
        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, f'{self.body_name}/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        
        # Variable controlling if the snake has eaten the body
        self.body_eaten = True # Must be set to False
    
    def new_body(self):
        new_num_body = len(self.body_list)
    
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = self.body_name

        if self.turtle_spawning_service_ready:
            if self.body_spawned and self.body_eaten:
                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
                except TransformException as ex:
                    self.get_logger().info(
                        f'EATEN: Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return
                except (LookupException, ConnectivityException, ExtrapolationException):
                    self.get_logger().info('EATEN: transform not ready')
                    return

                msg = Twist()
                theta_diff = abs(1 - abs(trans.transform.rotation.w))
                ang_diff = math.atan2(
                    trans.transform.translation.y,
                    trans.transform.translation.x)
                ang = math.atan2(math.sin(trans.transform.rotation.w),
                                math.cos(trans.transform.rotation.w),)
                if(ang <= -3.14) or (ang > 3.14):
                    ang = ang / math.pi

                dist = math.sqrt(
                    abs(trans.transform.translation.x) ** 2 +
                    abs(trans.transform.translation.y) ** 2)
                
                if(ang <= -3.14) or (ang > 3.14):
                    ang = ang / (2*math.pi)
            
                # if(dist < 0.8):
                #     msg.linear.x = 0 
                #     msg.angular.z = 0
                #     self.get_logger().info(
                #         f'Chasing target: Linear Speed: {msg.linear.x} || \
                #         Angular Speed: {msg.angular.z}\n \
                #         distance: {dist} || Angle difference: {ang*(2*math.pi)}')
                # else:
                #     msg.linear.x = 2.5 * dist                
                #     msg.angular.z = 7 * ang
                #     self.get_logger().info(
                #         f'Chasing target: Linear Speed: {msg.linear.x} || \
                #         Angular Speed: {msg.angular.z}\n \
                #         distance: {dist} || Angle difference: {ang*(2*math.pi)}')
                
                if ang_diff*180/math.pi >= 15 and dist > 0.1:
                    
                    scale_rotation_rate = 3
                    msg.angular.z = scale_rotation_rate * ang_diff
                
                    scale_forward_speed = 1
                    msg.linear.x = scale_forward_speed * dist

                    self.get_logger().info(
                        f'Chasing target: ')
                    self.get_logger().info(
                        f'Linear Speed: {msg.linear.x}')
                    self.get_logger().info(
                        f'Angular Speed: {msg.angular.z}')
                    self.get_logger().info(
                        f'distance: {dist}')
                    self.get_logger().info(
                        f'Angle difference:{theta_diff*180/math.pi}')
                    
                elif ang_diff*180/math.pi < 15 and dist > 1:
                    
                    scale_rotation_rate = 1
                    msg.angular.z = scale_rotation_rate * ang_diff
                
                    scale_forward_speed = 2
                    msg.linear.x = scale_forward_speed * dist

                    self.get_logger().info(
                        f'Full Speed: ')
                    self.get_logger().info(
                        f'Linear Speed: {msg.linear.x}')
                    self.get_logger().info(
                        f'Angular Speed: {msg.angular.z}')
                    self.get_logger().info(
                        f'distance: {dist}')
                    self.get_logger().info(
                        f'Angle difference:{theta_diff*180/math.pi}')
                else:
                    self.get_logger().info(
                        f'STOP target is close: ')
                    self.get_logger().info(
                        f'Linear Speed: {msg.linear.x}')
                    self.get_logger().info(
                        f'Angular Speed: {msg.angular.z}')
                    self.get_logger().info(
                        f'distance: {dist}')
                    self.get_logger().info(
                        f'Angle difference:{theta_diff*180/math.pi}')
                
                self.publisher.publish(msg)
                
                
            elif not self.body_spawned:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.body_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
            
            elif not self.body_eaten:   
                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
                    
                    distance = math.sqrt(
                        abs(trans.transform.translation.x**2) + 
                        abs(trans.transform.translation.y**2))
                    # self.get_logger().info(
                    #     f'current distance to {self.body_name} is {distance} m')
                    
                    if distance < 1:
                        self.body_eaten = True
                        self.get_logger().info
                        (f'Piece {self.body_name} EATEN, ÑAM ÑAM')
                        
                except TransformException as ex:
                    self.get_logger().info(
                        f'NOT EATEN: Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return
                except (LookupException, ConnectivityException, ExtrapolationException):
                    self.get_logger().info('NOT EATEN: transform not ready')
                    return
        else:
            spawn_result = self.body_spawner()
            if spawn_result is not None:
                self.result = spawn_result

    def body_spawner(self):
        if self.spawner.service_is_ready():
            # Initialize request with turtle name and coordinates
            # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
            request = Spawn.Request()
            request.name = 'body1'
            if self.start_position[0]:
                request.x = self.start_position[0]
                self.start_position[0] = None # So this only happens for the first turtle
                self.get_logger().info(
                    f'Given starting point X: {request.x}'
                )
            else:
                request.x = random.uniform(0, 10)
                self.get_logger().info(
                    f'Random starting point X: {request.x}'
                )
                
            if self.start_position[1]:
                request.y = self.start_position[1]
                self.start_position[1] = None # So this only happens for the first turtle
                self.get_logger().info(
                    f'Given starting point Y: {request.y}'
                )
            else:
                request.y = random.uniform(0, 10)
                self.get_logger().info(
                    f'Random starting point Y: {request.y}'
                )
                
            if self.start_position[2]:
                request.theta = self.start_position[2]
                self.start_position[2] = None # So this only happens for the first turtle
                self.get_logger().info(
                    f'Given starting Angle: {request.theta*180/math.pi}'
                )
            else:
                request.x = random.uniform(-2*3.14, 2*3.14)
                self.get_logger().info(
                    f'Random starting Angle: {request.theta*180/math.pi}'
                )
            
            # Call request and spawns turtle in result
            result = self.spawner.call_async(request)
            
            print(dir(result)) # debug
            
            self.turtle_object_list.append(result)
            self.turtle_spawning_service_ready = True
            return result
        else:
            # Check if the service is ready
            self.get_logger().info('Body spawner is not ready')
            return None
        
            
class ChangeBackgroundColor(Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')
        
        
def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()