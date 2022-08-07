import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from turtlesim.srv import Spawn

import random, time

class FrameListener(Node):

    def __init__(self,
                start_x=None, start_y=None, start_theta=None):
        super().__init__('snake_tf2_frame_listener')
        # For the background color change
        self.color_client = self.create_client(SetParameters, '/sim/set_parameters')
        while not self.color_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = None
        
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
        self.start_position = [start_x, start_y, start_theta]
        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, f'{self.body_name}/cmd_vel', 1)

        # Call on_timer function every second
        # Maybe increase the rate
        self.timer = self.create_timer(0.01, self.on_timer)
        
        # Variable controlling if the snake has eaten the body
        self.body_eaten = False # Must be set to False
    
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
                # Diference between both Theta values
                # theta_diff = abs(1 - abs(trans.transform.rotation.w))
                ang_diff = math.atan2(
                    trans.transform.translation.y,
                    trans.transform.translation.x)
                
                dist = math.sqrt(
                    abs(trans.transform.translation.x) ** 2 +
                    abs(trans.transform.translation.y) ** 2)
                
                if ang_diff*180/math.pi >= 15 and dist > 0.1:
                    
                    scale_rotation_rate = 6
                    msg.angular.z = scale_rotation_rate * ang_diff
                
                    scale_forward_speed = 1.5
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
                        f'Angle difference:{ang_diff*180/math.pi}')
                    
                elif ang_diff*180/math.pi < 15 and dist > 0.1:
                    
                    scale_rotation_rate = 3
                    msg.angular.z = scale_rotation_rate * ang_diff
                
                    scale_forward_speed = 3
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
                        f'Angle difference:{ang_diff*180/math.pi}')
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
                        f'Angle difference:{ang_diff*180/math.pi}')
                
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
                        self.change_background_color()
                        
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
                request.x = random.uniform(1, 19)
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
                request.y = random.uniform(1, 19)
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
                request.theta = random.uniform(-2*3.14, 2*3.14)
                self.get_logger().info(
                    f'Random starting Angle: {request.theta*180/math.pi}'
                )
            
            # Call request and spawns turtle in result
            result = self.spawner.call_async(request)
            
            print(dir(result)) # debug
            
            self.turtle_spawning_service_ready = True
            return result
        else:
            # Check if the service is ready
            self.get_logger().info('Body spawner is not ready')
            return None
        
    def change_background_color(self, r=None, g=None, b=None):
        self.req = SetParameters.Request()

        param = Parameter()
        param.name = "background_r"
        param.value.type = ParameterType.PARAMETER_INTEGER
        if r:
            param.value.integer_value = r
        else:
            param.value.integer_value = random.randint(0,255)
        self.req.parameters.append(param)

        param = Parameter()
        param.name = "background_g"
        param.value.type = ParameterType.PARAMETER_INTEGER
        if g:
            param.value.integer_value = g
        else:
            param.value.integer_value = random.randint(0,255)
        self.req.parameters.append(param)
        
        param = Parameter()
        param.name = "background_b"
        param.value.type = ParameterType.PARAMETER_INTEGER
        if b:
            param.value.integer_value = b
        else:
            param.value.integer_value = random.randint(0,255)
        self.req.parameters.append(param)

        self.future = self.color_client.call_async(self.req)
        self.get_logger().info('client request sent')
        
        
def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()