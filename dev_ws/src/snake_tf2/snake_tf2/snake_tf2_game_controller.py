import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import tf_transformations
from geometry_msgs.msg import TransformStamped

import random
import time

# Node type for the body (one per body)
# Handles body location, transforms and state
# Broadcasts the transforms
class SnakeBody(Node):
    
    def __init__(self, body_object, id):
        # Dynamic node name constructor to avoid spawning nodes with the same name
        super().__init__('node_body'+ str(id))
        self.object = body_object
        self.id = 'body' + str(id)
        self.position = 0
        self.eaten = False
        # Create velocity publisher for this body
        self.publisher = self.create_publisher(Twist, f'{self.id}/cmd_vel', 1)
        self.body_size = 0.8
        self.msg = Twist()
        self.parent = 'turtle1'
        # Sub to your pose to obtain the transform later
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.id}/pose',
            self.handle_turtle_pose,
            1)
        self.broadcaster = TransformBroadcaster(self)
        
    def body_eaten(self, position, parent = 'turtle1'):
        self.position = position
        self.eaten = True
        self.parent = parent
    
    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.id

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # self.get_logger().info(
        #     f'Sending Transform \n{t}'
        # )
        # Send the transformation
        self.broadcaster.sendTransform(t)
        
    def publish_speed(self, msg=None):
        if msg is None:
            msg = self.msg
        self.publisher.publish(msg)
        
    def check_linear_distance(self, trans):
        # -self.position*self.body_size to take into account
        # position in the snake
        return math.sqrt(
            abs(trans.transform.translation.x
            -self.body_size) ** 2 +
            abs(trans.transform.translation.y) ** 2)
        
    def check_angle_to_tail(self, trans):
        return math.atan2(
            trans.transform.translation.y,
            trans.transform.translation.x
            -self.body_size)
        
    def check_distance(self, trans):
        theta_diff = abs(1 - abs(trans.transform.rotation.w))
        # This is the angle to the center of the body
        ang = math.atan2(math.sin(trans.transform.rotation.w),
                        math.cos(trans.transform.rotation.w),)
            
        dist = self.check_linear_distance(trans)
        ang_diff = self.check_angle_to_tail(trans)
        
        return dist, ang_diff, theta_diff, ang
    
    def calc_speed(self, dist, ang_diff, theta_diff, ang):
        
        if ang_diff*180/math.pi >= 15 and dist > 0.1:
            
            scale_rotation_rate = 3
            self.msg.angular.z = scale_rotation_rate * ang_diff
        
            scale_forward_speed = 1
            self.msg.linear.x = scale_forward_speed * dist

            self.get_logger().info(
                f'Chasing target: ')
            self.get_logger().info(
                f'Linear Speed: {self.msg.linear.x}')
            self.get_logger().info(
                f'Angular Speed: {self.msg.angular.z}')
            self.get_logger().info(
                f'distance: {dist}')
            self.get_logger().info(
                f'Angle difference:{theta_diff*180/math.pi}')
            
        elif ang_diff*180/math.pi < 15 and dist > 1:
            
            scale_rotation_rate = 1
            self.msg.angular.z = scale_rotation_rate * ang_diff
        
            scale_forward_speed = 2
            self.msg.linear.x = scale_forward_speed * dist

            self.get_logger().info(
                f'Full Speed: ')
            self.get_logger().info(
                f'Linear Speed: {self.msg.linear.x}')
            self.get_logger().info(
                f'Angular Speed: {self.msg.angular.z}')
            self.get_logger().info(
                f'distance: {dist}')
            self.get_logger().info(
                f'Angle difference:{theta_diff*180/math.pi}')
        else:
            self.get_logger().info(
                f'STOP target is close: ')
            self.get_logger().info(
                f'Linear Speed: {self.msg.linear.x}')
            self.get_logger().info(
                f'Angular Speed: {self.msg.angular.z}')
            self.get_logger().info(
                f'distance: {dist}')
            self.get_logger().info(
                f'Angle difference:{theta_diff*180/math.pi}')
            
        return self.msg

    def calc_speed_2(self, dist, ang_diff, theta_diff, ang):
        if(dist < 0.1):
            self.msg.linear.x = 0 
            self.msg.angular.z = 0
            self.get_logger().info(
                f'Chasing target: Linear Speed: {self.msg.linear.x} || \
                Angular Speed: {self.msg.angular.z}\n \
                distance: {dist} || Angle difference: {ang*(2*math.pi)}')
        else:
            self.msg.linear.x = 2.5 * dist                
            self.msg.angular.z = 7 * ang
            self.get_logger().info(
                f'Chasing target: Linear Speed: {self.msg.linear.x} || \
                Angular Speed: {self.msg.angular.z}\n \
                distance: {dist} || Angle difference: {ang*(2*math.pi)}')
        
        return self.msg

    def return_parent(self):
        return self.parent
        

# Acts as game controller and transform listener
class SnakeGameController(Node):
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
            self.declare_parameter('snakemaster', 'turtle1')
            
            # The parameter received should be the adequate tail (one tail per body)
            self.target_frame = self.get_parameter(
                'snakemaster').get_parameter_value().string_value
        except:
            self.get_logger().info(
                'Node was not launched from launch file'
            )
            # Unless something else is specified, always chase turtle1
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
        self.body_num = 0
        self.num_eaten = 0
        self.spawn_list = []
        self.body_list = []
        self.start_position = [start_x, start_y, start_theta]
        self.body_max = 5

        # Call on_timer function every second
        # Maybe increase the rate
        self.timer = self.create_timer(0.1, self.on_timer)
        self.time_to_spawn = 15 # Seconds
        self.spawn_timer = time.time()
        
    def on_timer(self):
        # Check if there is any body in the sim
        if self.body_list:
            # Calculate speed for the bodies to follow their parent
            for index, body in enumerate(self.body_list):
                rclpy.spin_once(body)
                # Check Transform
                try:
                    trans = self.check_transform(body.id, body.parent)
                    if trans:
                        dist, ang_diff, theta_diff, ang = body.check_distance(trans)
                        
                        if body.eaten is True:
                            # Make the body chase its position
                            msg = body.calc_speed(dist, ang_diff, theta_diff, ang)
                            body.publish_speed(msg)
                            
                except TransformException as ex:
                    self.get_logger().info(
                        f'EATEN: Could not transform {body.id} to {body.return_parent()}: {ex}')
                    return
                except (LookupException, ConnectivityException, ExtrapolationException):
                    self.get_logger().info('Transform not ready')
                    return
        
        # Check if contact with the snake (spawned have turtle1 as parent)
        if self.spawn_list:
            for index, spawned in enumerate(self.spawn_list):
                
                rclpy.spin_once(spawned)
                trans = self.check_transform(spawned.id, spawned.parent)
                if trans:
                    dist,_,_,_ = spawned.check_distance(trans)
                    self.get_logger().info(
                        f'{spawned.id}: dist = {dist} m'
                    )
                    if dist < 1:
                        if self.body_list:
                            spawned.body_eaten(self, len(self.body_list)+1, parent = self.body_list[-1].id)
                            self.get_logger().info(
                                f'{len(self.body_list)+1} eaten: {spawned.id}, {spawned.parent} is the parent'
                            )
                        else:
                            # First body eaten
                            spawned.body_eaten(self, 1, parent = 'turtle1')
                            self.get_logger().info(
                                f'1 eaten: {spawned.id}'
                            )
                        # move the eaten body from spawn list to body list
                        self.body_list.append(spawned)
                        self.spawn_list.pop(index)
            
        else:
            # spawn first body or next one if there are no available
            self.body_spawner()
            # restart clock after spawn
            self.spawn_timer = time.time()
        
        # Spawn a body after a specific time
        time_elapsed = time.time() - self.spawn_timer    
        if time_elapsed > self.time_to_spawn:
            #Spawn a body and added to spawn list
            if self.body_num < self.body_max:
                self.body_spawner()
            else:
                self.get_logger().info(
                    '\nMAX SPAWNS REACHED'
                )
            self.spawn_timer = time.time()
                
    def check_transform(self, parent, child):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                child,
                parent,
                now)
            self.get_logger().info(
                f'Transformed from {child} to {parent}: \n{trans}')
            return trans
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {child} to {parent}: {ex}')
            return
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return None
        
    def body_spawner(self):
        if self.spawner.service_is_ready():
            # Initialize request with turtle name and coordinates
            # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
            
            self.body_num += 1
            request = Spawn.Request()
            request.name = 'body' + str(self.body_num)
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
            body_object = self.spawner.call_async(request)
            # Increase the number of bodies
            body = SnakeBody(body_object, self.body_num)
            self.spawn_list.append(body)
            self.turtle_spawning_service_ready = True
            
            return body
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
    node = SnakeGameController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()