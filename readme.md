**Environment**: Ros 2 Foxy, Python 3, Ubuntu 20.04
Use the prepared docker environment for fast deployment and testing, instructions available in *launch_docker_env.md*

## Packages:

### learning_tf2_py: 
Finished tf2 tutorial

### snake_tf2: 
Package with base programming practice: 

#### Functionality number 1: Full application launch + turtle follow upon contact using:

- [x] Center body follow:

```ros2 launch snake_tf2 snake_base.launch.py ```

 [![VIDEO](https://img.youtube.com/vi/n85vTV7h9Pc/default.jpg)](https://www.youtube.com/watch?v=n85vTV7h9Pc&ab_channel=PabloRoca)
 
- [x] Tail follow (follower stays slightly behind):

```ros2 launch snake_tf2 snake_fixed_frame.launch.py ```

 [![VIDEO](https://img.youtube.com/vi/RgRDOdGCIdE/default.jpg)](https://www.youtube.com/watch?v=RgRDOdGCIdE&ab_channel=PabloRoca)
  
#### Functionality number 2: Wait for service launch + turtle follow upon contact using:

##### Empty service

- [x] Center body follow for Empty service call:

```ros2 launch snake_tf2 snake_wait_call.launch.py ```

 [![VIDEO](https://img.youtube.com/vi/260QAY8FyUY/default.jpg)](https://www.youtube.com/watch?v=260QAY8FyUY&ab_channel=PabloRoca)

- [x] Tail follow for Empty service call(follower stays slightly behind):

```ros2 launch snake_tf2 snake_fixed_frame_wait_call.launch.py ``` 

 [![VIDEO](https://img.youtube.com/vi/HHhe4kjLpOM/default.jpg)](https://www.youtube.com/watch?v=HHhe4kjLpOM&ab_channel=PabloRoca)

Launch one of those launch.py and then call a ros2 service with the name **/start_turtlesim_snake** like this to spawn turtle in random point:

```ros2 service call /start_turtlesim_snake std_srvs/Empty ```

##### Initial location service

- [x] Center body follow for service call with input location:

```ros2 launch snake_tf2 snake_wait_initial_pose.launch.py ``` 

 [![VIDEO](https://img.youtube.com/vi/5TC7EmX0PSo/default.jpg)](https://www.youtube.com/watch?v=5TC7EmX0PSo&ab_channel=PabloRoca)
- [x] Tail follow for service call with input location(follower stays slightly behind):

```ros2 launch snake_tf2 snake_fixed_frame_wait_initial_pose.launch.py ``` 

 [![VIDEO](https://img.youtube.com/vi/DrX9gxzqmhU/default.jpg)](https://www.youtube.com/watch?v=DrX9gxzqmhU&ab_channel=PabloRoca)

Launch one of those launch.py and then call a ros2 service with the name **/start_turtlesim_snake** like this to spawn turtle in specific place:

```ros2 service call /start_turtlesim_snake snake_interfaces/srv/TurtlePos "{pos_x: 1, pos_y: 1, angle: 1}"```

##### Snake Game

##### Other Functionalities
- Change the simulation background color when the turtle is eaten. Good example on how to set parameters from other packages using their own services with a minimal client that belongs to the listener class.
  
### snake_interfaces: 
C++ package with the additional msg and srv types used in the snake package.

