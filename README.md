# value_iteration2: real-time value iteration planner package for ROS 2

a monolithic mobile robot navigation system as a replacement for the navigation stack

* ROS 1 version: https://github.com/ryuichiueda/value_iteration

## How to try

```bash
$ sudo apt install ros-humble-grid-map*
$ sudo apt install ... (I will write later about required packages. )
$ cd <workspace>
$ git clone https://github.com/ryuichiueda/emcl2_ros2.git
$ git clone https://github.com/ryuichiueda/value_iteration2.git
$ cd <workspace>
$ colcon build
$ source install/setup.bash
$ source install/local_setup.bash
$ ros2 launch value_iteration2 turtle.launch.py
```

### how to use with RViz and Gazebo

[![](https://img.youtube.com/vi/qNjMH5Ao6QM/0.jpg)](https://www.youtube.com/watch?v=qNjMH5Ao6QM)

## topic

... writing ...

## Nodes

### vi_node

This node executes value iteration.

#### how to execute value iteration

Give the goal to topic `/goal_pose`. The planner immediately starts working and giving the velocity of the robot through `/cmd_vel`. The action server is not implemented in this stage.

#### Services Called

* static_map ([nav_msgs/srv/GetMap](https://docs.ros2.org/foxy/api/nav_msgs/srv/GetMap.html))
    * Initiate the map for value iteration.

#### Subscribed Topics

* goal_pose ([geometry_msgs/msg/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html))
    * goal position and orientation
* scan ([sensor_msgs/msg/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html))
    * laser scans
* tf ([tf2_msgs/msg/tfMessage](https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html))
    * transforms

#### Published Topics

* /cmd_vel ([geometry_msgs/msg/Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html))
    * the control order to the robot; published only when the parameter `online` is true
