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

### how to use with RViz

[![](https://img.youtube.com/vi/YWBFev-naIo/0.jpg)](https://www.youtube.com/watch?v=YWBFev-naIo)

### with obstacles given in Gazebo


[![](https://img.youtube.com/vi/qNjMH5Ao6QM/0.jpg)](https://www.youtube.com/watch?v=qNjMH5Ao6QM)

## topic

... writing ...
