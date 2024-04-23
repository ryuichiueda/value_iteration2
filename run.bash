#!/bin/bash

( cd ~/ros_ws/ && colcon build )

ros2 run value_iteration2 vi_node --ros-args --params-file $(dirname $0)/config/params.yaml
