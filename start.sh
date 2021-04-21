#!/bin/bash

roslaunch mini_project gazebo.launch &
sleep 5
roslaunch mini_project multi_slam.launch &
# sleep 5

gnome-terminal -x sh -c "python python/navigation.py --robot=tb3_0; bash"
# python python/navigation.py --robot=r_1 &
# python python/navigation.py --robot=r_2 &
