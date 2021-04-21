#!/bin/bash

roslaunch mini_project gazebo.launch &
sleep 5
gnome-terminal -x sh -c "roslaunch mini_project multi_slam.launch; bash"
# sleep 5

gnome-terminal -x sh -c "python python/navigation.py --robot=tb3_0; bash"
gnome-terminal -x sh -c "python python/navigation.py --robot=tb3_1; bash"
gnome-terminal -x sh -c "python python/navigation.py --robot=tb3_2; bash"
# gnome-terminal -x sh -c "python python/navigation.py --robot=tb3_3; bash"
# python python/navigation.py --robot=r_1 &
# python python/navigation.py --robot=r_2 &
