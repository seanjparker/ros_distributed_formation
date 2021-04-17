#!/bin/bash

roslaunch turtlebot3_formation main.launch &
sleep 5
roslaunch turtlebot3_formation multirobot_slam.launch &
sleep 5

python python/navigation.py --robot=r_0 &
python python/navigation.py --robot=r_1 &
python python/navigation.py --robot=r_2 &