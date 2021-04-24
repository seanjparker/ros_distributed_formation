#!/usr/bin/env python

import argparse
import numpy as np
import os
import rospy
import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path


directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import params
except ImportError:
  raise ImportError('Unable to import params.py. Make sure this file is in "{}"'.format(directory))

from kalman import KalmanFilterDetector
from rvo_controller import RVOController
from ros_util import *

X = 0
Y = 1
YAW = 2

def main(args):
  robot_id = args.robot
  rospy.init_node('{}_navigation'.format(robot_id))

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher('/{}/cmd_vel'.format(robot_id), Twist, queue_size=5)
  path_publisher = rospy.Publisher('/path', Path, queue_size=1)

  # Define and init all the components for the systems to control bots
  goal = GoalPose()
  odom = Odom(robot_id=robot_id)
  laser = SimpleLaser(robot_id=robot_id)
  detector = KalmanFilterDetector(robot_id=robot_id) # or DBSCANDetector(robot_id)
  controller = RVOController()

  # Define constants that we will need in the main loop body
  current_time = 0
  last_det_time = 0
  last_pub_time = 0
  ctrl_time = 0
  last_ctrl_time = 0
  frame_id = 0
  current_path = []
  last_time = rospy.Time.now().to_sec()

  # Stop the robot from moving if it currently moving
  stop_msg = Twist()
  stop_msg.linear.x = 0.
  stop_msg.angular.z = 0.
  i = 0
  while i < 10 and not rospy.is_shutdown():
    publisher.publish(stop_msg)
    rate_limiter.sleep()
    i += 1

  # Main body of loop
  while not rospy.is_shutdown():
    current_time = rospy.Time.now().to_sec()

    # Make sure all measurements are ready.
     # Get map and current position through SLAM:
    # > roslaunch exercises slam.launch
    if not laser.ready:
      rate_limiter.sleep()
      continue
    
    # Use the defined detector to find the goal pose using the pre-defined constraints
    time_since = current_time - last_det_time
    if time_since > 0.2:
      detector.detect(laser.coordinates)
      last_det_time = current_time

    # Get the goal position from the detector, we will use this for our instantaneous path planning
    goal_position = detector.track_to
    #goal_position = goal.position

    # We limit the rate at which we can send updates to the robots control params
    time_since = current_time - last_pub_time
    if time_since > 0.2:
      # Enough time has passed, so update the last_control times
      last_ctrl_time = ctrl_time
      ctrl_time = rospy.Time.now().to_sec()
      dt = ctrl_time - last_ctrl_time

      u = 0.0
      w = 0.0

      # Using the previously detected obstacles, the goal pose and the time delta, we can calculate the control parameters required
      # to control the robot to both maintain the formation and move the robot towards the goal
      if detector.ready:
        # obstacle list, _, goal, dt
        u, w = controller.get_velocity(detector.obstacles, np.array([0,0,0], dtype=np.float32), goal_position, dt)
      
      # Few checks to make sure that we don't move too fast and ignores noise in some of the readings
      if robot_id == 'tb3_0' and u <= 1e-2:
        u = params.ROBOT_LEADER_DEFAULT_SPEED
      if w < 1e-5:
        w = 0.0
      print('u: {}, w: {}'.format(u, w))

      # Send the control update message to the robot
      vel_msg = Twist()
      vel_msg.linear.x = u
      vel_msg.angular.z = w
      publisher.publish(vel_msg)
      last_pub_time = current_time

      # For debug and results gathering in the report, we calculate the seperation distance of the robot to the goal
      def dist(start, end):
        return np.sqrt( (start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2 )

      # Logging the seperation distance to a file
      sep_dist = dist(odom.pose, goal_position)
      sep_dist_path = './logs/{}_sep_dist.txt'.format(robot_id)
      with open(sep_dist_path, 'a+') as fp:
        fp.write('\n' + str(ctrl_time) + ' ' + str(sep_dist))

    # Publish path to RViz.
    path_msg = Path()
    path_msg.header.seq = frame_id
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'
    for u in current_path:
      pose_msg = PoseStamped()
      pose_msg.header.seq = frame_id
      pose_msg.header.stamp = path_msg.header.stamp
      pose_msg.header.frame_id = 'map'
      pose_msg.pose.position.x = u[X]
      pose_msg.pose.position.y = u[Y]
      path_msg.poses.append(pose_msg)
    path_publisher.publish(path_msg)

    rate_limiter.sleep()
    frame_id += 1


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--robot', default='tb3_0')
  parser.add_argument('--detector')
  parser.add_argument('--controller')
  args, _ = parser.parse_known_args()
  main(args)
