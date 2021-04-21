#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import params
except ImportError:
  raise ImportError('Unable to import params.py. Make sure this file is in "{}"'.format(directory))

from dbscan import DBSCANDetector
from kalman import KalmanFilterDetector
from rvo_controller import RVOController

SPEED = .2
EPSILON = .1

X = 0
Y = 1
YAW = 2

class GoalPose(object):
  def __init__(self):
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
    self._position = np.array([np.nan, np.nan], dtype=np.float32)

  def callback(self, msg):
    # The pose from RViz is with respect to the "map".
    self._position[X] = msg.pose.position.x
    self._position[Y] = msg.pose.position.y
    print('Received new goal position:', self._position)

  @property
  def ready(self):
    return not np.isnan(self._position[0])

  @property
  def position(self):
    return self._position

class SimpleLaser(object):
  def __init__(self, robot_id=params.R0):
    self._name_space = robot_id
    rospy.Subscriber(robot_id + '/scan', LaserScan, self.callback)
    self._angles_degrees = np.array(range(0, params.ROBOT_FOV + 1, 2) + range(360 - params.ROBOT_FOV, 359, 2))
    self._angles = np.pi / 180. * self._angles_degrees
    self._width = np.pi / 180.
    self._measurements = [float('inf')] * len(self._angles)
    self._coordinates = [(float('inf'), float('inf'))] * len(self._angles)
    self._indices = None

  def callback(self, msg):
    # Helper for angles.
    def _within(x, a, b):
      pi2 = np.pi * 2.
      x %= pi2
      a %= pi2
      b %= pi2
      if a < b:
        return a <= x and x <= b
      return a <= x or x <= b

    # Compute indices the first time.
    if self._indices is None:
      self._indices = [[] for _ in range(len(self._angles))]
      for i, d in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        for j, center_angle in enumerate(self._angles):
          if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
            self._indices[j].append(i)

    ranges = np.nan_to_num(np.array(msg.ranges))
    for i, idx in enumerate(self._indices):
      # We do not take the minimum range of the cone but the 10-th percentile for robustness.
      self._measurements[i] = np.percentile(ranges[idx], 10)
      if self._measurements[i] > 3.5:
        self._measurements[i] = 3.5
      self._coordinates[i] = (self._measurements[i]*np.cos(self._angles[i]), self._measurements[i]*np.sin(self._angles[i]))

  @property
  def ready(self):
    return (not np.isnan(self._measurements[0]) and not np.isnan(self._coordinates[0][0]))

  @property
  def measurements(self):
    return self._measurements

  @property
  def coordinates(self):
    return self._coordinates


current_time = 0
previous_detection_time = 0
previous_publish_time = 0

current_control_time = 0
previous_control_time = 0

def main(args):
  global current_time
  global previous_detection_time
  global previous_publish_time

  global current_control_time
  global previous_control_time
  robot_id = args.robot

  rospy.init_node("{}_navigation".format(robot_id))

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  path_publisher = rospy.Publisher('/path', Path, queue_size=1)
  goal = GoalPose()
  frame_id = 0
  current_path = []
  previous_time = rospy.Time.now().to_sec()
  laser = SimpleLaser(robot_id=robot_id)

  detector = KalmanFilterDetector(robot_id) # DBSCANDetector(robot_id)
  controller = RVOController()

  # Stop moving message.
  stop_msg = Twist()
  stop_msg.linear.x = 0.
  stop_msg.angular.z = 0.

  # Make sure the robot is stopped.
  i = 0
  while i < 10 and not rospy.is_shutdown():
    publisher.publish(stop_msg)
    rate_limiter.sleep()
    i += 1

  while not rospy.is_shutdown():
    current_time = rospy.Time.now().to_sec()

    # Make sure all measurements are ready.
     # Get map and current position through SLAM:
    # > roslaunch exercises slam.launch
    if not laser.ready:
      rate_limiter.sleep()
      continue

    time_since = current_time - previous_detection_time
    if time_since > 0.2:
      detector.find_goal(laser.coordinates)
      previous_detection_time = current_time

    if not detector.ready:
      rate_limiter.sleep()
      continue

    # if not goal.ready:
    #   rate_limiter.sleep()
    #   continue
    
    goal_position = detector.goal_pose
    #goal_position = goal.position
    print("goal: {}".format(goal_position))

    time_since = current_time - previous_publish_time
    if time_since > 0.2:
      previous_control_time = current_control_time
      current_control_time = rospy.Time.now().to_sec()
      dt = current_control_time - previous_control_time
      # obstacle list, pose, goal, dt
      u, w = controller.get_velocity(detector.obstacles, np.array([0,0,0], dtype=np.float32), goal_position, dt)
      print(u, w)
      vel_msg = Twist()
      vel_msg.linear.x = u
      vel_msg.angular.z = w
      publisher.publish(vel_msg)
      previous_publish_time = current_time


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
  parser.add_argument('--robot', default='t_0')
  parser.add_argument('--detector')
  parser.add_argument('--controller')
  args, _ = parser.parse_known_args()
  main(args)
