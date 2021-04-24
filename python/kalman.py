import numpy as np
import scipy.signal
import rospy

from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle

import sys
import os
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import params
except ImportError:
  raise ImportError('Unable to import params.py. Make sure this file is in "{}"'.format(directory))

X = 0
Y = 1
YAW = 2

def dist(start, end):
  return np.sqrt( (start[X] - end[X]) ** 2 + (start[Y] - end[Y]) ** 2 )

class KalmanFilterDetector(object):
  def __init__(self, robot_id):
    self._namespace = robot_id
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._vel = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_vel = np.array([np.nan, np.nan], dtype=np.float32)
    self._obstacles = []
    self._tracking_pose = []
    self._tracking_vel = []
    self.has_detected = True
    rospy.Subscriber('/{}/tracked_obstacles'.format(robot_id), Obstacles, self.ros_callback)

  def ros_callback(self, msg):
    obstacles = []
    target_poses = []
    target_vels = []

    for i, _ in enumerate(msg.circles):
      obstacles.append(np.array([msg.circles[i].center.x, 
        msg.circles[i].center.y, 
        msg.circles[i].velocity.x, 
        msg.circles[i].velocity.y, 
        msg.circles[i].radius, 
        msg.circles[i].true_radius
      ], dtype=np.float64))

      if msg.circles[i].true_radius < 4 * params.TRACKED_OBJ_RAD:
        target_poses.append(
          np.array([msg.circles[i].center.x, msg.circles[i].center.y], dtype=np.float64)
        )

        target_vels.append(
          np.array([msg.circles[i].velocity.x, msg.circles[i].velocity.y], dtype=np.float64)
        )
    self._obstacles = obstacles
    self._tracking_pose = target_poses
    self._tracking_vel = target_vels


  def detect(self, coordinates):
    pose = []
    vel = []

    if self._tracking_pose == None:
      return

    if self.has_detected:
      robot_view = params.VIEW_DIST
      min_distance = params.ROBOT_MAX_DIST

      # Find the obstacle that has the minimum distance to our robot
      # The results from the obstacle_detector package are reversed which means we have to go backwards
      # through the search, this only happens for the first obstacle, in the following detetions we look
      # for the clostest obstacle from the last position
      for i, target in enumerate(self._tracking_pose):   
        if target[Y] < robot_view and target[Y] > -1.0 * robot_view and target[X] > 0:
          if dist(target, [0, 0]) < min_distance:
            self.has_detected = False
            speed = self._tracking_vel[i]
            min_distance = dist(target, [0, 0])
            pose = target

    else:
      robot_view = params.ROBOT_MAX_DIST * 2

      candidate_poses = []
      candidate_is = []

      for i, target in enumerate(self._tracking_pose): 
        if target[Y] < robot_view and target[Y] > -1.0*robot_view and target[X] > 0:
          # Calculate the distance between the candidate and previous pose
          # Need to take into account that the distance can be invalid
          distance = np.nan_to_num(dist(target, self._prev_pose))

          # Wwe only need to consider the candidates that are close enough (predefined by the constants)
          if distance < params.ROBOT_MAX_GAP:
            candidate_poses.append(target)
            candidate_is.append(i)

      if len(candidate_poses) > 0:
        # Get the candidate pose with the minimum distance compared to the previous pose
        min_distance = params.ROBOT_MAX_DIST
        for i, target in enumerate(candidate_poses):
          if dist(target, [0, 0]) < min_distance:
            vel = self._tracking_vel[candidate_is[i]]
            pose = target
            min_distance = dist(target, [0, 0])
      else: 
        self.has_detected = True
        self._pose = np.array([0, 0])

    if len(pose) > 0:
      self._prev_pose = self._pose
      self._prev_vel = self._vel
      self._pose = pose
      self._vel = vel

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def track_to(self):
    return self._pose

  @property
  def obstacles(self): 
    return self._obstacles