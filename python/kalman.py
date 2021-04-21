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

def dist(start, end):
  return np.sqrt( (start[X] - end[X]) ** 2 + (start[Y] - end[Y]) ** 2 )

X=0
Y=1
YAW=2
class KalmanFilterDetector(object):

  def __init__(self, robot_id):
    self._namespace = robot_id
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._speed = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_speed = np.array([np.nan, np.nan], dtype=np.float32)
    self._obstacles     = []
    self._target_coords = []
    self._target_speeds = []
    self._detectFollowable = True
    rospy.Subscriber(robot_id + '/tracked_obstacles', Obstacles, self.callback)

  # filtering the objects
  def callback(self, msg):
    obstacles = []
    target_coords = []
    target_speeds = []

    for idx, circle in enumerate(msg.circles):

      obstacles.append(np.array([msg.circles[idx].center.x, 
                                msg.circles[idx].center.y, 
                                msg.circles[idx].velocity.x, 
                                msg.circles[idx].velocity.y, 
                                msg.circles[idx].radius, 
                                msg.circles[idx].true_radius
                                ], dtype=np.float64))

      # filter out obstacles which are too big
      if msg.circles[idx].true_radius < 4*params.TRACKED_OBJ_RAD:
        target_coords.append(np.array([msg.circles[idx].center.x, 
                                      msg.circles[idx].center.y
                                      ], dtype=np.float64))

        target_speeds.append(np.array([msg.circles[idx].velocity.x, 
                                      msg.circles[idx].velocity.y
                                      ], dtype=np.float64))
    self._obstacles = obstacles
    self._target_coords = target_coords
    self._target_speeds = target_speeds


  # based on the filtered obstacles, last tracked obstacle position, etc, find the goal 
  def find_goal(self, coordinates):
    print ("(Avg dec) -- Finding goal")

    pose = []
    speed = []

    if self._target_coords == None:
      print ("(Avg dec) -- NO TARGETS")
      return

    # first time we are trying to detect smth
    if self._detectFollowable:
      tolerance = params.VIEW_DIST

      min_distance = params.ROBOT_MAX_DIST
      for idx, target in enumerate(self._target_coords): 
        
        # check if in our field of view -> forward and between some [-x, x] coords
        # the coords of the object finder are reversed 
        if target[Y] < tolerance and target[Y] > -1.0*tolerance and target[X] > 0:
          # compute distance to target: 
          # print (target)
          distance = dist(target, [0, 0])

          # get the closest initial obstacle and consider it as 
          # the object that should be followed 
          if distance < min_distance:
            min_distance = distance
            pose = target
            speed = self._target_speeds[idx]
            self._detectFollowable = False

    else:
      tolerance = params.ROBOT_MAX_DIST*2

      potential_positions = []
      indexes             = []


      for idx, target in enumerate(self._target_coords): 
        # check if in our field of view -> forward and between some [-x, x] coords
        if target[Y] < tolerance and target[Y] > -1.0*tolerance and target[X] > 0:
          # compute distance to target
          distance = np.nan_to_num(dist(target, self._prev_pose))
          # if np.isnan(self._prev_pose[0])
          # print (target)
          # print (distance)

          # look at the objects within a certain radius of the previous pose
          if distance < params.ROBOT_MAX_GAP:
            potential_positions.append(target)
            indexes.append(idx)

      # simple
      if len(potential_positions) > 0:

        # we take the closest yet again 
        min_distance = params.ROBOT_MAX_DIST
        for idx, target in enumerate(potential_positions):

          distance = dist(target, [0, 0])
          if distance < min_distance:
            pose = target
            speed = self._target_speeds[indexes[idx]]
            min_distance = distance
      
      else: 
        # we have lost the target
        print ("(Avg dec) -- Target Lost")
        self._detectFollowable = True
        self._pose = np.array([0,0])

    if len(pose) > 0: 
      self._prev_pose = self._pose
      self._prev_speed = self._speed
      self._pose = pose
      self._speed = speed
      print ("(Avg dec) -- Target found")
      print (pose)

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def goal_pose(self):
    return self._pose

  @property
  def obstacles(self): 
    return self._obstacles