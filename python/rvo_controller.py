import rvo
import numpy as np
import os
import sys

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import params
except ImportError:
  raise ImportError('Unable to import params.py. Make sure this file is in "{}"'.format(directory))

X = 0
Y = 1
YAW = 2

epsilon = 1e-3
epsilon_inv = 1.0 / epsilon

def clip_u(u):
  return params.ROBOT_SPEED if u > params.ROBOT_SPEED else u

def clip_w(w):
  if np.abs(w) > params.ROBOT_SPEED:
    return w / w * params.ROBOT_SPEED
  else:
    return w

def dist(start, end):
  return np.sqrt( (start[X] - end[X])** 2 + (start[Y] - end[Y])**2 )

class RVOController:
  def __init__(self):
    robot_diam = 2 * params.ROBOT_RADIUS
    self._rvo_model = dict(robot_radius=robot_diam, circular_obstacles=[], boundary=[])
    self.last_speed = [0, 0]

  def get_velocity(self, obstacle_list, pose, goal, dt):
    if dist(goal, [0, 0]) <= params.ROBOT_MIN_DIST or obstacle_list is None or dist(goal, [0, 0]) < params.ROBOT_MIN_DIST + 2 * params.ROBOT_RADIUS: 
      return 0, 0

    X_current = [[0, 0]]
    V_current = [self.last_speed]
    goal_pos = [goal]
    V_max = [params.ROBOT_SPEED / 2 for i in range(len(obstacle_list) + 1)]

    # define using the previously detected obstacles in the map
    for obstacle in obstacle_list:
      X_current.append([obstacle[0], obstacle[1]])
      V_current.append([obstacle[2], obstacle[3]])
      goal_pos.append([0, 0])

    # RVO updates
    V_des = rvo.compute_V_des(X_current, goal_pos, V_max)
    V_current = rvo.RVO_update(X_current, V_des, V_current, self._rvo_model)
    
    # compute control parameters from the RVO results
    velocity = V_current[0]
    yaw = np.arctan2(velocity[Y], velocity[X])
    u = velocity[X] * np.cos(yaw) + velocity[Y] * np.sin(yaw)
    w = (epsilon_inv) * (-1.0 * velocity[X] * np.sin(yaw) + velocity[Y] * np.cos(yaw))

    return clip_u(u), clip_w(w)
