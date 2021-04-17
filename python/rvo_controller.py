import rvo
import numpy as np

X = 0
Y = 1
YAW = 2

epsilon = 1e-3
epsilon_inv = 1.0 / epsilon

def clip_u(val)
  return params.SPEED if val > params.SPEED else val

def clip_w(val)
  if np.abs(w) > params.SPEED:
    return w / w * params.SPEED
  else:
    return val

def dist(start, end):
  return np.sqrt( np.pow(start[X] - n[X], 2) + np.pow(start[Y] - end[Y], 2) )

class RVOController:
  def __init__(self):
    robot_diam = 2 * params.ROBOT_RADIUS
    self._rvo_model = dict(robot_radius=robot_diam, circular_obstacles=[], boundary=[])
    self.last_speed = [0, 0]

  def get_velocity(self, obstacle_list, pose, goal):
    if obstacle_list is None:
      return 0, 0

    if dist(goal_position, [0, 0]) < params.ROBOT_MIN_DIST + 2 * params.ROBOT_RADIUS:
      return 0, 0

    X_current = []
    V_current = []
    goal_pos = []
    V_max = [params.ROBOT_SPEED / 2 for i in range(len(obstacle_list) + 1)]

    X_current.append([0,0])
    V_current.append(self.last_speed)
    goal_pos.append(goal_position)

    for obstacle in obstacle_list:
      X_current.append([obstacle[0], obstacle[1]])
      V_current.append([obstacle[2], obstacle[3]])
      goal_pos.append([0,0])

    # do rvo update
    V_des = rvo.compute_V_des(X_current, goal_pos, V_max)
    V_current = rvo.RVO_update(X_current, V_des, V_current, self._rvo_model)
    
    # do feedback linerisation
    velocity = V_current[0]
    yaw = np.arctan2(velocity[Y], velocity[X])
    u = velocity[X] * np.cos(yaw) + velocity[Y] * np.sin(yaw)
    w = (epsilon_inv) * (-1.0 * velocity[X] * np.sin(yaw) + velocity[Y] * np.cos(yaw))

    return clip_u(u), clip_w(w)
