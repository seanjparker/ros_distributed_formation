import os
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.insert(0, directory)
try:
  import params
except ImportError:
  raise ImportError('Unable to import params.py. Make sure this file is in "{}"'.format(directory))

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

class Odom(object):
  def __init__(self, robot_id=params.R0):
    self._name_space = robot_id
    self._pose = [0.0, 0.0, 0.0]
    odom_sub = rospy.Subscriber('/{}/odom'.format(robot_id), Odometry, self.callback)

  def callback(self, msg):
    self._pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

  @property
  def pose(self):
    return self._pose
