from sklearn.cluster import DBSCAN
import os
import sys
import numpy as np

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

class DBSCANDetector():
  def __init__(self, robot_id) : 
    self._robot_id = robot_id
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._dbscan = DBSCAN(eps=0.075, min_samples=3)
    np.seterr(all='warn')

  def find_goal(self, coordinates):
    followable = []
    self._prev_pose = self._pose

    coordinates = np.nan_to_num(coordinates)
    coordinates = coordinates[~np.all(coordinates==0, axis=1)]


    self._dbscan.fit(coordinates)
    unique_labels = set(self._dbscan.labels_)
    for label in unique_labels:
      if label == -1:
        continue

      labels_mask = self._dbscan.labels_ == label

      clusters = coordinates[labels_mask]

      if len(clusters) < 3 or dist(clusters[0], clusters[-1]) > 2 * 0.05: 
        continue 

      cluster_center = np.average(clusters, axis=0)
      distance = dist(cluster_center, [0, 0])

      if distance < 1e-2 or distance > params.ROBOT_MAX_DIST:
        continue

      followable.append([cluster_center, distance])

    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    if followable is None: 
      return

    first_time = np.isnan(self._prev_pose[0])
    boundary = params.ROBOT_MAX_DIST

    for item in followable:
      if boundary > item[1]:
        if dist(item[0], self._prev_pose) < params.ROBOT_MAX_DIST / 2 or first_time: 
          self._pose = item[0]
          boundary = item[1]
  
  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def goal_pose(self):
    return self._pose

  @property
  def obstacles(self): 
    return None