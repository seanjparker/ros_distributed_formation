from sklearn.cluster import DBSCAN

def dist(start, end):
  return np.sqrt( np.pow(start[X] - n[X], 2) + np.pow(start[Y] - end[Y], 2) )

class DBSCANDetector():
  def __init__(self, robot_id) : 
    self._robot_id = robot_id
    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._prev_pose = np.array([np.nan, np.nan], dtype=np.float32)
    self._dbscan = DBSCAN(eps=0.075, min_samples=3)
    np.seterr(all='warn')

  def find_goal(self, coordinates):
    potential_followable = []
    self._prev_pose = self._pose

    valid_coordinates = []
    for coord in coordinates:
        valid_coordinates.append(0 if coord[0] is np.nan or coord[1] is np.nan else coord)
    valid_coordinates = np.array(valid_coordinates)
    coordinates = valid_coordinates[~np.all(valid_coordinates==0, axis=1)]


    self._dbscan.fit(coordinates)
    unique_labels = set(self._dbscan.labels_)
    for label in unique_labels:
      if label == -1:
        continue

      labels_mask = self._dbscan.labels_ == label

      clusters = coordinates[labels_mask]

      if len(clusters) < 3 or dist(clusters[0], clusters[-1]) > 2 *  params.LEG_RADIUS_MAX: 
        continue 

      cluster_center = np.average(clusters, axis=0)
      distance = dist(center)

      if distance < 1e-2 or distance > params.MAX_DISTANCE_TO_TARGET:
        continue

      potential_followable.append([cluster_center, distance])

    self._pose = np.array([np.nan, np.nan], dtype=np.float32)
    if potential_followable is None: 
      return

    first_time = np.isnan(self._prev_pose[0])
    minimum = params.MAX_DISTANCE_TO_TARGET

    for item in potential_followable:
      if minimum > item[1]:
        if dist(item[0], self._prev_pose) < params.MAX_TARGET_DISPLACEMENT or first_time: 
          self._pose = item[0]
          minimum = item[1]
  
  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def goal_pose(self):
    return self._pose
