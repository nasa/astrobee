'''This script takes a bag of static data (/tf) and computes their standard deviation for position and orientation.
It receiveces as argument the bag to be analyze. Handles multiple trackers'''

# Import libraries
import matplotlib.pyplot as pyplot
import numpy as np
import rosbag
import math
import sys

# Import from libraries
from mpl_toolkits.mplot3d import Axes3D

# Check arguemnts
if (sys.argv.__len__() < 2):
  print("Usage: python getCovariance.py <data.bag>")
  sys.exit(-1)
bag = rosbag.Bag(sys.argv[1])

# Data structure
points = dict()
angles = list()
master_quaternion = np.matrix((4,1))

# Iterate over all bag
for topic, msg, t in bag.read_messages(topics=['/tf']):
  if not points.has_key(msg.transforms[0].child_frame_id):
    points[msg.transforms[0].child_frame_id] = np.empty([3,0])
    master_quaternion = np.matrix([msg.transforms[0].transform.rotation.w,
      msg.transforms[0].transform.rotation.x,
      msg.transforms[0].transform.rotation.y,
      msg.transforms[0].transform.rotation.z]).transpose()
  # Temporary position
  tmP = np.matrix([msg.transforms[0].transform.translation.x,
    msg.transforms[0].transform.translation.y,
    msg.transforms[0].transform.translation.z]).transpose()
  # Temporary quaternion
  tmQ = np.matrix([msg.transforms[0].transform.rotation.w,
    msg.transforms[0].transform.rotation.x,
    msg.transforms[0].transform.rotation.y,
    msg.transforms[0].transform.rotation.z]).transpose()
  # Saving the position
  points[msg.transforms[0].child_frame_id] = np.hstack(
    (points[msg.transforms[0].child_frame_id],tmP))
  # convert quaternion to angle
  diff_quaternion = (master_quaternion[0] * tmQ[0],
    master_quaternion[1] * -tmQ[1],
    master_quaternion[2] * -tmQ[2],
    master_quaternion[3] * -tmQ[3])
  angles.append(2 * math.atan2( math.sqrt(diff_quaternion[1]**2 +
    diff_quaternion[2]**2 +
    diff_quaternion[3]**2),
    diff_quaternion[0]**2 ))
bag.close()

# Compute standard deviation of position and orientation
trackers = points.items()
for tracker, _ in  trackers:
  var = np.cov(points[tracker])
  w,v = np.linalg.eig(var)
  print("STD translation " + tracker + ": " + str(max(np.sqrt(w))))
  print("STD orientation " + tracker + ": " + str(np.std(angles)))



# Plot
fig = pyplot.figure()
ax = fig.gca(projection='3d')
for tracker, _ in trackers:
  ax.scatter(np.array(points[tracker][0,:])[0],
    np.array(points[tracker][1,:])[0],
    np.array(points[tracker][2,:])[0],
    label = tracker)
ax.view_init(0, 0)
ax.set_xlabel('X [n]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
pyplot.show()
