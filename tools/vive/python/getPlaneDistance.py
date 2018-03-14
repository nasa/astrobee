'''This script takes a bag of dynamic (/tf) and computes their standard deviation for position and orientation.
It receiveces as argument the bag to be analyze. Handles multiple trackers'''

# Import libraries
import matplotlib.pyplot as pyplot
import numpy as np
import rosbag
import rospy
import math
import sys

# Import from libraries
from mpl_toolkits.mplot3d import Axes3D

class Transform:
  """This class handles 3D rigid body transforms
  It saves the translation and rotation and simplifies the notation when writing
  the script with it"""

  # Initializes the transform to default values
  def __init__(self):
    self.translation = np.matrix(np.zeros((3,1)))
    self.rotation = np.matrix(np.identity(3))
    self.parent = ""
    self.child = ""
    self.stamp = 0

  # Initializes the translation to the values we want
  def Translation(self,x,y,z):
    self.translation[0,0] = x
    self.translation[1,0] = y
    self.translation[2,0] = z

  #Initializes the orientation with a quaternion to the values we want
  def Rotation(self,w,x,y,z):
    self.rotation[0,0] = x**2 - y**2 - z**2 + w**2
    self.rotation[0,1] = 2*(x*y + z*w)
    self.rotation[0,2] = 2*(x*z - y*w)
    self.rotation[1,0] = 2*(x*y - z*w)
    self.rotation[1,1] = -x**2 + y**2 - z**2 + w**2
    self.rotation[1,2] = 2*(y*z + x*w)
    self.rotation[2,0] = 2*(x*z + y*w)
    self.rotation[2,1] = 2*(y*z + x*w)
    self.rotation[2,2] = -x**2 - y**2 + z**2 + w**2

  # Sets the parent of the frame
  def Parent(self, frame):
    self.parent = frame

  # Sets the child of the frame
  def Child(self, frame):
    self.child = frame

  # Multiplication of transforms
  def __mul__(self, other):
    product = Transform()
    product.translation = self.rotation * other.translation + self.translation
    product.rotation = self.rotation * other.rotation
    return product

  # Multiplication of transforms
  def __rmul__(self, other):
    product = Transform()
    product.translation = other.rotation * self.translation + other.translation
    product.rotation = other.rotation * self.rotation
    return product

  # Convertion to string of the translation only
  def __str__(self):
    return "([" + str(self.translation[0,0]) + "," + str(self.translation[1,0]) + "," + str(self.translation[2,0]) + "])\n"

# Check if the number of arguments is correct
if sys.argv.__len__() <= 1:
  print("Usage: python analysevive.py <plane.bag>")
  sys.exit(-1)

# Reading the bag of data
bag = rosbag.Bag(sys.argv[1])
poses = dict()
for topic, msg, t in bag.read_messages(topics=['/tf']):
  if not poses.has_key(msg.transforms[0].child_frame_id):
    poses[msg.transforms[0].child_frame_id] = list()
  pose = Transform()
  pose.Translation(msg.transforms[0].transform.translation.x,
    msg.transforms[0].transform.translation.y,
    msg.transforms[0].transform.translation.z)
  pose.Rotation(msg.transforms[0].transform.rotation.w,
    msg.transforms[0].transform.rotation.x,
    msg.transforms[0].transform.rotation.y,
    msg.transforms[0].transform.rotation.z)
  pose.stamp = msg.transforms[0].header.stamp
  poses[msg.transforms[0].child_frame_id].append(pose)

# Analyze all the data
fig_trajectories = pyplot.figure()
axis_trajectories = fig_trajectories.gca(projection='3d')
fig_error, axis_error = pyplot.subplots(1)
trackers = poses.items()
positions = dict()
normals = dict()
offsets = dict()
errors = dict()
abs_errors = dict()
for tracker, _ in trackers:
  positions[tracker] = (list(), list(), list())
  for i in range(0, poses[tracker].__len__()):
    positions[tracker][0].append(poses[tracker][i].translation[0,0])
    positions[tracker][1].append(poses[tracker][i].translation[1,0])
    positions[tracker][2].append(poses[tracker][i].translation[2,0])
  axis_trajectories.plot(positions[tracker][0],
    positions[tracker][1],
    positions[tracker][2],
    label = tracker)
  # min(x) y = ||Ax + b||**2
  A = np.matrix([positions[tracker][0], positions[tracker][1]]).transpose()
  A = np.hstack((A,np.ones((A.shape[0],1))))
  b = np.matrix(positions[tracker][2]).transpose()
  solution = np.linalg.inv(A.transpose() * A) * A.transpose() * (-b)
  normal = solution[0:2]
  normal = np.vstack((normal,np.matrix(1.0)))
  offsets[tracker] = solution[2] / np.linalg.norm(normal)
  normals[tracker] = normal / np.linalg.norm(normal)
  # Compute the error
  errors[tracker] = list()
  abs_errors[tracker] = list()
  for i in range(0,positions[tracker][0].__len__()):
    errors[tracker].append(float((normals[tracker].transpose() * np.matrix([positions[tracker][0][i],
      positions[tracker][1][i],
      positions[tracker][2][i]]).transpose() + offsets[tracker]) / np.linalg.norm(normals[tracker])))
    abs_errors[tracker].append(abs(errors[tracker][-1]))
  print("Maximum plane distance for " + tracker + ": " + str(max(abs_errors[tracker])))
  print("STD of plane distance for " + tracker + ": " + str(np.std(abs_errors[tracker])))
  axis_error.plot(errors[tracker], label = tracker)

# Plotting stuff
axis_trajectories.legend()
axis_trajectories.set_title("Trajectories")
axis_error.legend()
axis_error.set_title("Distance to average plane")
pyplot.legend()
pyplot.show()
