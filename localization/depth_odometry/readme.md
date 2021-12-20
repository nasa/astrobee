\page depthodometry Depth Odometry

# Package Overview

### DepthOdometryNodelet
The DepthOdometryNodlet subscribes to ROS messages for online use and publishes DepthOdometry messages.

### DepthOdometryWrapper
The DepthOdometryWrapper joins input point clouds and intensity images with the same timestamp and passes the resulting depth image measurement to the chosen DepthOdometry class.

### DepthOdometry
Base class for performing depth odometry using input depth image measurements. Child classes include ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry and PointToPlaneICPDepthOdometry. 

### ImageFeaturesWithKnownCorrespondencesAlignerDepthOdometry 
Tracks image features between successive depth image measurements using only the intensity images of the measurements and performs alignement with the correspondending 3d points. Supports brisk, surf, and lucas kanade optical flow tracking and optimization-based point to point, point to plane, and symmetric point to plane 3d point aligment. See the point\_cloud\_common package for more details on aligment options and the vision\_common package for more details on image feature tracking options.

### PointToPlaneICPDepthOdometry 
Utilizes a chosen variant of point to plane ICP (nonsymmetric cost, symmetric cost, coarse to fine, etc.) to align successive depth image measurements.  See the point\_cloud\_common package for more details on the point to plane ICP options. 

# Inputs
* `/hw/depth\_haz/extended/amplitude\_int`
* `/hw/depth\_haz/points`

# Outputs
* `/loc/depth/odometry`
