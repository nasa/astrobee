\page rosgraphVIO Ros Graph VIO

# Package Overview
Ros wrapper for the graph VIO. The graph VIO wrapper takes ros messages, convers these to localization measurements, and passes these to the GraphVIO object. The ros graph VIO nodelet handles live subscribing and publishing to ros topics.

## RosGraphVIOWrapper
Contains the GraphVIO. Converts ROS messages to localization measurements and provides these to the VIO object.

## ImuBiasInitializer
Assumes standstill and estimates IMU biases and stddevs from a sequence of IMU measurements.

# Inputs
* `/hw/imu`
* `/loc/of/features`
* `/loc/depth/odom`
* `/mob/flight_mode`

# Outputs
* `/graph_vio/state`
