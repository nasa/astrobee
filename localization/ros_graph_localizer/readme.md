\page rosgraphlocalizer Ros Graph Localizer

# Package Overview
Ros wrapper for the graph localizer. The graph localizer wrapper takes ros messages, convers these to localization measurements, and passes these to the graph localizer. The ros graph localizer nodelet handles live subscribing and publishing to ros topics. It runs the ros_graph_localizer, ros_graph_vio, and depth_odometry correspondence estimation sequential and passes input messages to each of these.

## RosGraphLocalizerWrapper
Contains the GraphLocalizer. Converts ROS messages to localization measurements and provides these to the localizer.

## RosGraphVIONodelet
Subscribes to ROS messages for online use and publishes GraphVIO messages and TFs. Contains a RosGraphLocalizerWrapper and passes messages to this.

# Inputs
* `/hw/depth_haz/extended/amplitude_int`
* `/hw/depth_haz/points`
* `/hw/imu`
* `/loc/ar/features`
* `/loc/of/features`
* `/loc/ml/features`
* `/loc/depth/odom`
* `/loc/depth/odom`
* `/mob/flight_mode`

# Outputs
* `/graph_loc/state`
* `/graph_vio/state`
* `/loc/depth/odom`
