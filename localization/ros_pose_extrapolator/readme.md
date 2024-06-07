\page rosposeextrapolator Ros Pose Extrapolator

# Package Overview
Extrapolates localization estimates using relative odometry and IMU measurements. 
The ros pose extrapolator wrapper takes ros messages, converts these to localization measurements, and performs the required extrapolation. The ros pose extrapolator nodelet handles live subscribing and publishing to ros topics.

## RosPoseExtrapolatorWrapper
Converts ROS messages to localization measurements and extrapolates localization estimates using relative odometry and interpolated IMU measurements. 

## RosPoseExtrapolatorNodelet
Subscribes to ROS messages for online use and publishes localization messages and TFs. Contains a RosPoseExtrapolatorWrapper and passes messages to this.

# Inputs
* `/hw/imu`
* `/graph_localizer/state`
* `/graph_vio/state`

# Outputs
* `/gnc/Ekf` (message named Ekf for historical reasons)
