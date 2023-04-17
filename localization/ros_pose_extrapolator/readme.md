\page rosposeextrapolator Ros Pose Extrapolator

# Package Overview
Extrapolates localization estimates using relative odometry and IMU measurements. 
The ros pose extrapolator wrapper takes ros messages, convers these to localization measurements, performs the required extrapolation. The ros pose extrapolatornodelet handles live subscribing and publishing to ros topics.

## RosPoseExtrapolatorWrapper
Converts ROS messages to localization measurements and extrapolates localization estimates using relative odometry and interpolated IMU measurements. 

## RosPoseExtrapolatorNodelet
Subscribes to ROS messages for online use and publishes localization messages and TFs. Contains a RosPoseExtrapolatorWrapper and passes messages to this.

# Inputs
* `/hw/imu`
* `graph_localizer/state`
* `graph_vio/state`

# Outputs
* `gnc/Ekf` (message named Ekf for historical reasons)
