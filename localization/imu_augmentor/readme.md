\page imuaugmentor Imu Augmentor

# Package Overview
The imu augmentor extrapolates a localization estimate with imu data.  This is useful in order to "catch up" the latest localization estimate to current time, for use with a planner or other module.  The imu augmentor uses an ImuIntegrator object to maintain a history of imu measurements and integrate up to the most recent imu measurement.  Imu biases are reset on receival of localization estimates to more accurately integrate measurements. 

## Ros Node
The ros node subscribes to the localization estimate and publishes an updated imu augmented localization estimate.  It also fills in the latest acceleration and angular velocity.  The augmentor also publishes the latest pose and twist as respective messages. 

# Inputs
* `/hw/imu`
* `graph_loc/state`

# Outputs
* `gnc/ekf`
* `loc/pose`
* `loc/twist`
