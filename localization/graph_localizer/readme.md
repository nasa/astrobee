\page graphlocalizer Graph Localizer

The graph localizer takes imu data, optical flow features, localization image projection measurements, and ar tag measurements and fuses the data to create a localization estimate.  It relies on the imu_integration package to integrate imu measurements.  The graph localizer uses the gtsam package (TODO: cite) to optimize a nonlinear weighted least-squares problem where the weights are the measurement uncertainties and the least-squares errors are measurement errors.  It relies on the `CombinedImuFactor` (TODO: cite) from gtsam which intelligently integrates imu measurements and provides an error function for relative states (consisting of pose, velocity, and imu biases) that depends on each state.  Additionally, the graph localizer uses the `SmartProjectionFactor` (TODO: cite) which is a structureless visual odometry factor that combines feature tracks from different camera poses and uses these measurements to estimate first the 3D feature, then project this feature into each measurement to create the factor.  The factor is structureless since it does not optimize for the 3D feature, rather it marginalizes this out using a similar null-space projection as presented in (TODO: cite msckf paper).  Additionally, image projection measurements from map to image space are incorporated into the graph.   

# Inputs
* `/localization/optical_flow/features`

# Outputs

