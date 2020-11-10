\page graphlocalizer Graph Localizer

# Package Overview
The graph localizer takes imu data, optical flow features, localization image projection measurements, and ar tag measurements and fuses the data to create a localization estimate.  It relies on the imu_integration package to integrate imu measurements.  The graph localizer uses the gtsam package (Dellaert, Frank. Factor graphs and GTSAM: A hands-on introduction. Georgia Institute of Technology, 2012.) to optimize a nonlinear weighted least-squares problem where the weights are the measurement uncertainties and the least-squares errors are measurement errors.  It relies on the `CombinedImuFactor` (Forster, Christian, et al. "IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation." Georgia Institute of Technology, 2015.) from gtsam which intelligently integrates imu measurements and provides an error function for relative states (consisting of pose, velocity, and imu biases) that depends on each state.  Additionally, the graph localizer uses the `SmartProjectionFactor` (Carlone, Luca, et al. "Eliminating conditionally independent sets in factor graphs: A unifying perspective based on smart factors." 2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014.) which is a structureless visual odometry factor that combines feature tracks from different camera poses and uses these measurements to estimate first the 3D feature, then project this feature into each measurement to create the factor.  The factor is structureless since it does not optimize for the 3D feature, rather it marginalizes this out using a similar null-space projection as presented in (TODO: cite msckf paper).  Additionally, image projection measurements from map to image space are incorporated into the graph.   

## Ros Node
The ros node subscribes to the measurement topics and publishes a graph localization state message containing state estimates and covariances.  Optionally, the node also publishes a serialized graph localizer that can be used to save the graph localizer and for visualization in rviz using one of the available localization plugins (see localization_rviz_plugins).

# Important Classes

## Graph Values
While the graph localizer contains a factor graph to store factors, state estimates are stored in the GraphValues object.  This provides the option to `slide_window` which maintains the correct duration and number of state parameters for the graph.  


## Feature Tracker
Optical flow feature tracks are stored in the FeatureTracker class which also support removing old measurements.  


## Sanity Checker
The sanity checker optionally checks localization estimates using either the current localization pose compared to some other source (sparse mapping) or the localization pose covariance.

# Inputs
* `/loc/of/features`
* `/loc/ml/features`
* `/loc/ar/features`
* `/hw/imu`

# Outputs
* `graph_loc/graph`
* `graph_loc/state`
