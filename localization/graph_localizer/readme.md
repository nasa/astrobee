\page graphlocalizer Graph Localizer

# Package Overview
The graph localizer fuses imu measurements, optical flow features, localization image projection measurements, and ar tag measurements to estimate a history of pose, velocity, and imu biases for desired timestamps. The graph localizer uses the gtsam package (_Dellaert, Frank. Factor graphs and GTSAM: A hands-on introduction. Georgia Institute of Technology, 2012._) to optimize a nonlinear weighted least-squares problem where the weights are the measurement uncertainties and the least-squares errors are measurement errors (factors).  It relies on the `CombinedImuFactor` (_Forster, Christian, et al. "IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation." Georgia Institute of Technology, 2015._) from gtsam which intelligently integrates imu measurements and provides an error function for relative states (consisting of pose, velocity, and imu biases) that depends on each state.  Additionally, the graph localizer uses the `SmartProjectionFactor` (_Carlone, Luca, et al. "Eliminating conditionally independent sets in factor graphs: A unifying perspective based on smart factors." 2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014._) which is a structureless visual odometry factor that combines feature tracks from different camera poses and uses these measurements to estimate first the 3D feature, then project this feature into each measurement to create the factor.  The factor is structureless since it does not optimize for the 3D feature, rather it marginalizes this out using a similar null-space projection as presented in (_Mourikis, Anastasios I., and Stergios I. Roumeliotis. "A multi-state constraint Kalman filter for vision-aided inertial navigation." Proceedings 2007 IEEE International Conference on Robotics and Automation. IEEE, 2007._).  Additionally, image projection measurements from map to image space are incorporated into the graph.   

## Ros Node
The ros node subscribes to the measurement topics and publishes a graph localization state message containing state estimates and covariances.  Optionally, the node also publishes a serialized graph localizer that can be used to save the graph localizer and for visualization in rviz using one of the available localization plugins (see localization\_rviz\_plugins).

# Important Classes

## Graph Localizer
Fuses measurements to estimate pose, velocity, and biases for a window of time (and optionally for a max number of states, where a state is a pose, velocity and bias estimate at a timestamp).  Creates error functions (factors) for each measurement and minimizes these errors by adjusting the history of state estimates using a nonlinear optimization procedure provided by gtsam.

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
