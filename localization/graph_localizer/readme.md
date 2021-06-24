\page graphlocalizer Graph Localizer

# Package Overview
The GraphLocalizer uses the GraphOptimizer (see documentation in GraphOptimizer package) to track the pose, velocity and IMU biases (the set of these three state parameters is referred to as a CombinedNavState) for a robot.  It uses a set of measurements (IMU, optical flow, map-based image features, ar tag detections, 3D handrail and wall plane points) to generate factors that constrain its history of CombinedNavStates.  
## Code Structure
The GraphLocalizer package consists of the GraphLocalizer, GraphLocalizerWrapper, and GraphLocalizerNodelet objects.  The GraphLocalizer handles the creation of factors and contains the GraphOptimizer, while the GraphLocalizerWrapper provides an interface that can be used both online with the GraphLocalizerNodelet and offline in a ROS free environement such as is done in the GraphBag tool (see GraphBag package).  The GraphLocalizerWrapper converts ROS messages to localization_measurement objects and subsequently passes these to the GraphLocalizer so that the GraphLocalizer does not contain any ROS code.  Finally, the GraphLocalizerNodelet is a ROS nodelet that subscribes to the required messages and passes these to the GraphLocalizerWrapper.  It also publishes required messages and TFs for online usage. 

# Background
For more information on the theory behind the GraphLocalizer and the factors used, please see our paper (TODO(rsoussan): link paper when publicly available).  
# Background (until paper is linked)
 The GraphLocaluzer uses the `CombinedImuFactor` (_Forster, Christian, et al. "IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation." Georgia Institute of Technology, 2015._) from gtsam which intelligently integrates imu measurements and provides an error function for relative states (consisting of pose, velocity, and imu biases) that depends on each state.  Additionally, the graph localizer uses the `SmartProjectionFactor` (_Carlone, Luca, et al. "Eliminating conditionally independent sets in factor graphs: A unifying perspective based on smart factors." 2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014._) which is a structureless visual odometry factor that combines feature tracks from different camera poses and uses these measurements to estimate first the 3D feature, then project this feature into each measurement to create the factor.  The factor is structureless since it does not optimize for the 3D feature, rather it marginalizes this out using a similar null-space projection as presented in (_Mourikis, Anastasios I., and Stergios I. Roumeliotis. "A multi-state constraint Kalman filter for vision-aided inertial navigation." Proceedings 2007 IEEE International Conference on Robotics and Automation. IEEE, 2007._).  Additionally, image projection measurements from map to image space are incorporated into the graph.   


# Important Classes

## Graph Localizer
Fuses measurements to estimate pose, velocity, and biases for a window of time (and optionally for a max number of states, where a state is a pose, velocity and bias estimate at a timestamp).  Creates error functions (factors) for each measurement and minimizes these errors by adjusting the history of state estimates using a nonlinear optimization procedure provided by gtsam.

## Graph Values
While the graph localizer contains a factor graph to store factors, state estimates are stored in the GraphValues object.  This provides the option to `slide_window` which maintains the correct duration and number of state parameters for the graph.  


## Feature Tracker
Optical flow feature tracks are stored in the FeatureTracker class which also support removing old measurements.  


## Sanity Checker
The sanity checker optionally checks localization estimates using either the current localization pose compared to some other source (sparse mapping) or the localization pose covariance.

## Ros Node
The ros node subscribes to the measurement topics and publishes a graph localization state message containing state estimates and covariances.  Optionally, the node also publishes a serialized graph localizer that can be used to save the graph localizer and for visualization in rviz using one of the available localization plugins (see localization\_rviz\_plugins).


# Inputs
* `/loc/of/features`
* `/loc/ml/features`
* `/loc/ar/features`
* `/hw/imu`

# Outputs
* `graph_loc/graph`
* `graph_loc/state`
