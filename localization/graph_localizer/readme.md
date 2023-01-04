\page graphlocalizer Graph Localizer

# Package Overview
The GraphLocalizer uses the GraphOptimizer (please first read the documentation in the GraphOptimizer package as this package uses many objects from there) to track the pose of a robot.  It uses a set of measurements (VIO, map-based image features, ar tag detections, 3D handrail and wall plane points) to generate factors that constrain its history of CombinedNavStates.  
## Code Structure
The GraphLocalizer package contains the GraphLocalizer, GraphLocalizerWrapper, and GraphLocalizerNodelet objects.  The GraphLocalizer handles the creation of factors and contains the GraphOptimizer, while the GraphLocalizerWrapper provides an interface that can be used both online with the GraphLocalizerNodelet and offline in a ROS free environement as is done in the GraphBag tool (see GraphBag package).  The GraphLocalizerWrapper converts ROS messages to localization_measurement objects and subsequently passes these to the GraphLocalizer so that the GraphLocalizer does not contain any ROS code.  Finally, the GraphLocalizerNodelet is a ROS nodelet that subscribes to the required messages and passes these to the GraphLocalizerWrapper.  It also publishes required messages and TFs for online usage.

# Background
For more information on the theory behind the GraphLocalizer and the factors used, please see our paper:
* Ryan Soussan, Varsha Kumar, Brian Coltin, and Trey Smith, "Astroloc: An efficient and robust localizer for a free-flying robot", Int. Conf. on Robotics and Automation (ICRA), 2022. [Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9811919 "Link")

# Graph Optimization Structure 
## Factor Adders
* LocFactorAdder
(TODO: add handrail, vio, depth adders)
## Graph Factors
(TODO: add vio, depth factor)
LocPoseFactor
LocProjectionFactor
PointToHandrailEndpointFactor
PointToLineFactor
PointToLineSegmentFactor
PointToPlaneFactor
## Node Updaters
(TODO: update this)

# Important Classes

## Graph Localizer
The GraphLocalizer is a GraphOptimizer that contains measurement specific FactorAdders and state parameter specific NodeUpdaters. 

### Sanity Checker
The SanityChecker validates the current localization pose using a reference pose or by checking that the covariances are within defined bounds.
### GraphLocalizerInitializer
The GraphLocalizerInitializer is responsible for loading GraphLocalizer parameters. It also stores the starting pose for the localizer.

### GraphLocalizerWrapper
The GraphLocalizerWrapper contains the GraphLocalizer, GraphLocalizerInitializer, and SanityChecker.  It passes ROS messages to each of these and also provides interfaces for accessing some GraphLocalizer data, such as messages built using GraphLocalizer values, estimates of the dock and handrail poses wrt the world frame, and more.

### GraphLocalizerNodelet
The GraphLocalizerNodlet subscribes to ROS messages for online use and publishes GraphLocalizer messages and TFs.

# Inputs
* `/loc/vio/odometry` (TODO: update this)
* `/loc/ml/features`
* `/loc/ar/features`
* `/loc/handrail/features`

# Outputs
* `graph_loc/graph`
* `graph_loc/state`
