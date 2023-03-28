\page graphvio Graph VIO

# Package Overview
The GraphVIO uses the GraphOptimizer (please first read the documentation in the GraphOptimizer package as this package uses many objects from there) to track the pose, velocity and IMU biases (the set of these three state parameters is referred to as a CombinedNavState) for a robot.  It uses a set of measurements (IMU and optical flow) to generate factors that constrain its history of CombinedNavStates.  
## Code Structure
The GraphVIO package contains the GraphVIO, GraphVIOWrapper, and GraphVIONodelet objects.  The GraphVIO handles the creation of factors and contains the GraphOptimizer, while the GraphVIOWrapper provides an interface that can be used both online with the GraphVIONodelet and offline in a ROS free environement as is done in the GraphBag tool (see GraphBag package).  The GraphVIOWrapper converts ROS messages to localization_measurement objects and subsequently passes these to the GraphVIO so that the GraphVIO does not contain any ROS code.  Finally, the GraphVIONodelet is a ROS nodelet that subscribes to the required messages and passes these to the GraphVIOWrapper.  It also publishes required messages and TFs for online usage.

# Background
For more information on the theory behind the GraphVIO and the factors used, please see our paper: 
* Ryan Soussan, Varsha Kumar, Brian Coltin, and Trey Smith, "Astroloc: An efficient and robust localizer for a free-flying robot", Int. Conf. on Robotics and Automation (ICRA), 2022. [Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9811919 "Link")

# Graph Optimization Structure 
## Factor Adders
* SmartProjectionCumulativeFactorAdder
* StandstillFactorAdder 
## Graph Factors
* CombinedIMUFactor
* RobustSmartProjectionFactor
* Standstill Factors (zero velocity prior and identity relative transform between factors)
## Node Updaters
* CombinedNavStateNodeUpdater 

# Important Classes

## Graph VIO
The GraphVIO is a GraphOptimizer that contains measurement specific FactorAdders and state parameter specific NodeUpdaters.  

## Other
### Sanity Checker
The SanityChecker validates the current VIO pose by checking that the covariances are within defined bounds.

### GraphVIOInitializer
The GraphVIOInitializer is responsible for loading GraphVIO parameters.  It also estimates initial IMU biases using a set of N consecutive measurements at standstill.

### GraphVIOWrapper
The GraphVIOWrapper contains the GraphVIO, GraphVIOInitializer, and SanityChecker.  It passes ROS messages to each of these and also provides interfaces for accessing some GraphVIO data such as messages built using GraphVIO values.

### GraphVIONodelet
The GraphVIONodlet subscribes to ROS messages for online use and publishes GraphVIO messages and TFs.

# Inputs
* `/loc/of/features`
* `/hw/imu`

# Outputs
* `graph_vio/graph`
* `graph_vio/state`
