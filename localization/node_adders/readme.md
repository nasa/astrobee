\page nodeadder Node Adders

# Package Overview
The node adders package provides various node adders that add nodes given a timestamp to a graph along with connecting relative factors. The SlidingWindowNodeAdder is used with a SlidingWindowGraphOptimizer and provides functions for removing old nodes and relative factors. Implementations of the sliding window node adder are provided such as the timestamped node adder that uses a node adder model to create relative factors and the measurement based node adder which is a timestamped node adder that uses provided measurements to create relative factors and nodes. See the pose node adder/adder model and combined nav state node adder/adder model for examples of these. The BetweenFactorAdderModel creates GTSAM between factors as relative factors and GTSAM prior factors for priors for a single gtsam type. Use this when possible, and see the PoseNodeAdderModel as an example.  

## NodeAdder
Base class for other node adders, interface for adding/accessing nodes and keys given their timestamps and adding initial values and priors for the first nodes.

## SlidingWindowNodeAdder
Base class node adder that extends the NodeAdder with functions that enable sliding the window, which consists of removed nodes older than a desired time. The SlideWindowNewStartTime function provides the desired start time for the window for the node adder, but the window size is ultimately decided by the SlidingWindowGraphOptimizer using each desired start time for each sliding window node adder in the graph. 

## TimestampedNodeAdder
SlidingWindowNodeAdder that adds nodes given a provided node adder model. Provides functions for adding new relative factors and splitting old ones when out of order nodes are received.

## TimestampedNodeAdderModel
Helper base class for TimestampedNodeAdder that adds and removes relative factors and priors given relative timestamps. 

## MeasurementBasedTimestampedNodeAdder
Base class that extends the TimestampedNodeAdder to enable adding a measurement from which nodes will be generated. Measurements are forwarded to the MeasurementBasedTimestampedNodeAdderModel.

## MeasurementBasedTimestampedNodeAdderModel
Virtual class that extends the TimestampedNodeAdderModel with Add/Remove measurement functions. Measurements are used to generate relative factors and priors.

## BetweenFactorNodeAdderModel
NodeAdderModel that creates GTSAM between factors as relative factors and GTSAM prior factors as priors. Requires some function specializations for adding nodes and measurements. See PoseNodeAdderModel for an example.

## CombinedNavStateNodeAdder
MeasurementBasedTimestampedNodeAdder that uses IMU measurements as a measurement and creates CombinedNavStateNodes. Uses the CombinedNavStateNodeAdderModel to generate relative IMU factors given the IMU measurements.

## CombinedNavStateNodeAdderModel
MeasurementBasedTimestampedNodeAdderModel that generates relative IMU factors given IMU measurements.

## PoseNodeAdder
MeasurementBasedTimestampedNodeAdder that uses timestamped pose with covariance objects as a measurement and creates gtsam::Pose3 timestamped nodes.

## PoseNodeAdderModel
BetweenFactorNodeAdderModel for gtsam::Pose3 poses. Uses timestamped pose with covariance objects as a measurement. 
