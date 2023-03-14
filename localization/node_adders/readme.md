\page nodeupdaters Node Updaters

# Package Overview
The node updaters package provides a set of node updaters for various graph value types for use with a graph optimizer. See the GraphOptimizer package for more information on their usage.

## NodeUpdaters
### CombinedNavStateNodeUpdater
The CombinedNavStateNodeUpdater is responsible for inserting and removing CombinedNavState nodes for the graph.  It links nodes using IMU preintegration factors (gtsam::CombinedIMUFactor).  If a node to be created's timestamp is between two existing nodes, the IMU factor linking the two existing nodes is split and the new node is inserted between them.  The CombinedNavStateNodeUpdater uses the LatestImuIntegrator object to store and utilize IMU measurements.  When sliding the window, old nodes are removed from the graph and prior factors are created for the new oldest node using the node's covariances from the most recent round of optimization if the add_priors option is enabled. 

### FeaturePointNodeUpdater
The FeaturePointNodeUpdater maintains feature point nodes for image features.  It removes old point nodes that are no longer being tracked by the graph.  Creation of point nodes occurs in this case in the ProjectionGraphActionCompleter which uses each measurement of a feature track to triangulate a new point node. (TODO(rsoussan): Update this when this is changed)
