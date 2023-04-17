\page localization Localization

This folder consists of libraries for the graph-based localizer, sparse mapping pipeline, sensor inputs for localization, and other supporting libraries. 

See the following publications for more details:
* Ryan Soussan, Varsha Kumar, Brian Coltin, and Trey Smith, "Astroloc: An efficient and robust localizer for a free-flying robot", Int. Conf. on Robotics and Automation (ICRA), 2022. [Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9811919 "Link")
  * Latest publication describing the current graph-based localization system for Astrobee

* Brian Coltin, Jesse Fusco, Zack Moratto, Oleg Alexandrov and Robert Nakamura, "Localization from visual landmarks on a free-flying robot," Int. Conf. on Intelligent Robots and Systems (IROS), 2016. [Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7759644 "Link")
  * Original publication describing previous EKF-based localizer and mapping system for Astrobee

If you are using the localizer or mapping pipeline in academic work, please cite the relevant publications above.

# Graph-based localization
Various packages exist for graph-based localization. The optimizers package provides nonlinear and ISAM2 based optimizers that are used by the graph_optimizer and sliding_window_graph_optimizer packages that perform graph-based optimization using GTSAM. 
The graph_optimizer package uses node_adders and factor_adders for node and factor creation and a provided optimizer for optimization, while the sliding_window_graph_optimizer uses sliding_window_node_adders for node creation and maintiains a graph with a windowed duration moving in time. 
The node_adders create nodes (optimization states) and relative factors for a set of types (i.e. pose/velocity/bias) given timestamped measurements (i.e. relative odometry or IMU). 
Node adders add nodes and relative factors when instructed by factor adders, which generate factors for specific measurements (i.e. map-based localization measurements or visual odometry measurements) at certain timestamps.
The graph_vio package performs VIO using image-based feature track measurements and estimates pose, velocity, and IMU bias values at each timestamp.
The graph_localizer package uses relative odometry measurements along with map-based localization measurements to perform localization for poses at each timestamp.
Finally, the ros_graph_localizer and ros_graph_vio packages wrap each of these object with ROS for live or offline usage with ROS message types, and the ros_pose_extrapolator performs extrapolation of localization poses using relative odometry and interpolated IMU data.
TODO(rsoussan): Add images to help explain these ideas.

\subpage camera

\subpage depthodometry

\subpage factoradders 

\subpage graphfactors

\subpage graphlocalizer

\subpage graphoptimizer

\subpage graphvio

\subpage groundtruthlocalizer

\subpage handrail

\subpage imuintegration

\subpage interestpoint

\subpage localizationcommon

\subpage localizationmanager

\subpage localizationmeasurements

\subpage localizationnode

\subpage markertracking

\subpage nodeadders

\subpage nodes

\subpage opticalflow

\subpage optimizationcommon 

\subpage optimizers 

\subpage pointcloudcommon 

\subpage rosgraphlocalizer 

\subpage rosgraphvio 

\subpage rosposeextrapolator 

\subpage slidingwindowgraphoptimizer 

\subpage sparsemapping

\subpage visioncommon 

\subpage vivelocalization
