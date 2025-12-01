\page localization Localization

This folder consists of libraries for the graph-based localizer, sparse mapping pipeline, sensor inputs for localization, and other supporting libraries. 

See the following publications for more details:
* Ryan Soussan, Marina Moreira, Brian Coltin, and Trey Smith, "AstroLoc2: Fast Sequential Depth-Enhanced Localization for Free-Flying Robots", Int. Conf. on Robotics and Automation (ICRA), 2025. [Link](https://ieeexplore.ieee.org/abstract/document/11128673 "Link") [Link](https://ntrs.nasa.gov/api/citations/20250001870/downloads/astroloc2_final.pdf "PDF")
  * Latest publication describing the current sequential graph-based localization system for Astrobee

* Ryan Soussan, Varsha Kumar, Brian Coltin, and Trey Smith, "Astroloc: An efficient and robust localizer for a free-flying robot", Int. Conf. on Robotics and Automation (ICRA), 2022. [Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9811919 "Link")
  * Previous publication describing the former tightly-coupled graph-based localization system for Astrobee

* Brian Coltin, Jesse Fusco, Zack Moratto, Oleg Alexandrov and Robert Nakamura, "Localization from visual landmarks on a free-flying robot," Int. Conf. on Intelligent Robots and Systems (IROS), 2016. [Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7759644 "Link")
  * Original publication describing the initial EKF-based localizer and mapping system for Astrobee

If you are using the localizer or mapping pipeline in academic work, please cite the relevant publications above.

# AstroLoc Library Quickstart
See [Quickstart](doc/astroloc_library_quickstart.pdf) for a tutorial on using the AstroLoc library and an example simple localizer.

# Localizer Implementations
## GraphVIO
The graph_vio package performs VIO using image-based feature track measurements and estimates pose, velocity, and IMU bias values at each timestamp.

<p align="center">
<img src="./doc/images/graph_vio.png" width="550">
</p>

## GraphLocalizer
The graph_localizer package uses relative odometry measurements along with map-based localization measurements to perform localization for poses at each timestamp.


<p align="center">
<img src="./doc/images/graph_localizer.png" width="550">
</p>

## Ros Wrappers
The ros_graph_localizer and ros_graph_vio packages wrap GraphVIO and GraphLocalizer objects with ROS for live or offline usage with ROS message types, and the ros_pose_extrapolator performs extrapolation of localization poses using relative odometry and interpolated IMU data.

# Packages
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
