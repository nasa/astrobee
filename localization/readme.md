\page localization Localization

This folder consists of various sensing algorithms which serve as input to localization, along with the graph-based localizer and helper libraries for components of its functionality.  It also contains libraries for other localization functionality, such as imu augmentation, a ground truth localizer primarily for use with simulation, and a vive localization package.

See the following publications for more details:
* Ryan Soussan, Varsha Kumar, Brian Coltin, and Trey Smith, "Astroloc: An efficient and robust localizer for a free-flying robot", Int. Conf. on Robotics and Automation (ICRA), 2022.[Named Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9811919 "Link")
  * Latest publication describing the current graph-based localization system for Astrobee
As well the previous localizer and mapping system are described here:
* Brian Coltin, Jesse Fusco, Zack Moratto, Oleg Alexandrov and Robert Nakamura, "Localization from visual landmarks on a free-flying robot," Int. Conf. on Intelligent Robots and Systems (IROS), 2016. [Named Link](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7759644 "Link")
  * Original publication describing previous EKF-based localizer and mapping system for Astrobee
If you are using the localizer in academic work, please cite the relevant publications above.
\subpage camera

\subpage depthodometry

\subpage graphlocalizer

\subpage graphoptimizer

\subpage groundtruthlocalizer

\subpage handrail

\subpage imuaugmentor
 
\subpage imuintegration

\subpage interestpoint

\subpage localizationcommon

\subpage localization_manager

\subpage localizationmeasurements

\subpage localization_node

\subpage opticalflow

\subpage optimizationcommon 

\subpage pointcloudcommon 

\subpage markertracking

\subpage sparsemapping

\subpage visioncommon 

\subpage vive_localization
