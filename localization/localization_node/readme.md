\page localization_node Localization Node

The localization node (localization_node) takes an input a sparse map built from previously acquired nav_cam images. It subscribes for incoming nav_cam images, and for each of them finds the pose of the nav_cam at the time the image was acquired based on the sparse map. It publishes this pose together with the sparse map features it used to find it on /loc/ml/features. A registration pulse is published on /loc/ml/registration.

For testing purposes, this node can be started by itself on the robot, or even on a local machine. How to do it while using a desired sparse map, is described in  

  localization/sparse_mapping/build_map.md

(at the bottom of the page).