\defgroup vive Vive Driver
\ingroup hw

This package provides drivers for pulling lighthouse, tracker, inertial, button and light capture data for the HTC Vive Lighthouse tracking system, and converting all data to ROS messages. The drivers are not loaded at runtime by default. To enable them, append extra:=vive to your launch command. For example:

    roslaunch astrobee granite.launch extra:=vive

All data is published as messages on ROS topics using the prefix hw/vive. You will need to record these topics in a bag and post-process them to get a localization solution.