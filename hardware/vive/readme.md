\defgroup vive Vive Driver
\ingroup hw

This package provides drivers for pulling lighthouse, tracker, inertial, button and light capture data for the HTC VIVE Lighthouse tracking system, and converting all data to ROS messages. 

VIVE is not on by default, and having it run is a two-step process. First, connect to LLP and send the command:


  eps_driver_tool -power -set on pay_ta

to turn on the power (check that the orange LED light on the trackers is on).

Then, when launching the flight software, append extra:=vive to your launch command. For example:

    roslaunch astrobee granite.launch extra:=vive

All data is published as messages on ROS topics using the prefix hw/vive. You will need to record these topics in a bag and post-process them to get a localization solution.