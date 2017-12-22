\defgroup ekfbag EKF Bag
\ingroup tools

This package provides scripts used to evaluate the EKF on
recorded bag files. To record the bags, use the command

    rosbag record /hw/cam_nav /hw/imu /loc/truth

The bag files should begin with at least 5 seconds of the robot
remaining stationary so that we can estimate the imu bias.

# ekf\_graph

Run `ekf_graph bag.bag` to create a pdf showing the results of the EKF
on the recorded bag.

# rosbag\_to\_csv

Run `rosbag\_to\_csv bag.bag` to create a CSV file detailing the results of a run,
which can then be passed to the GNC Matlab code for testing.

