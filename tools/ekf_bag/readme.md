\defgroup ekfbag EKF Bag
\ingroup tools

This package provides scripts used to evaluate the EKF and
localization on recorded bag files. To record the bags, use the
command

    rosbag record /hw/cam_nav /hw/imu /loc/truth

The bag files should begin with at least 5 seconds of the robot
remaining stationary so that we can estimate the imu bias.

# ekf\_graph

Run `ekf_graph bag.bag` to create a pdf showing the results of the EKF
on the recorded bag.

# rosbag\_to\_csv

Run `rosbag\_to\_csv bag.bag` to create a CSV file detailing the
results of a run, which can then be passed to the GNC Matlab code for
testing.

# sparse\_map\_eval

This tool evaluates how well a robot localizes images from a bag
against a sparse map.

To use, first copy the desired robot's config file to the current
directory, for example, as:

cp ~/freeflyer/astrobee/config/robots/bumble.config robot.config

Then run the tool as:

python ~/freeflyer/tools/ekf\_bag/scripts/sparse\_map\_eval mybag.bag mymap.map

It will print on output some text like:

Localized 70 / 71 images with mean of 33 features.=====================] 100%
Time per Frame: 0.32161 s
CPU Time per Frame: 0.27612 s
Run time: 23.9871

