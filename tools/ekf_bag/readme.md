\page ekfbag EKF Bag

This package provides scripts used to evaluate the EKF and
localization on recorded bag files. To record the bags, use the
command

    rosbag record /hw/cam_nav /hw/imu /loc/truth

The bag files should begin with at least 5 seconds of the robot
remaining stationary so that we can estimate the imu bias.

# ekf_graph

Run `ekf_graph bag.bag` to create a pdf showing the results of the EKF
on the recorded bag.

# rosbag_to_csv

Run `rosbag_to_csv bag.bag` to create a CSV file detailing the
results of a run, which can then be passed to the GNC Matlab code for
testing.

# sparse_map_eval

This tool evaluates how well a robot localizes images from a bag
against a sparse map.

To use, first copy the desired robot's config file to the current
directory, for example, as:

cp ~/freeflyer/astrobee/config/robots/bumble.config robot.config

Then run the tool as:

python ~/freeflyer/tools/ekf_bag/scripts/sparse_map_eval mybag.bag mymap.map

It will print on output some text like:

Localized 70 / 71 images with mean of 33 features.=====================] 100%
Time per Frame: 0.32161 s
CPU Time per Frame: 0.27612 s
Run time: 23.9871

If, before launching this tool one runs:

  export SAVE_FAILED_IMAGES_DIR=failed_images
  mkdir -p $SAVE_FAILED_IMAGES_DIR

it will save the images for which localization failed (with the image
timestamp as image name) in the specified directory.

# parameter_sweep.py
This tool runs ekf_graph tests for a set of parameter combinations.  Parameter ranges and names are defined in parameter_sweep.py and a gnc.config file is created for each parameter combination in the sweep.  Each parameter combination is tested in parallel and results are saved locally to a timestamped directory or user defined directory. 

# create_plots.py
Uses output of parameter_sweep.py and generates plots comparing the results of the sweep.

# bag_sweep.py
Runs ekf tests using a given gnc config file on a set of bag files in parallel.

# bag_and_parameter_sweep.py
Calls parameter_sweep on a set of bag files.  

# create_average_plots.py
Averages results for each job_id run over multiple tests and creates plots.  Used for plotting bag_sweep.py results.
