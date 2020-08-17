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

cp ~/astrobee/astrobee/config/robots/bumble.config robot.config

Then run the tool as:

python ~/astrobee/tools/ekf_bag/scripts/sparse_map_eval mybag.bag mymap.map

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

# ekf_diff

This is a tool for regression testing localization.  It plots the difference between the ekf output of two sets of data generated from a single bag file, and has several features defined as flags.
To run, first generate text files containing EKF output using ekf_graph.  Then, run ekf_diff::
'rosrun ekf_bag ekf_diff bag1.txt bag2.txt'
The flags are defined as follows:
-s: define start time
-e: define end time
-p: define a minimum time difference between poses to compare, rather than all of them
-i: resample data to match the first .txt file rather than selecting the neares timestamped data
-r: use relative poses; at each measurement, report the difference between the current measurement and the last one rather than absolute positions on the global map
-m: generate a heatmap of where in the ISS localization is less accurate
-sm: use the sparse mapping poses from the first text file supplied as ground truth
This tool relies on scipy 1.2.0: to upgrade this, run: 'python -m pip install scipy==1.2'

# streamlit webserver

This is a webserver to run some of the ekf_bag scripts and automatically cache results.
The server requires python >= 3.6 (which will not be already installed from ROS) and the streamlit package installed using pip.  After installing python3.6, run 'python3.6 -m pip install streamlit'.  If this command fails, upgrade pip and retry.  Additionally, install pdf2image by running: 'python3.6 -m pip install pdf2image'.  Create a folder to store all data in.  Then, inside that folder, create folders named "bags" and "maps".  Put any map files in the "maps" folder, and any bag files in the "bags" folder.  The "bags" folder also must contain a configuration file named robot.config (can be copied from astrobee/astrobee/config directory).  Then, from the first folder you created, run:
'streamlit run /path/to/streamlit_server.py'

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
