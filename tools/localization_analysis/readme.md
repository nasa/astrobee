\page localizationanalysis Localization Analysis

# Package Overview
The localization analysis package provides several tools for measuring localization performance as described below.

## Usage Instructions
For each tool and script, run `rosrun bag_processing tool_or_script_name -h` for further details and 
usage instructions.

# Tools
## `convert_depth_msg`
Converts messages for depth topic in provided bagfile to intensity images.

## `run_bag_imu_filterer`
Reads through a bag file and filters imu measurements, replacing the old imu measurements with new filtered ones.
Saves output to a new bagfile. The filtered data can be plotted and analyzed by the imu\_analyzer script.

## `run_depth_odometry_adder`
Adds depth odometry relative poses to a new bag file.

## `run_graph_bag`
Runs graph localization on a bagfile and saves the results to a new bagfile that can be processed by the plot_results_main.py script into a pdf showing information such as poses estiamtes, velocity estimates, bias estiates, covariances, and more.
Rather than relying on rosbag play, it loads measurements directly and greatly decreases runtime.  To accurately simulate measurement delays and drops, the LiveMeasurementSimulator class is provided along with a config file to provide delays and minimum spacing between measurements. 

## `run_imu_bias_tester_adder`
Adds imu bias tester predictions to a new bag file using recorded localization states and IMU msgs.
The IMU biases are taken from the recorded localization state msgs and applied to the IMU msgs to generate 
IMU poses.
These predictions are plotted in `run_graph_bag` against provided groundtruth to measure the accuracy of 
recorded IMU biases.

## `run_sparse_mapping_pose_adder`
Adds sparse mapping poses to a new bag file using sparse mapping feature messages and body_T_nav_cam extrinsics from the robot config file.

# Scripts
## `bag_and_parameter_sweep`
Runs a parameter sweep on a set of bagfiles.  See parameter_sweep.py and bag_sweep.py for more details
on parameter and bag sweeps.

## `bag_sweep`
The bag sweep tool runs the graph bag tool in parallel on multiple bag files.  It takes config file with bag names, map names, and robot configs and produces pdfs and result bagfiles for each entry.
Example config file:   
```
/home/bag_name.bag /home/map_name.map /mgt/img_sampler/nav_cam/image_record /home/astrobee/src/astrobee config/robots/bumble.config iss false  
/home/bag_name_2.bag /home/map_name.map /mgt/img_sampler/nav_cam/image_record /home/astrobee/src/astrobee config/robots/bumble.config iss false
```
Example bag sweep command:  
```
rosrun localization_analysis bag_sweep.py /home/bag_sweep_config.csv /home/output_dir
``` 

## `depth_odometry_parameter_sweep`
## `get_average_opt_and_update_times`
## `groundtruth_sweep`
## `imu_analyzer`
## `make_groundtruth`
## `make_map`
## `parameter_sweep`
## `plot_all_results`
## `plot_results`
## `run_graph_bag_and_plot_results`
## `plot_all_results`


## imu\_analyzer\_main
The imu analyzer tool plots imu data and also filtered imu data.  It can use python filtering tools to lowpass filter imu data at a certain cutoff frequency, or it can use a provided bag file with filtered imu data and compare this against a bag file with unfiltered data.  Data is plotted in its raw form and also after being passed through an FFT to show its frequency response.

## run\_graph\_bag\_and\_plot\_results
Generates localization results for a provided bagfile.
See 'rosrun graph\_bag run\_graph\_bag\_and\_plot\_results.py -h'
for further details and usage instructions.

## make\_groundtruth.py
Generates groundtruth map and bagfile for a given bagfile.
See 'rosrun graph\_bag make\_groundtruth.py -h'
for further details and usage instructions.

## make\_groundtruth\_map.py
Generates groundtruth map for a given bagfile.
See 'rosrun graph\_bag make\_groundtruth.py -h'
for further details and usage instructions.

## groundtruth\_sweep.py
Creates groundtruth in parallel for a set of bagfiles.
See 'rosrun graph\_bag groundtruth\_sweep.py -h'
for further details and usage instructions.

## depth\_odometry\_parameter\_sweep.py
Runs a parameter sweep for depth odometry on a bagfile and plots the results.
See 'rosrun depth\_odometry\_parameter\_sweep.py -h'
for further details and usage instructions.
