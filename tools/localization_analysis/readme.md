\page localizationanalysis Localization Analysis

# Package Overview
The localization analysis package provides several tools for measuring localization performance as described below.

## Usage Instructions
For each tool and script, run `rosrun localization_analysis tool_or_script_name -h` for further details and 
usage instructions.

# Tools
## `convert_depth_msg`
Converts depth messages in provided bagfile to intensity images.

## `run_bag_imu_filterer`
Reads through a bag file and filters imu measurements, replacing the old imu measurements with new filtered ones.
The filtered data is saved to a bagfile and can be plotted and analyzed using the imu\_analyzer script.

## `run_depth_odometry_adder`
Generates depth odometry relative poses using recorded depth camera data and saves these to a bag file.

## `run_graph_bag`
Runs graph localization on a bagfile and saves the results to a new bagfile. The results can be plotted using the `plot_results.py` script.

## `run_imu_bias_tester_adder`
Generates imu bias integrated poses and saves these to a bagfile.
The results can be plotted using the `plot_results.py` script.
See the `imu_bias_tester` package readme for more details.

## `run_sparse_mapping_pose_adder`
Generates sparse mapping poses using sparse mapping feature messages and the body_T_nav_cam extrinsics from the provided robot config file.

# Scripts
## `bag_and_parameter_sweep`
Runs a parameter sweep on a set of bagfiles.  See parameter_sweep.py and bag_sweep.py for more details
on parameter and bag sweeps.

## `bag_sweep`
Runs the graph bag tool in parallel on multiple bag files.  It takes config file with bag names, map names, and robot configs and produces pdfs and result bagfiles for each entry.
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
Runs a parameter sweep for depth odometry relative pose estimation and plots the results.

## `get_average_opt_and_update_times`
Prints stats for the optimization and update times of the graph localizer
from the recorded graph_state messages in the provided bagfile.

## `groundtruth_sweep`
Creates groundtruth in parallel for a set of bagfiles.

## `imu_analyzer`
Generates comparison plots for raw and filtered IMU data.

## `make_groundtruth`
Creates a groundtruth map and bagfile containing groundtruth poses for a provided bagfile.

## `make_map`
Creates a map for a provided bagfile, optionally merged with an existing map as well.

## `parameter_sweep`
Runs a parameter sweep for the graph localizer on a bagfile and plots the results.

## `plot_all_results`
Plots localization results for each bagfile in a directory.

## `plot_results`
Plots localization information for a bagfile to a pdf. Uses either sparse mapping poses
from the input bagfile or a seperate bagfile containing groundtruth poses for comparison
with the localization estimates.

## `run_graph_bag_and_plot_results`
Generates localization estimates and plots results for a provided bagfile.

