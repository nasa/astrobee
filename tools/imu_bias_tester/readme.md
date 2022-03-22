\page imubiastester IMU Bias Tester

## ImuBiasTester
The ImuBiasTester integrates IMU measurements using the latest estimated IMU biases from graph_state messages.
The integrated measurements are published as pose messages.
These poses can then be plotted against localization groundtruth to measure the accuracy of the estimated IMU biases.

# Usage
Launch the nodelet online or while playing a recored bagfile using
`roslaunch imu_bias_tester imu_bias_tester.launch`
to generated IMU bias poses. Ensure that localization estimates and IMU messages are
available as these are required for IMU bias pose estimation. 
Record the results to a bagfile and plot using the `plot_results.py` script in 
localization_analysis.  See the localization_analysis readme for more details on plotting results.

# Inputs
* `/hw/imu`
* `graph_loc/state`

# Outputs
* `imu_bias_tester/pose`
