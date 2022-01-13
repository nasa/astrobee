\page performance_tester Performance Tester

## Tools

### Test Plan

This tool can be used to test a plan execution. It executes a startup fplan, starts recording,
and starts the plan execution. The recording stops when the plan is completed. This is useful for
granite lab testing.
To execute the tool, run:

	rosrun performance_tester test_plan -startup_plan $STARTUP_FPLAN -data_to_disk $DATA_PROFILE -move_plan $MOVE_FPLAN

## Scripts

### Analyse bag

This script analyses bag files and plots several output plots for analysis.
It analyzes the CPU and Memory used, as well as the rate of publishing of topics.
To run the script for a certain bag file:

	./analyse_bag.py --bag-name $BAG_NAME

By default the topics that is scoped for frequency analysis are: "/loc/ml/features", "/hw/imu", "/gnc/ekf", "/gnc/ctl/command".
To customize this, the argument '--topic-list' can be defined when executing the script to specify the topics.