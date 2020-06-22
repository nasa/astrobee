\page simwrapper GNC Simulation Wrapper

This package wraps around the GNC simulator functionality.
Currently, the things it simulates are very limited, to
the IMU, sparse mapping localization, optical flow, handrail detection,
and the robot's motion.

# Required Inputs

* `/ctl/fam`
* `/pmc_actuator/command`

# Simulated Outputs

* `/imu/data`
* `/ctl/vpp_state`
* `/pmc_actuator/telemetry`
* `/clock`
* `/localization/mapped_landmarks/registration`
* `/localization/mapped_landmarks/features`
* `/localization/optical_flow/features`
* `/localization/optical_flow/registration`
* `/localization/handrail/features`
* `/localization/handrail/registration`
* `/ground_truth`

