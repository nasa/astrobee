\page simwrapper GNC Simulation Wrapper

This package wraps around the GNC simulator functionality.
Currently, the things it simulates are very limited, to
the IMU, sparse mapping localization, optical flow, handrail detection,
and the robot's motion.

# Required Inputs

* `hw/pmc/command`

# Simulated Outputs

* `hw/imu`
* `hw/pmc/telemetry`
* `/clock`
* `hw/cam_nav`
* `loc/ml/registration`
* `loc/ml/features`
* `loc/of/features`
* `loc/of/registration`
* `loc/hr/features`
* `loc/hr/registration`
* `loc/truth/pose`

