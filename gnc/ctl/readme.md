\page ctl GNC Control Wrapper

The control subsystem takes the pose from the EKF, as well as control commands, to determine
the forces and torques that should be applied to the robot for smooth control.

# Inputs

* `gnc/ekf`: EKF State from EKF
* `gnc/ctl/control` Action. See the  [Control](@ref ff_msgs_Control) action specification for details.

# Outputs

* `gnc/ctl/command`: The force and torque commanded by control.
* `gnc/ctl/shaper`: The output from the GNC command shaper, which smooths the control.
* `gnc/ctl/traj`: The trajectory that the control subsystem is being commanded to follow.
* `gnc/ctl/segment`: The current segment the control subsystem is traversing.
* `gnc/ctl/progress`: The progress in executing the current segment.

