\page ctl Control (CTL)

The control subsystem takes the pose from the EKF, as well as control commands, to
determine the forces and torques that should be applied to the robot for smooth control.
If executes the given segment based on a trajectory given by the choreographer, the faul
checking on the execution is done in the choreographer based on the feedback.

# Inputs

* `gnc/ekf`: EKF State from EKF
* `gnc/ctl/control` Action. See the  [Control](@ref ff_msgs_Control) action specification for details.

# Outputs

* `gnc/ctl/command`: The force and torque commanded by control.
* `gnc/ctl/shaper`: The output from the GNC command shaper, which smooths the control.
* `gnc/ctl/traj`: The trajectory that the control subsystem is being commanded to follow.
* `gnc/ctl/segment`: The current segment the control subsystem is traversing.
* `gnc/ctl/progress`: The progress in executing the current segment.

# GNC Control Wrapper behavior

\dotfile ctl_fsm "GNC Control Wrapper finite state machine" width=10cm

* WAITING: No setpoints sent to the internal Matlab controller.
* NOMINAL: Waits until the start time for deferred executions, commands the setpoints to the internal Matlab in nominal mode until the desired trajectory is finished.
* STOPPING: Commands internal Matlab into stopping mode. When the velocity and angular velocity fall under a certain stop threshould, it succeeds.


# Autocode Matlab Internal behavior

The internal Matlab behavior has 4 modes:
* Idle: No control is present.
* Stopping: Position error is calculated based on the delayed current position and the current position. Velocity and acceleration commands are zero.
* Nominal: It follows the trajectory given for position, velocity and acceleraion.
* Stopped: Position/Attitude error is zero. Control Velocity and acceleration goals are zero.


# Enable/Disable Control

In the case a new controller is to be tested, it is possible to enable/disble the current
controller using the service `gnc/ctl/enable`. `true` will enable the control and `false`
will disable it. If the requested state is the one already active, then the service does
not return success.