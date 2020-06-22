\page gnc Guidance, Navigation & Control

The Guidance, Navigation & Control subsystem is responsible for tracking the
robot's pose and controlling the robot's motion.

# GNC Subsystems

GNC consists of four main subsystems, that run in a chain:

\subpage ekf
1. `EKF`: The extended Kalman filter integrates IMU measurements, as well as
visual measurements from the localization subsystem, to track the robot's pose.
\subpage ctl
2. `CTL`: The control subsystem takes the robot's current state from the EKF,
and determines the forces and torques needed to execute the commands sent from
the mobility subsystem.
\subpage fam
3. `FAM`: The force allocation module takes the force and torque commands
from the control subsystem, and determines the PMC speeds and nozzle positions
to best execute these forces and torques.
\subpage
4. `SIM`: The simulator simulates the inputs to the EKF, and simulates the
robot's motion based on the outputs of the FAM.

# GNC Code Organization

Unlike the rest of flight software, the GNC systems are written mainly in
Simulink. Simulink drag and drop box diagrams are then converted to
automatically generated C code. The structure of the GNC code is:

1. [EKF](@ref ekf), [CTL](@ref ctl) and [FAM](@ref fam) are GN&C ROS Wrapper.
[Sim Wrapper](@ref simwrapper) is the model side of the GN&C ROS Wrapper.
These folders contain ROS nodelets that mainly convert inputs and outputs
between ROS messages and Simulink.
\subpage gncautocode
2. [GNC Autocode](@ref gncautocode): This folder contains a thin C++ wrapper
around the auto-generated C functions, which are all compiled in this package
into a library.
\subpage matlab
\subpage simwrapper
3. [Matlab](@ref matlab): This folder contains the Matlab / Simulink code. This
is where all the  core functionality is located.
