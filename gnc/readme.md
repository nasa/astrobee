\page gnc Guidance, Navigation & Control

The Guidance, Navigation & Control subsystem is responsible for controlling the robot's motion.

## GNC Subsystems

GNC consists of three main subsystems:

1. \subpage ctl : The control subsystem takes the robot's current state from the localizer,
and determines the forces and torques needed to execute the commands sent from
the mobility subsystem.

2. \subpage fam : The force allocation module takes the force and torque commands
from the control subsystem, and determines the PMC speeds and nozzle positions
to best execute these forces and torques. This is a ROS package that calls the
main functionality in the pmc module which does not depend on ROS.

3. \subpage pmc: This module implements the main functionality of the FAM. It also
simulates the PMC, converting PMC speeds and nozzle positions to forces and torques
(the inverse of the force allocation module), used only in simulation.
