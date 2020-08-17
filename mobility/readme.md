\page mobility Mobility

## Overview

The overarching objective of the mobility subsystem is to provide a clean interface for moving the Astrobee in such a way that avoids obstacles, adheres to kinematic constraints, and keeps within specified flight zones.

It is necessary to first define a few terms that will be used throughout the rest of this document. A FreeFlyer **plan** comprises of a number of stations (static poses) connected by **segments** (motion). The executive subsystem splits a plan and feeds the resulting segments one by one to mobility.

A segment is a time-ordered sequence of **setpoints** and a corresponding **flight mode**. Each setpoint is a vector containing a pose and its first and second time derivatives, or equivalently a second order curve in 6DoF that describes motion. The flight mode is a string ("nominal", "docking", etc.) that is used to reconfigure the system (set impeller speed, controller gains, planning limits, tolerance values, etc.) so that consistency is enforced. When integrated forward in time by the controller, the setpoint carves out a continous curve in 3D space, defined over the time interval that runs from the current setpoint to the next setpoint, which we call a **trajectory**.

![alt text](../images/mobility/definitions.png "Core nomenclature")

## Basic operation

The mobility subsystem provides the callee with a single action-based entry point for control, called ```ff_msgs::MotionAction```. This "motion action" accepts one of the following five commands:

1. PREP - Change the flight mode to some new value. Common values include "off", nominal", "quiet", "difficult". If the new mode has a impeller speed that is different from the current mode, the prep command will ramp the impeller up or down and wait for it to reach its goal before returning.
2. MOVE - Move the robot through a given set of poses.
3. EXEC - Execure a specific segment.
4. IDLE - Null the linear and angular velocities. The robot will try and stop moving, but will not necesserily hold its position.
5. STOP - Attempt to hold the current position. If the robot is manually moved when in stopped mode, it will detect this and try and hold the new position.

The action supports only one goal at a time, which is fully-preemptible. This means that a new goal always preempts the current goal (the previous goal's callee will be notified of preemption). Consequently, you must be careful not to interact with the otion action while active.

The mobility subsystem is comprised of four different nodes.

* `Choreographer` - The choreographer is the central point of entry to the mobility subsystem. In addition to offering the motion action, it also provides the ability to set keep-in and keep-out zones and inertial properties.

* `Mapper` -  As its name suggests, the role of the mapper is to maintain a map of environmental clutter. It provides hooks for the choreographer to validate trajectories, and for planners to obtain clutter maps.

* `Trapezoidal planner` - The trapezoidal planner is a simple "straight line" planner that assumes the acceleration and velocity on each of the supplied poses is zero. It is unaware of clutter, and is therefore typically used for short movesm, notably in docking and perching.

* `Quadratic Program (QP) planner` -  The QP planner is more complex planner that generates curved segments that avoid obstacles. Internally it represents the segment by a set of polynomial functions, which can be sampled at any desired rate to generate setpoints.

The diagram below illustrates how the various ndoes in the system interact with each other. ROS ctions are drawn in red, while ROS services are drawn in blue. Nodes that are part of the mobility subsystem are drawn as black boxes, while external nodes are drawn as white boxes.

![alt text](../images/mobility/mob_overview.png "Interaction between mobility modules")

## Using the mobility subsystem

Typically, mobility will be controlled through the Ground Data System (GDS). However, The arm behavior is packaged with its own gflags-based tool called teleop. This tool is essentially a convenience wrapper around an action client can be used to control the arm from the command-line.

    rosrun mobility teleop -helpshort

Here are some examples of how to use the tool. You will need an active simulator to run these examples.

To switch to AR tag localization:

    rosrun mobility teleop -loc ar

To move to coordinate X=0.3 Y=0.4, keeping the Z and rotation the same:

    rosrun mobility teleop -move -pos "0.3 0.4"

To rotate around Z (axis X=0 Y=0 Z=1) by -1.5 radians:

    rosrun mobility teleop -move -att "-1.5 0 0 1"

To switch to AR target localization, use the docking flight mode and move:

    rosrun mobility teleop -loc ar -mode docking -move -pos "0 0"

To use the QP planner to move to X=0.6 Y=0.6:

    rosrun mobility teleop -move -pos "0.6 0.6" -planner qp

At any point one can inspect the internal state of the mobility subsystem using the following command. The command will return a sequence of numbers, which represent a time ordered sequence of states. Please refer to ```ff_msgs::MotionState``` for a mapping from numbers to states.

    rostopic echo /mob/motion/state

If you ever need to manually set the motion state to a specific value, you can call the ```set_state``` service with the new state as the single argument.

    rosservice call /mob/motion/set_state 1

##  Generating a plan from a sequence of poses.

Given a list of poses (positions in meters, and angles in degrees) the
plangen tool can use the trapezoidal planner to create planned
trajectory that goes between these poses.

This tool does not enforce the robot following this plan to always
face forward. If that is desired, the input poses should be created
accordingly.

Since the planner will take the shortest path among two poses, to
avoid ambiguity it is suggested that the angles do not change by more
than 90 degrees between poses.

Here is an example list of poses in the Kibo module, written as a file
named input.txt.

# x    y      z  roll pitch yaw (degrees)
10.93 -9.2  4.85  0    0    90
10.93 -6.2  4.85  0    0     0

Here the robot will start looking down the length of the module (the
largest dimension, which is the Y axis), and move by 3 meters. During
that time, the robot's orientation will change from forward to
sideways.

Here is another example:

### 360 degree yaw rotation at a 45 degree pitch.
### x    y      z  roll pitch   yaw (degrees)
10.93 -9.2  4.85   0     0     90 # face forward
10.93 -9.2  4.85   0    45     90 # pitch up at 45 degrees
10.93 -9.2  4.85   0    45    180 # yaw rotation
10.93 -9.2  4.85   0    45    270 # more yaw rotation
10.93 -9.2  4.85   0    45    360 # more yaw rotation
10.93 -9.2  4.85   0    45     90 # back to the original yaw

The planner tool can be invoked as follows:

astrobee_build/native/devel/lib/mobility/plangen -input input.txt \
 -output output.fplan -vel 0.2 -accel 0.017 -omega 0.17 -alpha 0.2

It will write the file output.fplan that can be loaded and tested in GDS.

The input options are:

    -vel   : maximum desired linear velocity in m/s
    -accel : maximum desired linear acceleration in m/s^2
    -omega : maximum desired angular velocity in rads/s
    -alpha : maximum desired angular acceleration in rads/s^2

It is very important to note that the order in which the the roll,
pitch, and yaw rotations are multiplied gives rise to different
rotation matrices. This tool supports the option

  -rotations_multiplication_order

The default value is 'yaw-pitch-roll', hence the rotations matrices
with these angles are multiplied as

  rotation(yaw) * rotation(pitch) * rotation(roll)

As such, the roll rotation happens first, followed by the pitch
rotation, and followed by the yaw rotation. This option also supports
the value 'roll-pitch-yaw' when the order above is reversed. In total,
there are six ways of multiplying these rotation matrices.

Other options can be seen by invoking the -help option.

## Creating acceleration profiles.

If plangen is invoked with --output-type csv, it will create
acceleration profiles that could be used to control the gantry in
MGTF.

\subpage choreographer
\subpage framestore
\subpage mapper
\subpage planner_qp
\subpage planner_trapezoidal