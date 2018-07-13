\defgroup mobility Mobility

# Overview

The overarching objective of the mobility subsystem is to provide a clean interface for moving the Astrobee in such a way that avoids obstacles, adheres to kinematic constraints, and keeps within specified flight zones.

It is necessary to first define a few terms that will be used throughout the rest of this document. A FreeFlyer **plan** comprises of a number of stations (static poses) connected by **segments** (motion). The executive subsystem splits a plan and feeds the resulting segments one by one to mobility.

A segment is a time-ordered sequence of **setpoints** and a corresponding **flight mode**. Each setpoint is a vector containing a pose and its first and second time derivatives, or equivalently a second order curve in 6DoF that describes motion. The flight mode is a string ("nominal", "docking", etc.) that is used to reconfigure the system (set impeller speed, controller gains, planning limits, tolerance values, etc.) so that consistency is enforced. When integrated forward in time by the controller, the setpoint carves out a continous curve in 3D space, defined over the time interval that runs from the current setpoint to the next setpoint, which we call a **trajectory**.

![alt text](../images/mobility/definitions.png "Core nomenclature")

# Basic operation

The mobility subsystem provides the callee with a single action-based entry point for control, called ```ff_msgs::MotionAction```. This "motion action" accepts one of the following five commands:

1. PREP - Change the flight mode to some new value. Common values include "off", nominal", "quiet", "difficult". If the new mode has a impeller speed that is different from the current mode, the prep command will ramp the impeller up or down and wait for it to reach its goal before returning.
2. MOVE - Move the robot through a given set of poses.
3. EXEC - Execure a specific segment.
4. IDLE - Null the linear and angular velocities. The robot will try and stop moving, but will not necesserily hold its position.
5. STOP - Attempt to hold the current position. If the robot is manually moved when in stopped mode, it will detect this and try and hold the new position.

The action supports only one goal at a time, which is fully-preemptible. This means that a new goal always preempts the current goal (the previous goal's callee will be notified of preemption). Consequently, you must be careful not to interact with the otion action while active.

The mobility subsystem is comprised of four different nodes, each havin

* `Choreographer` - The choreographer is the central point of entry to the mobility subsystem. In addition to offering the motion action, it also provides the ability to set keep-in and keep-out zones and inertial properties.

* `Mapper` -  As its name suggests, the role of the mapper is to maintain a map of environmental clutter. It provides hooks for the choreographer to validate trajectories, and for planners to obtain clutter maps.

* `Trapezoidal planner` - The trapezoidal planner is a simple "straight line" planner that assumes the acceleration and velocity on each of the supplied poses is zero. It is unaware of clutter, and is therefore typically used for short movesm, notably in docking and perching.

* `Quartic Polynominal (QP) planner` -  The QP planner is more complex planner that generates curved segments that avoid obstacles. Internally it represents the segment by a set of polynomial functions, which can be sampled at any desired rate to generate setpoints.

The diagram below illustrates how the various ndoes in the system interact with each other. ROS ctions are drawn in red, while ROS services are drawn in blue. Nodes that are part of the mobility subsystem are drawn as black boxes, while external nodes are drawn as white boxes.

![alt text](../images/mobility/overview.png "Interaction between mobility modules")

# Using the mobility subsystem

Typically, mobility will be controlled through the Ground Data System (GDS). However, The arm behavior is packaged with its own gflags-based tool called teleop. This tool is essentially a convenience wrapper around an action client can be used to control the arm from the command-line.

    rosrun mobility teleop -helpshort

Here are some examples of how to use the tool. You will need an active simulator to run these examples.

To switch to AR tag localization:

    rosrun mobility teleop -loc ar

To move to coordinate X=0.3 Y=0.4, keeping the Z and rotation the same:

    rosrun mobility -move -pos "0.3 0.4"

To rotate around Z (axis X=0 Y=0 Z=1) by -1.5 radians:

    rosrun mobility -move -att "-1.5 0 0 1"

To switch to AR target localization, use the docking flight mode and move:

    rosrun mobility -loc ar -mode docking -move -pos "0 0"

To use the QP planner to move to X=0.6 Y=0.6:

    rosrun mobility -move -pos "0.6 0.6" -planner qp

At any point one can inspect the internal state of the mobility subsystem using the following command. The command will return a sequence of numbers, which represent a time ordered sequence of states. Please refer to ```ff_msgs::MotionState``` for a mapping from numbers to states.

    rostopic echo /mob/motion/state

If you ever need to manually set the motion state to a specific value, you can call the ```set_state``` service with the new state as the single argument.

    rosservice call /mob/motion/set_state 1

# Generating acceleration profiles

The plangen tool can be used to query a planner directly, obtain a segment as a response, and write the time-ordered acceleration profile to file. This is useful for the MGTF facility, which requires an acceleration profile as input for controlling the gantry and gimbal.

First, create an input file (eg. input.txt) with timestamped poses. If you put zeros for timestamps the planner will try and do the action as quickly as possible, given the kinematic constraints (the linear and angular velocity and acceleration limits for the current flight mode). For example, the pose sequence below is 60 seconds long, starts at x = 1.0, y = -1.0, z = 1.0 with identity rotation, then does a yaw of 90 degrees and move to x = -1.0, y = 1.0, z = 1.0. Each row is of the form [t x y z q_x q_y q_z q_w], where q = [q_x q_y q_z q_w] is an attitude quaternion.

Here is an example of a suitable input file:

    0.0   1.0  -1.0  1.0  0.0  0.0     0.0  1.0
    20.0 -1.0  1.0  1.0  0.0  0.7071  0.0  0.7071
    40.0 -1.0 -1.0  1.0  0.0  0.0     0.0  1.0
    60.0  1.0 -1.0  1.0  0.0  0.7071  0.0  0.7071

Then, run an ISS simulation and wait a couple of seconds for the system to start:

    roslaunch astrobee sim.launch

Now query the planner using my tool and the input pose sequence you created:

    rosrun mobility plangen -input /path/to/input.txt -output /path/to/output.csv

If everything works correctly you should see an output.csv created, which can be fed directly into the MGTF. The output file is really just a time-indexed sequence of accelerations that yield velocity trapezoids that move between the poses you supplied. To control the velocity and acceleration of the gantry you can use the following switches to plangen:

    vel  : desired linear velocity in m/s
    accel : desired linear acceleration in m/s^2
    omega : desired angular velocity in rads/s
    alpha : desired angular acceleration in rads/s^2
    ff : force the robot to always face in the direction of motion

For example this limits the net angular velocity to 10mm per second:

    rosrun mobility plangen -input /path/to/input.txt -output /path/to/output.csv -vel 0.01

There are other options, which might be useful. To see them use -helpshort.
