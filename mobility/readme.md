\defgroup mobility Mobility 
 
The overarching objective of the mobility subsystem is to provide a clean interface for moving the FreeFlyer platform in such a way that avoids obstacles, adheres to kinematic constraints, and keeps within specified flight zones.

# Segments, setpoints, trajectories and chunks

It is necessary to first define a few terms that will be used throughout the rest of this document. A FreeFlyer **plan** comprises of a number of stations (static poses) connected by **segments** (motion). The executive subsystem operates at the plan level, while the mobility subsystem operates at the segment level.

A segment is a time-ordered sequence of **setpoints** and a corresponding **flight mode**. Each setpoint is a vector containing a pose and its first and second time derivatives, or equivalently a second order curve in 3D space that describes motion. The flight mode is a string ("nominal", "docking", etc.) that is used by GNC to determine how to configure the controller's gains and tolerances during exection. The part of a segment between two setpoints is called a **chunk**.

Most interaction with the mobility subsystem reduces to a segment being fed to GNC for execution; the exceptions being for stopping or idling. Internally, GNC processes one setpoint at a time, choosing when to switch based on the setpoint timestamps. Each setpoint describes a *trajectory* that is tangent to the segment at the given timestamp. It is this trajectory that GNC follows until the next setpoint is loaded.

![alt text](../images/mobility/definitions.png "Plans, segments, setpoints and trajectories")

# Using the mobility subsystem

Interaction with the subsystem typically occurs through the folloing five actions available on the /mobility/choreographer namespace:

* `Execute` - Validate and then execute a precomputed segment. This will normally be a segment computed using the Ground Station, and passed in by executive.
* `Move` - Plan and execute a path through a sequence of poses. The poses are passed to a planner implementation, and then validated for correctness before being executed.
* `Stop` - Stop any current motion. The exact stopping point will by unknown. However, the FreeFlyer can be manually repositioned, and it will hold that new position.
* `Idle` - Null all the rates. Should only be used if stop fails to work.

You interact with mobility using the `teleop` tool. To see what options the tool offers type the following:

    rosrun mobility teleop -helpshort

Here is what syou should see

    teleop: Usage: rosrun mobility teleop <opts>

    Flags from teleop.cc:
    -accel (Desired acceleration) type: double default: -1
    -active (Action active timeout) type: double default: 10
    -alpha (Desired angular acceleration) type: double default: -1
    -att (Desired attitude in axis-angle format X Y Z angle (radians))
      type: string default: ""
    -connect (Action connect timeout) type: double default: 10
    -deadline (Action deadline timeout) type: double default: 60
    -exec (Execute a given segment) type: string default: ""
    -ff (Plan in face-forward mode) type: bool default: false
    -idle (Send idle command) type: bool default: false
    -loc (Localization pipeline (none, ml, ar, hr)) type: string default: ""
    -mode (Flight mode) type: string default: "nominal"
    -move (Send move command) type: bool default: false
    -nobootstrap (Don't move to the starting station on execute) type: bool
      default: false
    -nocollision (Don't check for collisions during action) type: bool
      default: false
    -novalidate (Don't validate the segment before running) type: bool
      default: false
    -ns (Robot namespace) type: string default: ""
    -omega (Desired angular velocity) type: double default: -1
    -planner (Path planning algorithm) type: string default: "trapezoidal"
    -pos (Desired position in cartesian format X Y Z (meters)) type: string
      default: ""
    -rate (Segment sampling rate) type: double default: 1
    -rec (Plan and record to this file. Don't execute.) type: string
      default: ""
    -response (Action response timeout) type: double default: 10
    -stop (Send stop command) type: bool default: false
    -vel (Desired velocity) type: double default: -1
    -wait (Defer move by given amount in seconds (needs -noimmediate))
      type: double default: 0

Here are some examples of how to use the tool. You will need an active simulator to run these examples.

To switch to AR tag localization:

    rosrun mobility teleop -loc ar

To move to coordinate X=0.3 Y=0.4, keeping the Z and rotation the same:

    rosrun mobility -move -pos "0.3 0.4"

To rotate around Z (axis X=0 Y=0 Z=1) by -1.5 radians:

    rosrun mobility -move -att "0 0 1 1.5"

To switch to AR target localization, use the docking flight mode and move:

    rosrun mobility -loc ar -mode docking -move -pos "0 0"

To use the QP planner to move to X=0.6 Y=0.6:

    rosrun mobility -move -pos "0.6 0.6" -planner qp

Please note that all configuration persists. So, if you use the `-planner qp` flag in a teleop, all future invocations of the teleop tool will use the QP planner unless you excplicity ask provide `-planner trapezoidal`.


# Interaction between the nodes in mobility

The interfaces are based on the ROS actionlib framework, which essentially implements connection-oriented, non-blocking, asynchronous service calls with periodic feedback.

* `Choreographer` - The choreographer sits at the heart of the mobility subsystem, and ensures that motion requests are serviced, segment generation and/or validation takes place, and that the resulting segment is passed to Guidance, Navigation & Control for execution. It also activates the sentinel in-motion, and responds to any collision/tolerance notifications.

* `Sentinel` - The role of the sentinel is to continually check for upcoming collisions, and that the error between the estimated and the goal state remains within some tolerance threshold.

* `Mapper` -  As its name suggests, the role of the mapper is to maintain a map of the environment. The layered map comprises of keep-in and keep-out zones -- specified by the ground operator -- and obstacles detected by one or more hazard cameras.

* `Planner` - The planner offers two actions, the first of which is "validate action that takes a segment and checks it. The planner is a proxy that passes planning requests from the choreographer to a specific planner implementation. The resulting "generated" segments are checked for validity by the planner, before being returned to the choreographer.

In addition to the four nodes above, the mobility subsystem also includes the following two reference planner implementations:

* `Trapezoidal` - The trapezoidal planner is a simple "straight line" planner that assumes the acceleration and velocity on each of the supplied poses is zero. It constructs a trapezoidal rate ramp for each axis of control, taking into account hard and soft kinematic limits. If face-forward mode is activated, the platform is rotated into and out of the direction vector between poses with the goal of translation always being in positive body-frame X.

* `QP` -  The Quartic Polynominal planner is more complex planner, which generates curved segments that, in addition to obeying hard and soft kinematic constraints, also obeys keep-ins and keep-outs and avoids obstacles. Internally it represents the segment by a set of polynomial functions, which can be sampled at any desired rate to generate setpoints.

The diagram below illustrates how the various ndoes in the system interact with each other. ROS ctions are drawn in red, while ROS services are drawn in blue. Nodes that are part of the mobility subsystem are drawn as black boxes, while external nodes are drawn as white boxes.

![alt text](../images/mobility/overview.png "Interaction between mobility modules")