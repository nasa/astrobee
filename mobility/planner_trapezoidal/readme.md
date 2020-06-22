\page planner_trapezoidal Trapezoidal planner

The trapezoidal planner provides a baseline mechanism for generating **segments** that pass through a supplied sequence of poses. It constructs linear translations between poses, with a trapezoidal ramp on angular and linear velocity. This planner is completely agnostic to clutter and keep-out zones, and so it is typically used for short moves, like those in docking or perching.

Astrobee is a **holonomic** vehicle, and is therefore capable of both rotating ans translating simultaneously when moving from one pose to another. However, there are several compelling reasons why one would want to opt-out of this behaviour in favor of a **face-forward** mode. In this mode Astrobee is oriented to face along the direction of translation.

1. Face-forward mode always ensures that during translations the hazard detection camera always points in the direction of motion. This increases the likelihood of the robot "seeing" any obstacle in the desired path.
2. The robot's acceleration is fundamentally constrained by a hard limit on how much pressure the impellers can build in the plenum. During holomnic moves this energy mst be spread along six degrees of freedom. This lowers the angular and linear acceleration limits.
3. It is easier for an astronaut or operator to determine the intent of the freeflyer when it nominally moves in the direction it faces.

# Basic operation

Describing how the trapezoidal planner works requires the definition of several input parameters:

* Hard limits - These are the maximum linear and angular velocity and acceleration magnitudes that the propulsion system can safely achieve, and is dictated by the current flight mode. It is calculated a priori and depends on the impeller speeds, controller gains and inertial properties of the platform.
* Soft limits - These are customizable limits on the linear and angular velocities and acceleration magnitudes. It allows the use to specify target values that are lower than the hard upper limits.
* Time limits - As part of the request the callee provides timestamps with each desired pose, which implictly encodes a "time limit" to get from one target pose to the next. The trapezoidal planner will try and choose a linear and angular velocity in order to satisfy this time limit. If this is not possible (the soft or hard limits are violated) then the soft limits are used, which lengthens the duration of segment.

The limit precedence is therefore the following:

  hard limits > soft limits > time limits

The planner take in as an argument a time-indexed sequence of poses to move through. Using the limit hierarchiy above it outputs a fully-defined segment containing a sequence of setpoints, each specifying a postion, orientation, velocity, angular velocity, acceleration and angular acceleration.

## Holonomic moves

We'll begin by considering a holonomic move, which is an a sense the generalization of a face-forward move. A face-forward move follows the same core construction logic except that it structures the move as a rotation-translation-rotation sequence, and in so doing yields a segment in which rotation and translation is never performed simultaneously.

The first challenge with a holomic move is to determine whether of the linear or angular components ```dominates```. That is, we need to know whether it will take longer to rotate or translate based on our set of hard/soft/time limits. Once the dominating axis and duration is known, we select a lower limit for the other axis so that its motion is "spread" to take the same time as the dominating axis. Not doing this leads to counter-intuitive moves, like rotation completing several seconds before the translation does.

Once the dominating duration is known, the problem reduces to finding separate velocity profiles for the linear and angular components of the move. The sequence of setpoints is produced by sampling the profiles. For the less intuitive case of orientation, integrating the angular velocity profile yields the angle (magnitude) component of an angle-axis orientation representation, which we can then convert to a quaternion for use in a setpoint.

The one subtle detail that one must be careful to account for is the fact that the velocity profile may in fact be a pyramid and not a trapezoid. This happens when the angle or distance moved is short and the acceleration limit is low. In this case the axis never ramps up to its velocity limit before needing to ramp down to stop at the correct point. Equivalently, it never reaches cruising speed and therefore it's cruise duration is zero.

## Face-forward moves

The same planning algorithm is used for face-forward mode, except the motion is split into three distinct phases - (1) look, (2) translate, (3) rotate.

(1) In the look-phase the freeflyer performs a rotation to align its forward direction (body-frame X) with the vector pointing in the direction it intends to translates. There is one remaining degree of freedom that remains unconstrained (roll, or rotation about body-frame x). We select this value in such a way that the body-frame y axis lies in the world X-Y plane. This nominally keeps the freeflyer oriented so that its body-frame Z never inverts with respect the world Z. This works for all motions except pure downward translations. In this case we align body-frame Y with the Y-Z plane.

(2) In the translate phase the robot moves from its initial to final position without any orientation change. Since the final orientation of the look phase pointed in the direction of translation the robot always "looks forward".

(2) In the rotate phase of a face-forward move the robot reorients itself to be pointed in the direction of the end pose. 

# Usage

Although one could theoretically interact with the trapezoidal planner, this is not advised. To generate and execute a trapezoidal plan one should use the move command in conjunction with the "trapezoidal" planner flag. Here is an example command that shows how to move to the origin of the world:

    rosrun mobility teleop -move -pos "0 0 0" -att "0" -planner trapezoidal

By default the teleop tool configures the platform to operate in holomonic mode. To enforce face-forward mode the "-ff" flag can be used. For example:

    rosrun mobility teleop -move -pos "0 0 0" -att "0" -planner trapezoidal -ff
