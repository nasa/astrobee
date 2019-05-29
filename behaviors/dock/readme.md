\defgroup dock Dock Behavior
\ingroup beh

# Overview

Astrobee is upmassed to the ISS along with a smart docking station, which we refer to as "the dock". The astrobee robot attaches to the dock via one of two points (berth 1 and berth 2). Each berth provides a matrix of pogo pins, which mates with a matching pad on the aft-side of the robot to provide connectivity and charging. There is only one correct mating orientation. A pair of cones help guide the freeflyer into the correct docking position, and a mild magnetic force holds it in position until an undock command is issues. 

Docking is a precision operation that requires sub-centimeter localization accuracy. Standard mapped landmark localization -- our default in-flight -- cannot guarantee this performace. For this reason we use a different mechanism technique here we localize ourselves relative to a set of visual markers, which we refer to as the "AR Target". This target is positioned between the berths, and designed to be in view of the aft-facing docking camera as we approach or leave the berth. Incidentally, this camera is offset to one side, which means that the correct mating orientation is reversed on berth 2. This means that the robot must invert itself with respect to the dock (carry out a 180 degree roll) prior to engaging with berth 2.

The docking behavior is fairly complex and requires the existence of several other nodes in the system to work correctly. Specifically, it uses the ```ff_msgs::SwitchAction``` action to switch between localization system, and the ```ff_msgs::MotionAction``` to move into and out from the dock, and to turn the propulsion system. It also uses messages and services offered by the eps_driver to determine whether it has mated sucessfully with the dock, and to initiate the undock process (magnetic).

Docking requires the dock location to be specified a priori. The dock origin is aligned with the midpoint of the AR target. The berth frame is fixed with respect the dock frame but may be on a 50 degree tilt. All of these values are configurable in the world config file. The framestore node assembles a static transform hierarchy in TF2, which the dock behavior uses to calculate two berth-specific positions -- the "approach point" and the "complete point". The complete point is the desired pose of the freeflyer body in the world frame when on dock, while the approach point is a pose offset slightly from the complete point in such a way that the robot docks and undocks along a specific trajectory, thereby maximizing the change of success.

# Internal behavior

The dock behavior provides the callee with a single action-based entry point for control, called ```ff_msgs::DockAction```. This "dock action" accepts a command -- ether DOCK or UNDOCK -- and executes it in the background, sending periodic feedback to the callee. When the task completes a result is returned, along with a response code that captures any errors. Unlike services, actions are non-blocking so exhibit superior performance in nodelet contexts.

The action supports only one goal, which is fully-preemptible. This means that a new goal always preempts the current goal (the previous goal's callee will be notified of preemption). Consequently, you must be careful not to interact with docking action, or any of its dependencies while active.

Internally, the dock behavior is encoded as a finite state machine depicted below and implemented using the ff_util/FSM class. This class essentially captures a map of (state, event) -> lambda function relationships. The state of the system is moved forward by setting the initial state and then pushing a sequence of events to the FSM.

\dotfile dock_fsm "Dock node finite state machine"

Broadly speaking, there are two nominal sequences -- a dock sequence and an undock sequence. The recovery sequences branch off of these, ensuring that the robot returns to a safe location if an error occurs. The recovery strategy is different for docking and undocking, and depend on where the failure occurs. 

A high-level overview of docking is straightforward to describe. The actual implementation for both sequences is substantially more complex. For example, the state machine must aynchronously wait for every action to complete before proceeding to the next stage.

This is the nominal docking sequence:

1. Switch to mapped landmark localization
2. Move to the approach point in nominal flight mode
3. Switch to marker tracking localization
4. Move to the complete point in docking flight mode
5. Check for attachment (try to move back to approach point, expecting failure)
5. Turn off localization
6. Turn off propulsion

If any of the stages 1 - 5 above fail, then the recovery sequence involves returning to the approach point / undocked state. If stages 5 or 6 fail, then we move to a docked state. In all cases the error is reported in the feedback.

This is the nominal undock sequence:

1. Turn on propulsion
2. Turn on mapped landmark localization
3. Call the undock service provided by EPS to disable magnetic retention.
4. Move to the approach point in undocking flight mode

If any of the stages 1 - 3 above fail, then the recovery sequence involves staying on the dock in a docked state. If stage 4 fails, the system tries again to move to the approach point / an undocked state.

Some extra information:

* The behavior continually monitors the EPS dock state in the background to detect when the robot is manually docked or undocked by an astronaut. It updates its internal state in response to these changes, but never automatically updates localization or propulsion. For example, if the robot is manually removed from the dock its propulsion doesn't automatically turn on.
* A safeguard is in place to prevent the operator accidentally docking from any point on the space station. The robot must be within some threshold distance of the approach point (currently 2 meters) for docking to start.
* Propulsion is turned on and off using the prep command as part of the motion action. A prep to flight mode "off" is the same as disabling propulsion, except that it asynchronously notifies the callee when the motors have spun down completely. This provides a more controlled shutdown mecahnism. This flight mode also disables the controller from running.
* Localization is turned off using a special localization mode called "none". This effectively disables all cameras, and stops the EKF from running.
* The dock can only supply a limited amount of current to the robot, and it might be insufficent to support charging batteries and turning on the PMCs. For this reason one should initiate an undock only when the batteries are full.

# Using the test tool

Typically, docking and undocking will be controlled through the Ground Data System (GDS). However, The dock behavior is packaged with its own gflags-based tool called dock_tool. This tool is essentially a convenience wrapper around an action client can be used to dock and undock from the command-line.

To get a list of supported commands, have a look at the help:

    rosrun dock dock_tool -helpshort

For example, lets assume that we have started the robot in free-flight, and it is currenty positioned within two meters from the dock. This command will attempt a dock to berth 1:

    rosrun dock dock_tool -dock -berth 1

Alternatively, one can dock to berth 2 using the following command. Although, note that in the granite world this is not possible, because the robot cannot roll by 180 degrees to move to the required berth 2 approach point.

    rosrun dock dock_tool -dock -berth 2

Once docked, undocking can be initiated using the following command. You need not specify the berth, as it is automatically detected based on the robot pose.

    rosrun dock dock_tool -undock

At any point one can determine the internal state of the dock behavior using the following command. The command will return a sequence of numbers, which represent a time ordered sequence of states. Please refer to ```ff_msgs::DockState``` for a mapping from numbers to states.

    rostopic echo /beh/dock/state

If you ever need to manually set the dock state to a speficic value, you can call the ```set_state``` service with the new state as the single argument:

    rosservice call /beh/dock/set_state -4