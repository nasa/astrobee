\defgroup perch Perch behavior
\ingroup beh

Astrobee may be optionally equipped with a **perching arm** payload, which provides the robot with a three degrees of freedom arm. This enables Astrobee to perch and unperch from handrails on the ISS, alowing it to turn off localization and propulsion thereby saving considerable energy. When perched the arm provides two degrees of freedom, allowing the operator to pan and tilt the robot as required.

# Internal behavior

The perch behavior provides the callee with a single action-based entry point for control, called ```ff_msgs::PerchAction```. This "perch action" accepts a command -- ether PERCH or NPERCH -- and executes it in the background, sending periodic feedback to the callee. When the task completes a result is returned, along with a response code that captures any errors. Unlike services, actions are non-blocking so exhibit superior performance in nodelet contexts.

The action supports only one goal, which is fully-preemptible. This means that a new goal always preempts the current goal (the previous goal's callee will be notified of preemption). Consequently, you must be careful not to interact with docking action, or any of its dependencies while active.

Internally, the perch behavior is encoded as a finite state machine depicted below and implemented using the ff_util/FSM class. This class essentially captures a map of (state, event) -> lambda function relationships. The state of the system is moved forward by setting the initial state and then pushing a sequence of events to the FSM.

\dotfile perch_fsm "Perch behavior finite state machine"

# Using the test tool

Typically, perching and unpeching are initiated through the Ground Data System (GDS). However, The perch behavior is packaged with its own gflags-based tool called perch_tool. This tool is essentially a convenience wrapper around an action client can be used to manually perch and unperch.

To get a list of supported commands, have a look at the help:

    rosrun perch perch_tool -helpshort

For example, lets assume that we have started the robot in free-flight, and it is currenty positioned so that its aft-side is facing a handrail. The exact location at which these conditions are met is world-speific.

    rosrun perch perch_tool -perch

Once perched, undocking can be initiated using the following command.

    rosrun perch perch_tool -unperch

At any point one can view the internal state of the perch behavior using the following command. The command will return a sequence of numbers, which represent a time ordered sequence of state transitions. Please refer to ```ff_msgs::PerchState``` for a mapping from the numeric values to states.

    rostopic echo /beh/perch/state

If you ever need to manually set the perch state to a speficic value, you can call the ```set_state``` service with the new state as the single argument:

    rosservice call /beh/perch/set_state 3