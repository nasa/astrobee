\page arm Arm Behavior

# Overview

Astrobee is equipped with a 3DoF perching arm, which may be attached to either the top or the bottom payload bay. The perching arm enables Astrobee to grasp handrails on the International Space Station (ISS). Once grasped the remaining two degrees of freedom are used as a pan/tilt mechanism. This allows propulsion and localization to be turned off completely, thereby saving significant energy and computational resources.

Control of the arm is achieved by a three-layer driver stack. The firmware (avionics) is at the lowest level, and executes on a dedicated microcontroller. It has direct control of the servos. The firmware communicates over serial with the mid-level driver (perching_arm) which runs on the mid-level processor (MLP). The mid-level driver mostly acts as a bridge, converting serial commands to and from generic ```sensor_msgs::JointState``` messages. This abstraction integrates the perching arm into the ROS ecosystem, allowing us to use generic visualization tools, simulation software and controllers. At the highest level is the arm behavior (this node). This enforces high-level logic on arm usage and a convenient means of interacting with the arm.

# Internal behavior

The arm behavior provides the callee with a single action-based entry point for controlling the arm, called ```ff_msgs::ArmAction```. This "arm action" accepts a command, executes it in the background, and sends periodic feedback. When the task completes a result is returned to the callee. Unlike services, actions are non-blocking so exhibit superior performance in nodelet contexts.

The action supports only one goal, which is fully-preemptible. This means that a new goal always preempts the current goal (the previous goal's callee will be notified of preemption). Consequently, you must be careful not to interact with the arm if another entity is using it, for example the perch behavior.

Fundamentally, the arm action provides the following commands:

* ARM_STOP - Stop any action underway.
* ARM_DEPLOY - Deploy arm to pan = 0, tilt = 0.
* ARM_STOW - Stow arm back to its home position.
* ARM_PAN - Pan to a specific value in degrees.
* ARM_TILT - Tilt to a specific value in degrees.
* ARM_MOVE - Move to a specific pan and tilt value.
* GRIPPER_CALIBRATE - Instruct firmware to find gripper end-stops.
* GRIPPER_SET - Set the gripper to a percentage open.
* GRIPPER_OPEN - Open the gripper.
* GRIPPER_CLOSE - Close the gripper.

Internally, the arm behavior is encoded as the finite state machine depicted below and implemented using the ff_util/FSM class. This class essentially captures a map of (state, event) -> lambda function relationships. The state of the system is moved forward by setting the initial state and then pushing a sequence of events to the FSM.

\dotfile arm_fsm "Arm finite state machine"

The state machine is fairly intuitive. The only aspect that warrants further discussion is the mapping of commands to states. Under the hood, an ARM_PAN / ARM_TILT command is really the same as an ARM_MOVE command, except the respective tilt / pan values remain unchanged. Likewise, ARM_DEPLOY is mapped to an ARM_MOVE to pan and tilt values of zero, and GRIPPER_CLOSE / GRIPPER_OPEN are simply just GRIPPER_SET to 0 and 100 respectively.

# Using the test tool

Typically, the arm will be controlled through the Ground Data System (GDS). However, The arm behavior is packaged with its own gflags-based tool called arm_tool. This tool is essentially a convenience wrapper around an action client can be used to control the arm from the command-line.

To get a list of supported commands, have a look at the help:

    rosrun arm arm_tool -helpshort

For example, lets assume that we have started the robot and the arm is stowed in its home position (tucked into the body). To deploy the arm, pan to 45 degrees then tilt to 30 degrees use the following sequence of commands:

    rosrun arm arm_tool -deploy
    rosrun arm arm_tool -pan 45
    rosrun arm arm_tool -tilt 30

Before using the gripper it must first be calibrated. Calibration involves finding the number of motor ticks at the high-stop -- when the gripper in the fully-open position. The low-stop is treated as a fixed number of ticks relative to the high-stop. Visually, calibration appears as if the gripper is opened once and then closed once.

To calibrate the gripper, open it, close it and then set it to 50% open try the following sequence of commands:

    rosrun arm arm_tool -cal
    rosrun arm arm_tool -open
    rosrun arm arm_tool -close
    rosrun arm arm_tool -set 50

Finally, you can stow the arm. Note that the arm can be stowed from any position. To avoid self-collisions the gripper is first closed, then the arm is then first panned to zero and then tilted to its stow point. You can stow the arm using the following command:

    rosrun arm arm_tool -stow

While waiting for the instruction to complete, the arm_tool will print out periodic feedback, which takes the following form. Pan and tilt angles are expressed in degrees, while the gripper position is expressed in percentage. Once completed, a final response is printed.

    PAN: X deg TILT: Y deg GRIPPER: Z %
    Response: Successfully completed

At any point one can inspect the internal state of the arm behavior using the following command. The command will return a sequence of numbers, which represent a time ordered sequence of states. Please refer to ```ff_msgs::ArmState``` for a mapping from numbers to states.

    rostopic echo /beh/arm/state

If you ever need to manually set the arm state to a specific value, you can call the ```set_state``` service with the new state as the single argument.

    rosservice call /beh/arm/set_state 1