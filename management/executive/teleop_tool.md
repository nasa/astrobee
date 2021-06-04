\page teleop Teleoperation Tool Instructions

These instructions assume that you have followed the
[install instructions](../../INSTALL.md) and the
[simulation instructions](../../simulation/sim_overview.md).

A teleop tool has been added to allow you to issue some commands and get some
telemetry from the robot. This should only be used to play with the simulator
and get a feel for how Astrobee works. As an external user, we consider you to
be a guest sciencist and we expect you to follow the guest science framework we
have created. Please see the Guest Science Applications section in the
[simulation_instructions](../../simulation/sim_overview.md#guest-science-applications) for
more information.

To use the teleop tool, the user must specify what they want to do through
flags. This section will take you through the flags the tool offers. Please
note, you can only specify one mobility flag at a time. For example, you
will not be able to use both the dock flag and the move flag. Also you don't
need to use a mobility flag. You can run the teleop tool just to get telemetry.
Please make sure the terminal you are using for the tool has its environment
setup. For information on how to setup the terminal environment, please see
the Setting up your Environment section of the
[simulator readme](../../simulation/sim_overview.md#setting-up-your-environment).

## Basic Usage

### Dock

This flag can be used to dock the robot and is considered a mobility flag. To
dock, please run the following:

    rosrun executive teleop_tool -dock

By default, the robot will dock in berth 1 (the left berth). If you would like
it to dock in the right berth, you will have to use the berth flag. To dock
Astrobee in the right berth, please run the following:

    rosrun executive teleop_tool -dock -berth 2

### Move

This flag can be used to move the robot and is be considered a mobility flag. If
you want to move relative to the robot, please use the relative flag. If you
want to specify an absolute position, do not specify the relative flag. You
should use the pos flag to specify the desired position in cartesian format
(X Y Z). You should use the att flag to specify the desired attitude in
angle-axis format. You don't need to specify the position if you only want to
rotate the robot and you don't need to specify the attitude if you only want to
move Astrobee. Please see the following examples for clarity (these examples
assume you are using the ISS world):

To move the robot forward 1, right 2 and down 0.5:

    rosrun executive teleop_tool -move -relative -pos "1 2 0.5"

To move the robot back 1:

    rosrun executive teleop_tool -move -relative -pos "-1"

To rotate the robot around Y (axis X=0 Y=1 Z=0) by -1.5 radians:

    rosrun executive teleop_tool -move -att "-1.5 0 1 0"

To move to the middle of the JEM:

    rosrun executive teleop_tool -move -pos "11.25 -6.59"

To move to the dock approach point facing the dock:

    rosrun executive teleop_tool -move -pos "10.34 -9.51 4.49" -att "-3.14 0 0 1"

### Stop

This flag can be used to stop the robot and is considered a mobility flag. If
you are using the teleop tool to move the robot and it isn't running in the
background, you will have to run the teleop tool with the stop flag in another
terminal whose environment is setup. For information on how to setup the
terminal, please see the Setting up your Environment section of the
[simulator readme](../../simulation/sim_overview.md#setting-up-your-environment).
To stop, run:

    rosrun executive teleop_tool -stop

### Undock

This flag can be used to undock the robot and is considered a mobility flag. To
undock, please run the following:

    rosrun executive teleop_tool -undock

### Get Position

This flag is used to get the robot's current position. To use it, run:

    rosrun executive teleop_tool -get_pose

### Get State

This flag is used to get the robot's state. This will display the robot's
operating and mobility states. If you try to issue a command that is rejected
because the robot is not in the right state, you may want to use this flag to
figure out what the robot is doing. To use it, run:

    rosrun executive teleop_tool -get_state

### Reset Ekf

This flag is used to reset the localization. If your Astrobee is spinning or
jumping around erraticaly, you should try to reset the ekf. To do this, run:

    rosrun executive teleop_tool -reset_ekf

## Advanced Usage

### Namespace

If you used a namespace when launching the robot, you must use a namespace for
the teleop tool. If you don't, the program will report that there are no
publishers for the ack topic and the program will then exit. For example, to
undock the Astrobee named bumble, you would issue the following:

    rosrun executive teleop_tool -ns "bumble" -undock

**Important**: Much like the launch namespace, please DO NOT capitalize the
namespace as we don't support this.

### Reset Bias

This flag is used to reset the bias. To do this, run:

    rosrun executive teleop_tool -reset_bias

### Get Faults

This flag is used to display the current faults in the system. If you received
a heartbeat fault, you may want to check the faults to see if the heartbeat
came back (i.e. the heartbeat fault disappeared). To use this flag, run:

    rosrun executive teleop_tool -get_faults

### Set Planner

This flag is used to set the planner. The flight software currently supports two
planners; a trapezoidal and a quadratic program (qp). The default planner is the
trapezoidal planner. If you would like to use the qp planner, run:

    rosrun executive teleop_tool -set_planner "qp"

The qp planner has not been tested rigorously and may have some bugs. To switch
back to the trapezoidal planner, run:

    rosrun executive teleop_tool -set_planner "trapezoidal"

### Get Planner

This flag is used to display the current planner. If you want to see the planner
being used, please run:

    rosrun executive teleop_tool -get_planner

### Set Face Forward

This flag is used to tell the robot to fly in face forward mode. By default,
the robot flies in face forward mode meaning the robot will turn towards the
point it is navigating to before flying. If face foward mode is turned off, the
robot will not turn towards the point it is navigating to or in other words, the
robot will fly blind. If there is an object in the robot's path, the robot will
collide with it so please use this flag with caution. To turn face forward off,
run:

    rosrun executive teleop_tool -set_face_forward "off"

To turn face forward back on, please run:

    rosrun executive teleop_tool -set_face_forward "on"

### Get Face Forward

This flag is used to display the current state of face forward mode. To see if
Astrobee is current flying face forward, please run:

    rosrun executive teleop_tool -get_face_forward

### Set Operating Limits

This flag is used to set the operating limits. If you set the operating limits
incorrectly, Astrobee may be unable to move. If you want to set the operating
limits, we recommend looking at the flight modes so you know what the
upper/hard limits for the flight modes are. The flight modes can be found in the
world config files which can be found in `astrobee/config/worlds/`. Please note
that the mode must match the name of one our flight modes. Also note, there
are docking, undocking, perching and unperching flight modes. You DO NOT need
to set these modes before you dock, undock, perch, or unperch; the flight
software does this for you. If you want to use nominal mode but you want the
target velocities and accelerations to be half the hard limits, run:

    rosrun executive teleop_tool -set_op_limits -mode "nominal" -vel 0.1 -accel 0.01 -omega 0.08725 -alpha 0.08725 -collision_distance 0.25

If you want to push astrobee to the max, you can try to use the hard limits for
difficult mode. This may not work and may result in tolerance violations when
trying to move Astrobee. To do this, run:

    rosrun executive teleop_tool -set_op_limits -mode "difficult" -vel 0.4 -accel 0.0400 -omega 0.5236 -alpha 0.5236 -collision_distance 0.25

### Get Operating Limits

This flag is used to display the current operating limits. To use it, run:

    rosrun executive teleop_tool -get_op_limits

