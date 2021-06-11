\page sim-issues Common Simulation Issues

## The program 'roslaunch/rosrun' is currently not installed

This usually happens when the environment has not been setup. Please see the
'Setting up your Environment' section in \ref running-the-sim.

## \[Sim.launch\] is neither a launch file in package \[astrobee\] ...

This usually happens when the environment has not been setup. Please see the
'Setting up your Environmant' section in \ref running-the-sim.

## Rospack Error Finding Package

This usually happens when the environment has not been setup. Please see the
'Setting up your Environment' section in \ref running-the-sim.

## Issues Starting up the Simulator

There is a small chance that you will have issues when starting up the
simulator. This mainly happens in computers with less computing power. If you
are experiencing issues, try launching the world and Astrobee seperately. To do
this, you will need to open 2 terminals. Please make sure you setup your
environment in both terminals (see the 'Setting up your environment' section
in \ref running-the-sim). In the first terminal, run:

    roslaunch astrobee sim.launch default_robot:=false rviz:=true

After everything in the first terminal starts up, in the second terminal, run:

    roslaunch astrobee spawn.launch dds:=false robot:=sim_pub

*Note: If you have the dds ros bridge compiled or installed, please omit the
dds and robot flags.*

If the robot body doesn't show up in RVIZ, please see the
[Robot Body Doesn't Show up in RVIZ](#robot-body-does-not-show-up-in-RVIZ)
section on how to fix this.

If this doesn't fix the problem, you may need to slow down the simulation to
a speed that your computer can handle. Please see the
[Tolerance Violated Errors](#tolerance-violated-errors) section.

## Robot Body Does Not Show up in RVIZ

If you end up having to launch the world and Astrobee separately, there is a
chance that the robot body will not show up in RVIZ. To fix this, you can toggle
the robot. To do this, please find the robot with the namespace you are using
in the `Displays` section of RVIZ (bottom left corner) and click the checkbox
located to the right twice (first to turn it off, then to turn it back on). If
you didn't specify a namespace, the namespace is the forward slash character(/)
and it can be found in the Debug section. It should have a red circle with a
white rectangle in the circle in front of the forward slash and the forward
slash should be red but this will go away after toggling it.

## No Image message in RVIZ

This is not an error in our system. Please see the 'Images' section in 
\ref running-the-sim for more information.

## Heartbeat Fault Detected

### DDS ROS Bridge

More than likely this is because the dds ros bridge is not running. If so,
please set the robot flag to sim_pub. If you have the dds ros bridge compiling,
you can set the dds flag to true. If you want to use a specific robot instead of
sim_pub, you will need to add the dds ros bridge node to the list of nodes not
running in the robot config. For example, if you are simulating the robot
*bumble*, you must open the LUA config file
*astrobee/config/robots/bumble.config*, and add the node name so that it looks
something like this

    node_not_running = {"dds_ros_bridge"}

### Multiple Heartbeat Failures

This is mostly likely due to a performance issue. If running in a virtual
machine, please make sure that it has access to your graphics card. If you have
sped up the simulation and are running one or more GUIs, the outgoing heartbeat 
queue may be too small for the number of messages that must be delivered. You
can fix this error by increasing the value of variable "heartbeat_queue_size" in
the robot config. For example, if you are simulating the default simulation
robot, you must open the LUA config file *astrobee/config/robots/sim.config*,
and increase the value of this line:

    heartbeat_queue_size = <x>

## Command Fails due to Robot State

Astrobee only excepts a subset of commands in each operating and mobility state.
If you are using GDS, the mobility and operating states will be displayed in a
health section/subtab at the top left corner of most of the main tabs. If you
are using the teleop tool, please see the 'Get State' section in \ref teleop.

A general rule of thumb is Astrobee will not allow mobility commands or commands
that set mobility properties when executing a mobility command. For instance,
Astrobee will reject a move command when it is executing an undock command and
it will reject a set operating limits command when executing a move command. In
these instances, you can either wait for the command to complete or issue a stop
command which cancels the mobility command being executed.

Futher, Astrobee accepts a limited set of commands when in the fault operating
state. If you are using the teleop tool, please see the 'Get Faults' section of
the \ref teleop to get the list of occurring faults. If you are using GDS,
please navigate to the 'Engineering' tab. The faults should be listed in the
bottom left corner. Most times a flight software/simulator restart will get
rid of the faults. However, if you are repeatedly getting heartbeat faults,
please see the [Heartbeat Fault Detected](#heartbeat-fault-detected) section.

## Tolerance Violated Errors

Most of the time tolerance violated errors occur due to computer performance. To
see if this is the case, please run *gz stats* in a terminal that has its
environment setup while the simulator is running. If the factor doesn't roughly 
match the speed you are running the simulator at, you will have to reduce the
speed of the simulator. Please see the 'speed' section in \ref running-the-sim
on how to change the simulator speed. In some cases, this means you will have to
run the simulator at a speed slower than real time.

## Keep in Zone Violation

If you get a keep in zone violation failure when trying to move, please make
sure the position you are trying to get to is in the world Astrobee is operating
in. For reference, the point (0, 0, -0.67) is in the middle of the granite table
in the granite world but it is not inside the ISS in the ISS world. If you are
using the teleop tool, please see the 'Get Position' section in \ref teleop to
get the current positon of the robot. If you are using GDS, you can get the
current position by navigating to the 'Teleoperate' tab and clicking on the
'Snap Preview to Bee' buttonb. This will change the manual inputs to match the
current position of Astrobee.

## Unable to Plan a Segment

The only time we have seen this happen is when the operating limits are not set
correctly. Please make sure the operating limits are within Astrobee's
capabilities. A flight software/simulator restart will reset the operating
limits back to the defaults.

## Couldn't Configure the Mobility Node

This error usually occurs when an operator issues a move command before the
flight software has finished starting up. Please wait a little longer after
starting the simulator before trying to issue a move command.

## Could Not Query the Pose of Robot

This error usually occurs when an operator issues a move command before the
flight software has finished starting up. Please wait a little longer after
starting the simulator before trying to issue a command.

## Dock Action Server Not Connected

This error usually occurs when an operator issues an undock command before the
flight software has finished starting up. Please wait a little longer after
starting the simulator before trying to issue a command.

## GDS Node - Process has Died

This error occurs if the gds flag was set to true but the Astrobee launch GDS
script is unable to find and start GDS. Please ensure that you have GDS and that
you have followed the GDS setup instructions in section 'Ground Data System' of
\ref running-the-sim.

## The Robot is Spinning, Flying Higher Than the World, or Jumping Around Erratically

To fix any of these issues, please try to reset the ekf first. If that doesn't
work, try initializing the bias. If you are using the teleop tool, please see
the 'Reset Ekf' and 'Reset Bias' sections in \ref teleop on how to do this. If
you are using GDS, please navigate to the 'Engineering' tab. The 'Reset EKF' and
'Initialize Bias' buttons should be in the top right corner.


### Teleop Tool Reports No Publisher for Acks Topic

This issue occurs when a namespace is used for Astrobee but the namespace wasn't
provided to the teleop tool or the namespace provided doesn't match Astrobee's
namespace. Please check the namespace of Astrobee and make sure it matches the
namespace provided to the teleop tool. Remember, Astrobee namespaces should not
be capitialized.

### Bumble not in repository

This error only occurs when running the simulator in a native linux machine
instead of a virtual machine. To get rid of it, you will have to open the
astrobee/config/communications/dds/RAPID_QOS_PROFILES.xml file and remove
ip address '128.102.\*' from line 118.
