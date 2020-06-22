\page sim_issues List of common issues encountered with the simulator

**This is a draft document**

### The program 'roslaunch/rosrun' is currently not installed

This usually happens when the environment has not been setup. Please see the
[Setting up your Environment](#setting-up-your-environment) section.

### \[Sim.launch\] is neither a launch file in package \[astrobee\] ...

This usually happens when the environment has not been setup. Please see the
[Setting up your Environmant](#setting-up-your-environment) section.

### Rospack Error Finding Package

This usually happens when the environment has not been setup. Please see the
[Setting up your Environment](#setting-up-your-environment) section.

### Issues Starting up the Simulator

There is a small chance that you will have issues when starting up the
simulator. This mainly happens in computers with less computing power. If you
are experiencing issues, try launching the world and Astrobee seperately. To do
this, you will need to open 2 terminals. Please make sure you setup your
environment in both terminals (see
[Setting up your environment](#setting-up-your-environment)). In the first
terminal, run:

    roslaunch astrobee sim.launch default:=false rviz:=true

After everything in the first terminal starts up, in the second terminal, run:

    roslaunch astrobee spawn.launch dds:=false robot:=sim_pub

If the robot body doesn't show up in RVIZ, please see the
[Robot Body Doesn't Show up in RVIZ](#robot-body-does-not-show-up-in-RVIZ)
section on how to fix this.


If this doesn't fix the problem, you may need to slow down the simulation to
a speed that your computer can handle. Please see the
[Tolerance Violated Errors](#tolerance-violated-errors) section.

### Robot Body Does Not Show up in RVIZ

If you end up having to launch the world and Astrobee separately, there is a
chance that the robot body will not show up in RVIZ. To fix this, you can toggle
the robot. To do this, please find the robot with the namespace you are using
in the `Displays` section of RVIZ (bottom left corner) and click the checkbox
located to the right twice (first to turn it off, then to turn it back on). If
you didn't specify a namespace, the namespace is the forward slash character(/)
and if can be found in the Debug section. It should have a red circle with a
white rectangle in the circle in front of the forward slash and the forward
slash should be red but this will go away after toggling it.

### No Image message in RVIZ

This is not an error in our system. Please see the [Images](#images) section
for more information.

### Heartbeat Fault Detected

### Command Fails due to Robot State
Cannot move when in ready and not stopped
Command not accepted in op state teleop

### Tolerance Violated Errors
use gz stats to make sure you are running at your speed up factor

### Keep in Zone Violation
Make sure the position you are trying to get to is in the world

### Unable to Plan a Segment
The only time we have seen this happen is when the op limits are not set
correctly. make sure your limits are correct

### Couldn't Configure the Mobility Node
Wait a little longer before trying to move robot

### GDS Node - Process has Died
You don't have GDS as of yet, don't set the the gds flag to false

### The robot is spinning, flying higher than the world, or jumping around erratically
need to reset ekf or initialize bias, see teleop section


### Teleop Tool reports no publisher for acks topic
make sure the namespace matches the namespace you launched with

<!---
### Bumble not in repository
(ros.Astrobee) RapidCommandRosCommand exception: get failed, "Bumble" not in repository
terminate called after throwing an instance of 'Miro::RepositoryBase::ENotRegistered'
  what():  get failed, "Bumble" not in repository
[mlp_communications-17] process has died [pid 18163, exit code -6, cmd /opt/ros/kinetic/lib/nodelet/nodelet manager __name:=mlp_communications __log:=/home/jmlombar/.ros/log/d89caf2c-5a23-11e9-abba-a402b9d91435/mlp_communications-17.log].

Remove first IP in QOS profile
-->
