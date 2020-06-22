\page sim_overview Simulator usage instructions for non-NASA users

This documentation assumes you have followed the [install readme](../INSTALL.md).

## Setting up your Environment

You will need to setup your environment so that ROS knows about the new packages
provided by Astrobee flight software:

    pushd $BUILD_PATH
    source devel/setup.bash
    popd

After this command has completed, you should be able to run `ros` commands from
any directory in your Linux filesystem.

## Running the Simulator

To start a simulation of a single Astrobee docked in the ISS, run the following:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true

- Note 1: The very first time Gazebo starts, it could take a long time because
  it loads some large models
- Note 2: To run the simulation with the Gazebo rendering, you can add the flag
  `sviz:=true`. This allows one to see the ISS module used for the simulation.
  However this is resource intensive.

This command tells ROS to look for the `sim.launch` file provided by the
`astrobee` package, and use roslaunch to run it. Internally, ROS maintains a
cache of information about package locations, libraries and executables. If you
find that the above command doesn't work, try rebuilding the cache:

    rospack profile

This command launches both the world and Astrobee. There are things in the
world that need to be started up before Astrobee is started up. There is a
small chance that this will not happen and the user will experience issues upon
start up. If this happens, please see the
[Issues Starting up the Simulator](#issues-starting-up-the-simulator) section.

**Important**: If you experience issues with the simulator and commanding the
robot, please see the [Simulator Issues](#simulator-issues) section before
contacting the Astrobee team. 

### Simulator Flags

There are many options when launching the simulator. This section will take you
through the options we think are important to external users. If you are happy
with the simulator setup from above, you can skip this section and jump to the
[Moving the Robot](#moving-the-robot) section.

**Important**: You can specify more than one flag meaning you could launch more
than one of the GUIs we offer. The simulator is very resource intensive. The
more GUIs running, the more likely it is that you will get tolerance violation
errors when you try to move. If you are running a virtual machine, please make
sure that it has access to your graphics card.

#### pose

This is the pose/location that Astrobee starts up at. The first 3 values are 
position and the last 4 are the orientation expressed as a quaternion. By
default, the Astrobee starts up docked in the JEM module of the ISS. To start up
in the middle of the JEM, run the following:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true pose:="11.25 -6.95 4.49 0 0 0 1"

#### gds

This flag is used to start up our ground data system (GDS). Currently, external
users do not have access to GDS. Please see the
[Ground Data System](#ground-data-system) section for more information. GDS is
the least computationally intensive GUI out of the 3 we offer and it is the only
GUI that allows users to send commands. Thus we would prefer it if users use
GDS when it is released to the public.

#### rviz

This flag is used to start up the ros visualization system also known as RVIZ.
RVIZ is our second most computationally intensive GUI. If you don't have access
to GDS, we recommend using RVIZ to visualize what the robot is doing.

*Note: `rviz` displays the robot perception of the world (estimated pose, sensor
measurements, etc.).*

#### sviz

This flag is used to start up the Gazebo GUI. This GUI is the most
computationally intensive GUI we have. We recommend using this GUI if you have a
high performance machine.

*Note: `sviz` displays the model of the world (ground truth). This means that
the position represented in rviz or gds can differ from the position in sviz.*

#### dds

This flag is used to tell ROS if it should start the DDS ROS bridge. This bridge
basically translates ROS to DDS and vice versa. DDS is used to communicate with
our GDS. Since GDS is not available to external users, please always set the dds
flag to false. 

#### speed

This flag is the speed multiplier for the simulator. By default, it is set to 1
meaning the simulator runs in real time. If you are having issues with simulator
performance, you can slow the simulator down by specifying a value less than
one. For example, the following would slow the simulator down to half speed:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true speed:=0.5

If you have a high performance machine, you can speed up the simulator by
specifying a value greater than 1. Please note, if you specify a speed up
greater than what your computer can handle, the simulator will not run at the
speed you specified. In this case, you can run gz stats in another terminal
to figure out what the actual speed up is. If the factor in gz stats doesn't
match the speed you specified, you can expect weird simulator issues.

#### ns

This flag is the namespace of the simulated Astrobee. If you want to know more 
about ROS namespaces, please see the ROS documentation. This flag is only needed
when running multiple robots in simulation. Please see the
[Launching Multiple Robots](#launching-multiple-robots) section for more
information. The default namespace is blank.

**Important**: We only support 4 namespaces in simulation; the blank namespace,
honey, bumble, and queen. Please DO NOT capitalize the namespace as our GUIs
don't support this.

#### robot

This flag tells the flight software which configuration file to read. Since
external users don't have access to our GDS and thus don't run our dds ros
bridge, we have created a sim_pub configuration so that the system doesn't
complain about the missing bridge. Until GDS is public, please always set this
flag to sim_pub.

#### default

This flag tells ROS to launch the default robot into the world. If you want to
launch the world without the default robot, please set this to `false` (see the
[developers simulator introduction](readme.md) for an example).

#### world

This flag tells ROS which world to launch. There are 2 Astrobee worlds; the ISS
and the granite lab (a model of the actual testbed at Ames). By default, this is
set to the ISS world. If you want to test in the granite lab, you will have to
set up the robot for a world with gravity. To do this, first launch the
simulation using the following:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true world:="granite"

After the simulator is launched, you will have to reset the bias as we encode
gravity in our bias. Please see the Reset Bias section of the 
[teleop tool readme](../management/executive/teleop_tool.md#reset-bias)
for instructions on how to send the reset bias command. After you do
this, the robot should be stable. If you want to go back to using the ISS world,
you will have to issue another reset bias command after launching the robot in
the ISS world.

#### debug

The Astrobee flight software is still in development and there are bugs that we
don't always catch. If you come across a bug that crashes the flight software,
please help us fix it. To help us, we need to know what happened. The crashed
output should state the node that caused the crash. If it isn't clear which node
it was, please email us the output and we can help. Once you have the name of
the node, please set the debug flag to the node name. This flag will cause a GNU
Debugger(GDB) terminal to pop up. Once the code crashes, navigate to the GDB
terminal, type `bt`, and hit enter. This should display a backtrace. Please copy
the backtrace and send it to us. Here is an example of launching the simulator
and debugging the executive node:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true debug:="executive" 

## Moving the Robot

There are currently 3 ways to move the robot in simulation.

### Ground Data System

The Ground Data System or GDS is our control station. It sends commands to
Astrobee and displays telemetry from Astrobee. Using GDS is the preferred way to
for an operator to operate Astrobee. However, GDS isn't currently open sourced.
Thus as an external user, you will not be able to use it until it is open
sourced.

### Teleop Tool

A teleop tool has been added to allow you to issue some commands and get some
telemetry from the robot. This should only be used to play with the simulator
and get a feel for how Astrobee works. As an external user, we consider you to
be a guest scientist and we expect you to follow the guest science framework we
have created. Please see the
[Guest Science Applications](#guest-science-applications) section for more
information.

For information on how to use the teleop tool, please see the
[teleop tool instructions](../management/executive/teleop_tool.md).

### Guest Science Applications

If you are a guest scientist, please make sure you have the astrobee_android
NASA GitHub project checked out (if you followed the usage instructions, you
should have checked this out already). For more information on guest science
and how to create guest science applications, please see
[`astrobee_android/readme.md`](https://github.com/nasa/astrobee_android/blob/master/README.md)
located in the `astrobee_android/` folder.

## Images

When running RVIZ, you may notice the big "No Image" text in the two
`DEBUG` windows that are meant to show camera information. This is
because, by default, we don't publish simulated camera images for the
navigation and dock cameras due to the fact that simulating these
images is very resource intensive. If you do want to generate these
images, please open `astrobee/config/simulation/simulation.config` and
change one or more of `nav_cam_rate` and `dock_cam_rate` to something
other than 0. We recommend setting these values to 1 meaning these
images will be published once per second. After this, you should see
those images when you start the simulator. Please note this only works
when the namespace is blank. If you want to see these images, please
don't use the ns flag when launching the simulator. To show sci camera
images, modify analogously the `sci_cam_rate` field, and enable this camera
from the `Panels` menu in RVIZ.

**Important** Unless you have a high performance machine, there is a good chance
that these images will not initialize properly and you will be looking at the
inside of the robot. If this happens, you will need to launch the world and
Astrobee separately. Please follow the
[Issues Starting up the Simulator](#issues-starting-up-the-Simulator) section
with one caveat. DO NOT start up Astrobee (i.e. don't issue the second terminal
command) until 30 to 40 seconds after RVIZ has started up. If you have a low
performance computer, this fix may not be the only thing you have to do to fix
the images. Please try reducing the speed of your simulator as well. For
information on how to reduce the simulator speed, please see the [speed](#speed)
flag section.

## Launching Multiple Robots

Coming soon!!!

<!---
 (Each robot takes more computer resources)
<> (GDS only supports 3 robots.)
<> (While this is offered, we have not tested this thoroughly and you could run into unforeseen issues.)
<> (If using teleop tool, be sure to specify the namespace)
-->

## Simulator Issues

[List of common issue with the simulator](sim_issues.md)

