\page running-the-sim Running the Astrobee Simulator

This documentation assumes you have followed the \ref install-nonNASA or the
\ref install-NASA documentation.

## Communication Nodes

If you are an external user and would like to install and run the Astrobee
communication nodes, please contact your Astrobee point of contact.

## Setting up your Environment

You will need to setup your environment so that ROS knows about the new packages
provided by Astrobee flight software:

    pushd $BUILD_PATH
    source devel/setup.bash
    popd

After this command has completed, you should be able to run `ros` commands from
any directory in your Linux filesystem.

## Running the Simulator

To start a simulation of a single Astrobee docked in the ISS, run one of the
following:

If you don't have the Astrobee communication nodes compiled or installed,
please run:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true

If you have the Astrobee communication nodes compiled or installed, please run:

    roslaunch astrobee sim.launch rviz:=true

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
start up. If this happens, please see the 'Issues Starting up the Simulator'
section in \ref sim-issues.

**Important**: If you experience issues with the simulator and commanding the
robot, please see the \ref sim-issues before contacting the Astrobee team. 

### Simulator Flags

There are many options when launching the simulator. This section will take you
through the most used options. If you are happy with the simulator setup from
above, you can skip this section and jump to the 
[Moving the Robot](#moving-the-robot) section.

**Important**: You can specify more than one flag meaning you could launch more
than one of the GUIs we offer. The simulator is very resource intensive. The
more GUIs running, the more likely it is that you will get tolerance violation
errors when you try to move. If you are running a virtual machine, please make
sure that it has access to your graphics card.

#### pose

This is the pose/location that Astrobee starts up at. The first 3 values are 
position and the last 4 are the orientation expressed as a quaternion. By
default, Astrobee starts up docked in the JEM module of the ISS. To start up

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true pose:="11.25 -6.95 4.49 0 0 0 1"

*Note: If you have the Astrobee communication nodes compiled or installed,
please omit the dds and robot flags.*

#### gds

This flag is used to start up the ground data system (GDS). For the flag to
work, gds must be setup in a certain location. See the 
[Ground Data System](#ground-data-system) section for more about GDS and 
instructions on how to set it up. Please note that GDS will be unable to connect
to and command Astrobee if the Astrobee communication nodes are not running.
Also note, GDS is the least computationally intensive GUI out of the 3 we offer 
and it is the only GUI that allows users to send commands. Thus, it may be a
convenient tool for external users to use.

    roslaunch astrobee sim.launch gds:=true

*Note: If you don't have the Astrobee communication nodes compiled or
installed, the roslaunch above will throw a dds ros bridge fault and you will
not be able to move Astrobee.*

#### rviz

This flag is used to start up the ros visualization system also known as RVIZ.
RVIZ is our second most computationally intensive GUI.

*Note: `rviz` displays the robot perception of the world (estimated pose, sensor
measurements, etc.).*

#### sviz

This flag is used to start up the Gazebo GUI. This GUI is the most
computationally intensive GUI we have. We recommend using this GUI if you have a
high performance machine.

*Note: `sviz` displays the model of the world (ground truth). This means that
the position represented in rviz or gds can differ from the position in sviz.*

#### gviz

This flag starts the gnc visualizer. This visualizer is mainly used to see how
the localization and gnc systems are performing. For more information no gviz,
please see the \ref gncvisualizer documentation.

#### dds

This flag is used to start the Astrobee communication nodes. There are two
communication nodes in the Astrobee flight software: the dds ros bridge and the
astrobee to astrobee communication node. If you are unable to compile or install
the Astrobee communication nodes, please set dds to 'false' and set robot to
'sim_pub'.

#### speed

This flag is the speed multiplier for the simulator. By default, it is set to 1
meaning the simulator runs in real time. If you are having issues with simulator
performance, you can slow the simulator down by specifying a value less than
one. For example, the following would slow the simulator down to half speed:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true speed:=0.5

*Note: If you have the Astrobee communication nodes compiled or installed,
please omit the dds and robot flags.*

If you have a high performance machine, you can speed up the simulator by
specifying a value greater than 1. Please note, if you specify a speed up
greater than what your computer can handle, the simulator will not run at the
speed you specified. In this case, you can run gz stats in another terminal
to figure out what the actual speed up is. If the factor in gz stats doesn't
match the speed you specified, you can expect weird simulator issues.

Also note that all GUIs have a substantial effect on performance and care should
be taken when speeding up the simulation with GUIs enabled. Under certain
high-load conditions (many robots and/or a high speed factor) the outgoing
heartbeat queue in simulation ends up being too small for the number of 
messages that must be delivered. The outcome is this error message in your
console:

    [ERROR] [<timestamp>] : (ros.Astrobee) Fault with id # occurred. Fault error message: Didn't receive a heartbeat from <node>

Please see the 'Multiple Heartbeat Failures' section in \ref sim-issues on how
to fix this issue.

#### ns

This flag is the namespace of the simulated Astrobee. If you want to know more 
about ROS namespaces, please see the ROS documentation. This flag is only needed
when running multiple robots in simulation. Please see the
[Launching Multiple Robots](#launching-multiple-robots) section for more
information. The default namespace is blank.

#### robot

This flag tells the flight software which configuration file to read. If you
don't have the Astrobee communication nodes compiled or installed, you need to
use the sim_pub configuration so that the system doesn't complain about the
missing bridge. Otherwise, we recommend not setting this flag because the
configuration for each Astrobee is specific to the robot's hardware and it is
unknown if the hardware configuration will break something in simulation. If you
need to run multiple robots in simulation, please leave the robot flag set to
sim_pub or the default, sim, and instead set the namespace flag when spawning
a robot.

#### default_robot

This flag tells ROS to launch the default robot into the world. If you are
having performance issues starting up the simulator, you may want to launch the
world without the default robot and then spawn the robot after the simulator
starts up. For more information on this, please see the 'Issues Starting up the
Simulator' section in \ref sim-issues. To launch the world without the default
robot, please run:

    roslaunch astrobee sim.launch default_robot:=false

#### perch

This flag tells the simulation to start Astrobee in a perch ready position.
After the simulation has started up, you should be able to issue a perch command
and Astrobee will perch on a handrail on the deck.

#### world

This flag tells ROS which world to launch. There are 2 Astrobee worlds; the ISS
and the granite lab (a model of the actual testbed at Ames). By default, this is
set to the ISS world. If you want to test in the granite lab, launch the
simulation using the following:

    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true world:="granite"

*Note: If you have the Astrobee communication nodes compiled or installed,
please omit the dds and robot flags.*

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

*Note: If you have the Astrobee communication nodes compiled or installed,
please omit the dds and robot flags.*

## Moving the Robot

There are currently 3 ways to move the robot in simulation: using the Ground
Data System gui, using the command line teleop tool, or programmatically by
creating a guest science application that uses the astrobee api library.

### Ground Data System

The Ground Data System or GDS is our control station. It sends commands to
Astrobee and displays telemetry from Astrobee. Using GDS is the preferred way to
for an operator to operate Astrobee. Please follow one of the two sections based
on the type of user you are.

#### External Users

You can find the GDS software on github in the
[astrobee_gds repo](github.com/nasa/astrobe_gds). If you want to compile and run
GDS, you will need to aquire the RTI libraries. If you just want to run GDS, we
are able to distribute the GDS executable. You will also need to have the
Astrobee communication nodes compiled or installed for GDS to communicate with
Astrobee. As with GDS, you need the RTI libraries in order to compile the
Astrobee communication nodes but we are able to distribute the communications
debians that allow you to install and run the Astrobee communication nodes
without compiling the code. Please contact your Astrobee point of contact if
you would like the GDS executable and/or the Astrobee communications debians. 

Our script to launch GDS expects GDS to be in a certain location on your
machine. Thus, if you would like to use simulator GDS flag, please make a
folder in your home directory called GDS and in that directory, make a symbolic
link called 'latest' to the unextracted gds folder you received from your
Astrobee point of contact. If you want, you can put the unextracted folder in
the gds folder you created. Here's a rough idea of what to do:

    mkdir ~/gds
    pushd ~/gds
    unzip ~/Downloads/AstroBeeWB.rbuild#.date-time-linux.gtk.x86_64.zip
    ln -s AstroBeeWB.rbuild#.date-time latest
    popd

#### Internal Users

If you are interested in running GDS, please follow the
[Running GDS in Linux](https://babelfish.arc.nasa.gov/confluence/display/FFFSW/Running+GDS+in+Linux)
page on the FreeFlyer FSW Confluence space.

If you plan on working on GDS code, please checkout the bitbucket
[astrobee_gds repo](ssh://git@babelfish.arc.nasa.gov:7999/astrobee/astrobee_gds.git)
and follow the SETUP documentation. If you followed the NASA Astrobee install
instructions, you will have the RTI libraries already installed. Please make
sure you set the LD_LIBRARY_PATH to the correct RTI location when following the
'Run the Control Station for the first time' section.

### Teleop Tool

A teleop tool has been added to allow you to issue some of the basic Astrobee
commands and get a small set of telemetry from the robot. This should only be
used to play with the simulator and get a feel for how Astrobee works. As an
external user, we consider you to be a guest scientist and we expect you to
follow the guest science framework we have created. Please see the
[Guest Science Applications](#guest-science-applications) section for more
information.

For information on how to use the teleop tool, please see the
\ref teleop.

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
when the namespace is blank. In other words, please don't use the ns flag when
launching the simulator if you want to see the images. To show sci camera
images, modify analogously the `sci_cam_rate` field, and enable this camera
from the `Panels` menu in RVIZ.

**Important** Unless you have a high performance machine, there is a good chance
that these images will not initialize properly and you will be looking at the
inside of the robot. If this happens, you will need to launch the world and
Astrobee separately. Please follow the issues starting up the simulator section 
in \ref sim-issues with one caveat. DO NOT start up Astrobee (i.e. don't issue
the second terminal command) until 30 to 40 seconds after RVIZ has started up.
If you have a low performance computer, this fix may not be the only thing you
have to do to fix the images. Please try reducing the speed of your simulator
as well. For information on how to reduce the simulator speed, please see the
[speed](#speed) flag section.

## Launching Multiple Robots

The Astrobee simulator supports running up to three robots. Please keep in mind
that each simulated robot takes computer resources and can cause simulator
issues if your machine cannot support the extra load.

The robot namespaces we support are "bumble", "queen", and "honey". Please DO
NOT capitalize the namespace. When starting the simulator, you can either start
with no robots or you can start with the default robot and one of the
namespaces. To start the simulator without a robot, please run:

    roslaunch astrobee sim.launch default_robot:=false

To start with the default robot using a namespace, please run:

    roslaunch astrobee sim.launch ns:=honey dds:=false robot:=sim_pub

To spawn robots into the simulator, run: 

    roslaunch astrobee spawn.launch ns:=bumble dds:=false robot:=sim_pub pose:="11 -7 4.5 0 0 0 1"
    roslaunch astrobee spawn.launch ns:=queen dds:=false robot:=sim_pub pose:="11 -4 4.5 0 0 0 1"

*Note: If you have the Astrobee communication nodes compiled or installed,
please omit the dds and robot flags.*

**Important** If you issue the roslaunch commands too close together, the robot
may not start up properly. Please allow 15 to 20 seconds in between each
roslaunch command.

Each robot's ROS topics, services and actions are now prefixed with the supplied
namespace, which allows you to interact with them individually. So, if you are 
using the teleop tool to command the robots, be sure to specify the namespace of
the robot you want to command. For example:

    rosrun executive teleop_tool -ns "honey" -undock 

*Note: The simulator and teleop tool may be able to support more than 3 robots
if needed. However, GDS only supports 3 robots so the documentation limits
spawning to 3. We will leave it as an exercise for the user to launch more than
3 robots.*

# Simulator Issues

Please see \ref sim-issues.
