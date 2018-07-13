\defgroup smart_dock Smart dock driver
\ingroup hw

The smart_dock package provides a single non-ROS service that accepts a DDS
command to wake up an Astrobee that is docked to berth 1 or 2. It is essentially
a proxy that converts this command to a i2c transaction that is sent from the
amrt dock running Linux to the slave dock controller. The dock controller then
sets an internal register, which is then collected at some future point in time
by the astrobee, which is an i2c master on a separate bus.

    [smart dock] <(m)-i2c-(s)> [dock controller] <-(s)-i2c-(m)-> [astrobee]

# Implementation details

Like most of the hardware drivers, there is a proxy library (smart_dock) that
provides an abstraction away from the i2c bus transactions. The smart_dock_node
is a free-standing DDS application that provides a bridge between the network
and the proxy library. The smart_dock_tool is a simple command-line utility that
allows one to manually probe the smart dock.

# Functional test

Ultimately, smart dock will bem automatically launched as a systemd service.
Take a look in the scripts/daemon/dock folder of the astrobee robot software for
example service files for doing this. If you have not installed the service then
you can run it manually:

First, make sure nothing is docked to either of the berths. Then, make sure you
are connected to the ISS network. You can not SSH into the dock using its ISS IP
and run the smart_dock_service application:

    ssh astrobee@$DOCK_IP
    [dock] astrobee@dock:/res$ smart_dock_service

This application pushes telemetry whenever the dock state changes. It also
respondes to commands from GDS and wakes up a robot that is berthed in hibernate
mode.

You can now launch GDS, making sure the DOCK_IP is in your DDS peer list. If all
works correctly, you should see "Vacant" in both berths listen under the
"Docking Station Status and Commanding" pane, which is the bottom left quarter
of the opening screen.

Now, turn on the power switch to an astrobee but do not wake it up with the wake
button. In stead, plug the astrobee into berth 1. In less than 10 seconds you
should see berth 1 update to reflect the name of the astrobee and the fact that
it's in hibernate mode. You might also notice that you can now wake the astrobee
using the button underneath the status panes.

Finally, press the "wake" button in the "Wake Commanding" pane. You should see
the status change to "Awake" and, if your computer, the dock and the freeflyer
are all connected to the ISS network, then after 30 seconds or so you should see
the astrobee you woke up in the "Bee Status" pane.

If any of the above fails, check that your network configuration is correct and
try using the command-line tool to verify the dock functionality.

# Command-line tool

The tool cannot be run if the the service is already running. Before running any
of the commands below, make please use systemctl to make sure that the service
is not running.

You can retrieve help on using the tool in the following way:

    smart_dock_tool -help

Here's what you should see when the EPS is connected to the bus:

    DESCRIPTION
     Command-line tool for interacting with the smart dock
     
    SUBSYSTEM CONTROL
     The tool supports the following subsystem flags
      -power    : Turn on an off power to dock channels
      -led      : Configure dock LEDs to be on, off or flash
      -berth    : View information about a given berth
      -command  : Send a command to a given berth
      -hk       : View dock housekeeping information
      -fault    : View and clear dock fault information
      -string   : View dock build info, software and serial
     For more help about a specific subsystem, add the flag
      eg. smart_dock_tool -power
       
    ONE-SHOT COMMANDS
     The tool supports the following oneshot commands
      -reboot       : Reboot the smart dock
     To use the command, just add the flag
      eg. smart_dock_tool -reboot

The dock controller is largely based off the EPS code, and so the -power, -led,
-hk, -fault and -string switches are similar. The -berth and -command switches
are unique to the dock, and enable the user to respectively get information
about the robot attached to each of the two berths, as well as send one of
several commands to that robot.

For example, to print information about the berths, type the following:

    smart_dock_tool -berth -get

To get a list of all commands that you can send to a robot, type the following:

    smart_dock_tool -command -list

To send a nominal wake up command to the robot on berth 1, type the following;

    smart_dock_tool -command -set wake 1
