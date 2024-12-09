\page ground_dds_ros_bridge Ground DDS ROS Bridge

The purpose of the ground DDS ROS bridge is to allow guest scientists to use
ros tools on the robot data coming from space. In order to do this, it listens
to DDS telemetry coming from the robot and converts it to ROS messages.
Currently it only supports converting the access control state, guest science
data, dock camera compressed image, navigation camera compressed image, science
camera compressed image, and the robot position messages. 

It also allows guest scientists to command the robot by converting messages
published on the ros command topic to a dds command message and publishes them
on the dds command message topic. Please see the "Commanding the Robot" section
on how to use this feature.

Please note that as of now, the ground dds ros bridge can only connect to one
robot.

The ground bridge will need to be configured for your specific situation. Please
see the next section for more information.

# Configuration

## Configuring the DDS Specific File

DDS will need the ip address of the relay server that is broadcasting the data
from the space robots. Please ask your Astrobee point of contact for this ip
address. Once you have it, open the NDDS discovery peers file found here:

    $ASTROBEE_WS/src/astrobee/config/communications/dds_ground_bridge/NDDS_DISCOVERY_PEERS

Please add the ip address to the end of the discovery peers file and end the
line with a semicolon. NOTE any line added to this file must end in a semicolon.

## Configuring the Ground Bridge Node

The ground dds ros bridge config file can be found here:

    $ASTROBEE_WS/src/astrobee/config/communications/ground_dds_ros_bridge.config

The following sections explain what may need to be changed in the file.

### Connecting Robot

The ground bridge needs to know the name of the robot you want to connect to.
The default is Bumble. If you need to change this, please open the config file
and find the connecting_robot variable. Change it to the name of the robot you
wish to connect to. Make sure to capitalize the first letter of the name.

### Domain id

The default value for the domain id is 37 which is the domian id for the
simulation. Please ask your Astrobee point of contact for the space domain id.

### Namespace

If you are running the ground bridge in simulation, you will probably want to
use the namespace feature. When set to true, the ground bridge will publish
messages received over DDS to ros using the robot name as the namespace. This
will allow you to distigush topics/messages published by the core flight
software versus topics/messages published by the ground bridge. By default, it
is set to false.

# Running the Ground DDS ROS Bridge

To run the ground bridge, you'll need to set the ASTROBEE_CONFIG_DIR environment
variable. In a terminal with the astrobee setup sourced, run:

ASTROBEE_CONFIG_DIR=$ASTROBEE_WS/src/astrobee/config/ roslaunch ground_dds_ros_bridge ground_dds_ros_bridge.launch

# Commanding the Robot

In order to command Astrobee from the ground, you must grab control of the
robot. Grabbing control of an Astrobee is done in a handshake like manor. First,
you request control, Astrobee sends a cookie, and then you use that cookie to
grab control. You can do this either using ros command line tools or creating
a ros node to do it. The steps are listen to the access control state, issue the
request control command, extract the cookie out of the access control state, and
then send the grab control command with the cookie as an argument in the
command. IMPORTANT, Astrobee uses the command source portion of the command to
determine who has control. Be sure to set the command source to the same thing
every time you issue a command or the robot may reject your command. It is also
helpful to set it to a distinguishing name so that it is clear who is commanding
the robot. For instance, ISAAC used "ISAACGroundSystem" as the command source.
The ISAAC team created a node that grabs control of the robot and issues
commands. If you would like to use it as an example, please see:

https://github.com/nasa/isaac/blob/master/communications/ros_gs_bridge/src/ros_gs_bridge.cc
