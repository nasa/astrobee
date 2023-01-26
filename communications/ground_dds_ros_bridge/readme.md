\page ground_dds_ros_bridge Ground DDS ROS Bridge

The ground DDS ROS bridge acts like a translator between DDS and ROS. It listensto DDS telemetry coming from the robot and converts it to ROS messages.
Currently it only supports converting the access control state, guest science
data, dock camera compressed image, navigation camera compressed image, science
camera compressed image, and the robot position messages. It also converts
messages published on the ros command topic to a dds command and publishes them
on the dds command topic. Please note that as of now, the ground dds ros bridge
can only connect to one robot.

The ground bridge will need to be configured for your specific situation. Please
see the next section for more information.

# Configuration

The ground dds ros bridge config file can be found here:

    $ASTROBEE_WS/src/astrobee/config/communications/ground_dds_ros_bridge.config

The following sections explain what may need to be changed in the file.

## Connecting Robot

The ground bridge needs to know the name of the robot you want to connect to.
The default is Bumble. If you need to change this, please open the config file
and find the connecting_robot variable. Please change it to the name of the
robot you wish to connect to. Make sure to capitalize the first letter of the
name.

## Domain id

The domain id for simulation is 37 and this is default value. If you need the
space domain id for use during an activity, please ask your Astrobee point of
contactor.

## Namespace

If you are running the ground bridge in simulation, you will probably want to
use the namespace feature. When set to true, the ground bridge will publish
messages received over DDS to ros using the robot name as the namespace. This
will allow you to distigush topics/messages published by the core flight
software versus topics/messages published by the ground bridge. By default, it
is set to false.

# Running the Ground DDS ROS Bridge

The ground bridge is not run by default in simulation. To run it, you'll need to
set the ASTROBEE_CONFIG_DIR environment variable. In a terminal with the
astrobee setup sourced, run:

ASTROBEE_CONFIG_DIR=$ASTROBEE_WS/src/astrobee/config/ rosrun ground_dds_ros_bridge ground_dds_ros_bridge_node
