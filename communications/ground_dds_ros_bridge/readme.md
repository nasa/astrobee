\page ground_dds_ros_bridge Ground DDS ROS Bridge

The Ground DDS ROS Bridge acts like a translator between DDS and ROS. It listensto DDS telemetry coming from the robot and converts it to ROS messages.
Currently it only supports converting the access control state, guest science
data, dock camera compressed image, navigation camera compressed image, science
camera compressed image, and the robot position. It will also converts messages
published on the ros command topic to a dds command and publish them on the dds
command topic. Please note that as of now, the ground dds ros bridge can only
connect to one robot.

# Configuration

The ground dds ros bridge config can be found here:

    $ASTROBEE_WS/src/astrobee/config/communications/ground_dds_ros_bridge.config

The following sections explain what may need to be changed in the file.

## Connecting Robot
The ground bridge needs to know the name of the robot you want to connect to.
The default is Bumble. If you need to change this, please open the config file
and find the connecting_robot variable. Please change it to the name of the
robot you wish to connect to 

## Domain id


## Namespace




