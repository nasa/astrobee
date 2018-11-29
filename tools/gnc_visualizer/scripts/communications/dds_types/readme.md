# DDS Profile and Types

This directory contains all xml files used to define the DDS entities needed 
for the visualizer DDS mode.

## Files description

### DDSProfile.xml

This xml defines the QoS Profiles as well as the DDS Entities and Types needed 
for the visualizer. From line 1 to 993, it is a copy from the file located in
*freeflyer/astrobee/config/communications/dds/RAPID_QOS_PROFILES*. From then on 
it defines all entities and types needed.

#### Future improvements

* Get the RAPID_QOS_PROFILES dynamically.
* Get the correct partition name depending on the Astrobee name (Bumble, Honey,
  Queen).

### Free-flyer types

EkfState.xml, GncControlState.xml, GncFamCmdState.xml, LogSample.xml and 
PmcCmdState.xml are custom DDS type definitions for the freeflyer ROS messages. 

Once you build the freeflyer project you should be able to find an update copy
of this files in *freeflyer_build/native/communications/dds_msgs/idl

### Common types

BaseTypes.xml, Header.xml and Message.xml are common types for the free-flyer
ones. This files have been taken from */usr/etc/rtiRoutingService* once the
freeflyer repo was cloned and installed.
