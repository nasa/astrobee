\page comms_bridge DDS Generic Comms Bridge

This is a bidirectional bridge to communicate between different astrobees or ros
clients using DDS to relay the topics. Supporting an arbitrary number of in-out
topics, relaying of ROS messages from one topic to another, without any
specification of the message types, nor even the requirement that the types be
available to the nodes at compile-time.

## Basic architecture

- The subscriber class (A) subscribes to a specified list of topics
- Upon receipt of the first message on a given forwarded topic, A sends an advertisement message to a metadata "advertisement" topic
- Receiving the metadata advertisement, the republisher class (B) itself advertises the output (republished) topic
- The first and all subsequent messages A receives are serialized and published to a metadata "content" topic
- A delay is implemented in B between its advertising and subsequent republishing of any messages to practically avoid the potential race condition of a subscriber on that end missing the first message as it connects to B only upon seeing the advertisement
- Otherwise, B deserializes all messages received on the "content" metadata topic, looks up the topic in previously received "advertisement" metadata, and republishes the message on the output topic
- In this implementation, the metadata advertisement info and content are converted to and passed over dds.

## Enabling DDS Communication

The generic comms bridge runs by default when the astrobee flight starts up. By
default, dds publishing and subscribing is disabled since astrobee to astrobee
communication is not desired for every activity. There are 2 ways to enable it.
One can either change the 'initialize_dds_on_start' parameter in the comms bridge
config file to true before starting flight software or send an "enable astrobee
intercomms" command to astrobee after the flight software has started.

## Specifying Topics

To add topics to be passed between robots, please see the comms bridge config
file. The comments in the config file should explain how to add topics.

## DDS Specifics

### Discovery Peers File

In order to have two or more Astrobees communicate, one must add the ip
addresses to the "NDDS_DISCOVERY_PEERS" file in the "dds_generic_comms" folder.
Be sure to add a semi-colon (;) to the end of every line added.

### QoS Profile

By default, DDS properties are used to ensure the advertisement info messages
are reliably deliveried to the republisher class even if the subscriber or
republisher are restarted. Currently, DDS is configured to work if there are
no more than 20 topics being passed between robots. If more topics are required,
please change the message depth in the "RAPID_QOS_PROFILES.xml" file in the
"dds_generic_comms" folder. The depth for the "RapidReliableDurableQos" will
need to be changed for both the "datareader_qos" and the "datawriter_qos". This
should be on or around lines 206 and 218 in the file. You will also need to
change the value check in the "comms_bridge_nodelet.cpp" file on or around
line 307.
