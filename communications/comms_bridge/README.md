\page coms_bridge Modular Polymorphic ROS Relay


See that dir for architecture info.

This is unidirectional, supporting an arbitrary number of in-out topics.
For bi-directional, an additional paired set of flipped subscribers/publishers can be run.

This is a demonstration of unidirectional relaying of ROS messages from one topic to another, without any specification of the message types, nor even the requirement that the types be available to the nodes at compile-time.

This is a demonstration insofar as the metadata backchannel is itself just ROS topics on the same ROS master, but this can be replaced with other conduits.

## Basic architecture

- The subscriber node (A) subscribes to a specified list of topics
- Upon receipt of the first message on a given forwarded topic, A sends an advertisement message to a metadata "advertisement" topic
- Receiving the metadata advertisement, the republisher node (B) itself advertises the output (republished) topic
- The first and all subsequent messages A receives are serialized and published to a metadata "content" topic
- A delay is implemented in B between its advertising and subsequent republishing of any messages to practically avoid the potential race condition of a subscriber on that end missing the first message as it connects to B only upon seeing the advertisement
- Otherwise, B deserializes all messages received on the "content" metadata topic, looks up the topic in previously received "advertisement" metadata, and republishes the message on the output topic

## Reverse channel

To account for situations in which the republisher node may be restarted after the subscriber node has already been running, or to handle implementations in which advertisement metadata could be lost (eg if the metadata channels were implemented on a lossy medium), the republisher can request retransmission of "advertisement" metadata for any topic.  It does this by publishing a message to a "reset" metadata channel, specifying the topic name, to which the the subscriber node listens and retransmits the advertisement.


