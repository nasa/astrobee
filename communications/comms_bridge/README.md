\page comms_bridge DDS Generic Comms Bridge


This is bidirectional bridge to communicate between different astrobees or ros clients using DDS to relay the topics. Supporting an arbitrary number of in-out topics, relaying of ROS messages from one topic to another, without any specification of the message types, nor even the requirement that the types be available to the nodes at compile-time.

## Basic architecture

- The subscriber class (A) subscribes to a specified list of topics
- Upon receipt of the first message on a given forwarded topic, A sends an advertisement message to a metadata "advertisement" topic
- Receiving the metadata advertisement, the republisher class (B) itself advertises the output (republished) topic
- The first and all subsequent messages A receives are serialized and published to a metadata "content" topic
- A delay is implemented in B between its advertising and subsequent republishing of any messages to practically avoid the potential race condition of a subscriber on that end missing the first message as it connects to B only upon seeing the advertisement
- Otherwise, B deserializes all messages received on the "content" metadata topic, looks up the topic in previously received "advertisement" metadata, and republishes the message on the output topic
