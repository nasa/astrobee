\page ff_util Utility code

This package contains modular code blocks that are used throughout the flight software.

# Configuration client (config_client) and server (config_server)

These two classes are essentially C++ interfaces to [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure). As its name suggests, dynamic_reconfigure allows one to update parameters on a node at runtime. One great feature of this package is that it provides a Qt-based GUI for interacting with your node, which is assembled at runtime from your configuration specification.

The config server takes its parameters, metadata and default values from a LUA block. It then broadcasts the parameters and current values using dynamic_reconfigure messages. Any node in the system can use a ConfigClient to get or set parameters on a given node that supports this behavior. The parameter values will be preserved until overwritten or the system restarts.

The parameter values are cashed to rosparam values on roscore, which means that if a node dies and respawns the configuration state is not lots.

# Finite state machine (ff_fsm)

This class provides skeleton code for implementing a finite state machine, which is a frequently-used paradigm for asynchronous state-based code. The class allows you to declare lambda functions (arbitrary blocks of code that are executed in the node's context) representing event-triggered state transitions. Once the finite state machine is initialized with a starting state, then the calling code simply pushes a sequence of events to the machine, and it takes care of maintaining the overall system state.

The class also provides a callback when the internal state changes. This can be used to trigger the production of action client feedback, or print the state transitions as debug output.

# Name database (ff_names)

The name database ensures that we are naming our subsystems, topics, services, actions, nodes and frames consistently across all of the flight software code. 

# Nodelet (ff_nodelet)

[ROS nodelets](http://wiki.ros.org/nodelet) provide a way of reducing the transport overhead between nodes running on the same processing core. For large messages like images or point clouds, the memory copying, serialization and deserialization overhead is substantial. To circumvent this, the nodelet package provides a means of launching nodes in the same process, allowing them to exchange pointers to internal message structures rather than serializing the message across a loopback interface.

The FreeFlyerNodelet class augments the Nodelet class with the following Astrobee-specific functionality:

1. It adds a heartbeat to every node in the system. Thus, if a node seizes or crashes then this event will be detected by the system monitor, and will possibly result in third-party intervention by the executive node.
2. It provides a convenient mechanism to assert faults. By virtue of the fact that it inherits functionality from this class, every flight software node can call AssertFault(...) at any point to signal that it has encountered a problem. The fault is encoded in the heartbeat and collected centrally.
3. It provides convenient function calls to get the name of the node and the robot on which it is running. This allows one to select configuration blocks based on node name, which turns out to be very useful when using a common driver for two different sensors.
4. It provides an optional mechanism for sending diagnostics.

# Serialization (ff_serialization)

This class provides a mechanism for serializing and deserializing ROS messages to binary files. This is essence provides similar functionality to rosbag but for use within code. An example use-case might be saving and loading calibrates parameters for a specific node.

# Actions (ff_action)

Although it is tempting to use the request-response communication model provided by ROS services, there are many reasons why one should not. Firstly, it is a blocking mechanism, which means that all other callbacks in the same node (or nodelet) will be deferred until the response completes. Secondly, services do not handle multiple requests gracefully, nor do they allow for intermediary feedback to be sent to the user.

[ROS actions](http://wiki.ros.org/actionlib) provide a superior request-response communication model, at the expense of greater complexity.They are asynchronous and state-based, which means that -- as a client -- you can send a goal as a background task, and receive a callback when feedback or a result arrives. It also means that the server can accept multiple goals and offer preemption if required.

There are three main messages used in a ROS action:

* `Goal` - A message sent from the client to the server containing instructions.
* `Feedback` - Information sent periodically from the server to client containing progress data.
* `Result` - The final result of the action, as well as any final data.

Despite their numerous advantages, there are several drawbacks with ROS actions. Firstly, there is no timeout management for connecting to a service, sending goals, receiving feedback or a result. Secondly, their constructor assumes the existence of an active ROS handle, so they cannot be use as member variables on a nodelet without a segmentation fault occurring. Fixing these deficiencies on a per-use basis requires substantial boilerplate code.

The FreeFlyerActionClient and FreeFlyerActionServer classes are essentially wrapper aroud ROS simple action client and servers respectively, providing fixes to the problems above in a reusable way.

# Services (ff_services)

Similar to FreeFlyerAction, the FreeFlyerServiceClient augments ROS services with various timeouts, preventing unnecessary boilerplate code.

