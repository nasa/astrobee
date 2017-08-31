\defgroup choreographer Choreographer
\ingroup mobility
The choreographer node's primary responsibility is to accept and carry out motion requests from client nodes. In most cases, the client will be one of the executive, docking or perching nodes. Requests are packaged as [ROS actions](http://wiki.ros.org/actionlib), and so we will use their nomenclature throughout the remainder of this page. 

There are three main messages used in a ROS action:

* `Goal` - A message sent from the client to the server containing instructions.
* `Feedback` - Information sent periodically from the server to client containing progress data.
* `Result` - The final result of the action, as well as any final data.

One of the key advantages over ROS services is that actions are asynchronous, or non-blocking. This means that -- as a client -- you can send off a goal as a background task, and be called back when there is feedback or a final result.

Although the action library allows for more complex implementations in which goals can be queued or executed in parallel, we are using a ```simple action server and client``` model. This model assumes that only one action can be processed at any time, and that a new goal preempts any previous goal (the existing goal will issue a result with a preemption response, so that the its calling client knows that it was preempted by some other client).

## Conceptual overview

Our software supports a number of different mobility requests -- ```executing``` an existing plan, ```moving``` through a sequence of timestamped poses, ```holding``` a given position, ```stopping``` as quickly as possible, and ```idling``` the propulsion system. Each request has different data in the payload, and so it makes sense to support multiple action types:

* `/mobility/choreographer/execute` - Validate and then execute a precomputed segment. This will normally be a segment computed using the Ground Station, and passed in by executive.
* `/mobility/choreographer/move` - Plan and execute a path through a sequence of timestamped poses. The poses are passed to a planner implementation, and then validated for correctness before being executed.
* `/mobility/choreographer/stop` - Stop any current motion. The exact stopping point will by unknown. However, the FreeFlyer can be manually repositioned, and it will hold that new position.
* `/mobility/choreographer/idle` - Null all the rates. Should only be used if stop fails to work.
* `/mobility/choreographer/hold` - Move to and hold a given pose. If the FreeFlyer is perturbed, it will attempt to move back to the desired pose.

A consequence of this design choice is that each new goal must preempt the four other action types offered by the choreographer, in addition to its own action. For example, if the choreographer is busy handling an execute goal, a new move goal will first preempt the execute goal before becoming active. This necessary behaviour convolutes the programmatic flow.

The bulk of the work actually occurs in the planner, sentinel and GNC wrapper nodes, with the choreographer being the glue that binds them all together. For this reason you will notice that the feedback and result messages for all actions are really just wrappers around the generic  ```ff_msgs/MobilityProgress``` and ```ff_msgs/MobilityResult``` types. With the minor exceptions described in the subsections that follow, the choreographer really just acts a proxy that passes passes on feedback from the planner / GNC wrapper to the callee, depending on the stage of execution, and gracefully handles successes, preemptions and failures of it peer nodes.

![alt text](../images/mobility/detail.png "Interaction between mobility modules")

### Deferred planning and **control** mode

In the case where a user might want to generate a segment but not have it actually execute on the robot. This is useful in two ways -- (1) it acts like an on/off switch for control, and (2) is enable a client to test the feasibility of a motion request without actually executing it.

### Time synchronization and **immediate** mode

In most cases users will want the robot to execute their given motion request as soon as possible. However, there are cases where it makes sense to defer execution to some future point in time (multi-robot coordination, etc.). The **immediate** mode prevents you from having to calculate the exact start time for an execute or move request so that it occurs immediately. At the last stage of the choreographer pipeline, the initial set point timestamp is manipulated to be the current time, so that GNC execution begins immediately.

### Tolerance monitoring and **bootstrapping** mode

GNC continually checks that the segment it receives from the choreographer is executed to within a specific pose and twist tolerance. These tolerances are determined by the flight mode. In all cases if tolerance is breached then GNC stops the robot and returns an error to the choreographer.

In an experimental environment there is a limited accuracy to which you can manually position the robot to lie on the first set point of a segment. The ```bootstrapping``` mode allows you to start a segment with the robot placed anywhere. If this mode is enabled, the choreographer will first try to execute the segment. If the GNC immediately yields a tolerance failure, it means that the current position of the robot is different from the first set point in the segment GNC was passed. The choreographer will then generate a segment from the current pose to the initial set point, and append the resulting segment to the input segment before passing it to GNC again.

### Obstacle detection and **replanning** mode

The role of the sentinel node is to detect upcoming collisions and calculate the time until the collision takes place. If this duration is greater than the **collision horizon** -- a value twice as large as the permissible time required to generate and validate a segment (which we call eht **replanning horizon**), then replanning can take place. If replanning is allowed, a future handover time is calculated -- this is the time exactly halfway between the replanning and collision horizons, and represents the point where the old segment will handover to the new segment. The choreographer generates and validates a new segment that moves from this setpoint at this time to the end point of the original segment.

## Configurable options

The choreographer exposes its configuration through ```common::ConfigServer``` class. Thus, the ```rqt_reconfigure``` client can be used to change settings manually, or the ```common::ConfigClient``` can be used to change the settings programatically.

The following configurable parameters are supported:

| Parameter                   | Description |
|:--------------------------- |:----------- |
| ```generate_timeout```      | How long to wait for planner generation action feedback before considering the peer as non-responsive |
| ```validate_timeout```      | How long to wait for planner validation action feedback before considering the peer as non-responsive |
| ```watch_timeout```         | How long to wait for sentinel watch action feedback before considering the peer as non-responsive |
| ```control_timeout```       | How long to wait for GNC control action feedback before considering the peer as non-responsive |
| ```enable_control```        | Whether control should actually be applied. This is useful for generating segments for use later |
| ```enable_immediate```      | Whether the segment should be executed immediately, therefore ignoring the first set point time stamp | 
| ```enable_replanning```     | Whether the choreographer should replan around upcoming obstacles that are detected by the sentinel. |
| ```enable_bootstrapping```  | Whether the choreographer should be allowed to plan a segment that moves it to the first set point |

## Under the hood

The behaviour of the choreographer node is described graphically in the diagram below. Green boxes indicate the arrival of a new goal for each of the five actions respectively. The red boxes denote peer nodes, while the grey lines are requests made by the choreographer on remote action servers running on the peer nodes. The white boxes show what typically happens when a new request is received, while the orange and yellow boxes denote programmatic flow for the case of replanning and bootstrapping respectively.

![alt text](../images/mobility/choreographer.png "How the choreographer node works")

Every incoming goal results in the preemption of any goal currently running on any of the action servers offered by the choreographer. In the case of a ```move``` action, the choreographer generates a segment using the planner. This segment is validated for correctness, before being packaged as a command. The ```execute``` and ```hold``` actions behave similarly, but the segment is provided in the goal, rather than by the planner. The ```idle``` and ```stop``` commands are much simpler, as they don't require a segment to be specified. 

All five cases reduce to a command that ultimately gets packaged in a goal request to GNC. Before this is done, however, a watch action is started on the sentinel to start the collision detection system. Once the control command is passed to GNC the choreographer does nothing but ensure that feedback is continually received, and that this feedback is forwarded to the action callee.

If at any point a peer node yields an error or fails to become active, send feedback or a result within some timeout, a stop command is sent to GNC and an error is returned as a response to the action. The only exception to this rule is for handling boostrapping mode, where an error from GNC is used to determine if the platform is located at the first set point.

