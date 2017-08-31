\defgroup sentinel Sentinel
\ingroup mobility

As its name suggests, the responsibility of the sentinel node is to look out for the safety of the platform. Unlike the \ref planner, which depends on map-building to proactively plan a safe path through a representation of the world, the sentinel node reacts to sensed data to check if the platform is likely to collide with an obstacle over some fixed time horizon, based on the segment is it currently following.

![alt text](../images/mobility/sentinel.png "Collision avoidance")

When the choreographer begins executing a segment, it starts a parallel action request to the sentinel node, which signals to the sentinel node that it must start watching for collisions. When an upcoming collision is predicted, this information is sent as feedback to the choreographer. Depending on the time to collision, the choreographer might choose to either stop the platform or to replan a new segment to get to the desired station.

The sentinel node subscribes to point cloud measurements from the Hazard Camera (hazcam). The point clouds are represented in the optical frame, and can be easily transformed into the body frame, since the camera is rigid with respect to the body frame. The problem now reduces to finding -- if it exists -- the first position along the segment that comes within some threshold distance of any point in the cloud.

Recall that each set point in a segment defines the position, velocity and acceleration at that point in time, which allows us to propagate the state forward in time (a trajectory). This trajectory is valid for the period of time up until the next set point, which by definition should like on the trajectory. 

For computational efficiency we carry out collision detection in the following way:
# We reduce the complexity of the point cloud 
# We perform all computation in the sensor frame

By performing collision detection in the sensor frame, we avoid

== Operation ==
