\defgroup mapper Mapper
\ingroup mobility

The \ref mapper node is responsible for maintaining a representation of the environment, which is in turn used by the \ref planner node for planning segments. Internally, the \ref mapper stores a collection of keep-in and keep-out zones, as well as a map of the environment, which it builds by incrementally fusing measurements from \picoflexx sensor(s). Ultimately, both representations are reduced by the \ref mapper to a collection of free-space polygons, in which the \ref planner searches for a suitable path.

# Keep-in and keep-out zones

The keep-in and keep-out zones describe safe and unsafe areas for flight respectively. Each zone is a cuboid in 3-space, and is fully-defined by the coordinates of two diagonally opposite vertices. The union of all keep-in zones minus the union of all keep-out zones describes the free space in which safe flight can occur. The default zones are provided as JSON formatted files with suffix `.json` in the `zones` directory of the \ref astrobee folder.

An example of a keep-in zone JSON file might look like this:

	{
	  "sequence" : [ [ -1.0, -1.0, -3.0, 1.0, 1.0, 3.0 ]  ] ,
	  "dateCreated" : "1475516840",	
	  "notes" : "Keep-in zone",
	  "author" : "John Smith",
	  "name" : "GraniteTableWorkspace",
	  "safe" : true
	}

Whereas, a keep-out zone JSON file might contain this:

	{
	  "sequence" : [ [ -1.0, -0.3, -3, -0.6, 1.0, 3.0 ], [0.3,-0.3,-3.0,1.0,0.3,3.0] ] ,
	  "dateCreated" : "1475516840",
	  "notes" : "Keep-out zone",
	  "author" : "John Smith",
	  "name" : "GraniteTableDockAndClutter",
	  "safe" : false
	}

Note that the "sequence" field takes an array of 6-vectors, and not just a single 6-vector. Each element of this array represents a zone, with each vector denoting the two coordinates that fully-define the cuboid. A consequence of this design choice is that multiple keep-in or keep-out zones can be specified in a single file (but you cannot mix keep-outs and keep-ins in one file).

After loading and parsing these JSON files, the resulting data structure is serialized into a binary file called `0.bin`. The digit zero tells the \ref mapper system that the binary structure contains the default set of zones. At any point an operator can upload a new set of zones using the `SetZones` service call on the ROS namespace `~/mob/mapper/set_zones`. Calling this service will result in the creation of a new file `%lu.bin` in the `zones` file, where `%lu` is a unsigned long Unix timestamp for the zones. When the \ref mapper node is next started, it will search for the `*.bin` file with the latest timestamp, and load it by default.

Please refer to the definition of \ref ff_msgs_SetZones for more information on how to update zones using a ROS service call.

The \ref mapper node publishes [visualization_msgs::MarkerArrays](http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/MarkerArray.html) on the namespace `~/mob/mapper/zones`. When listened to in rviz, these marker arrays render keep-in and keep-out zones as semi-transparent green and red cuboids. The two example zones should be rendered as shown below:

![alt text](../images/mobility/zones.png "How the RVIZ user interface draws zones")

# Environmental mapping

Work in progress -- this shows intent rather than the current state.

The \ref mapper builds a representation of the environment (clutter topology) using measurements from the \ref picoflexx depth sensor, using a mapping service (OctoMap / VoxelGrid TBD). Mapping is done in the world frame, and the TF2 library is used to transform from the sensor frame to the world frame based on dynamic transforms published by the EKF, and static transforms published by \ref framestore.