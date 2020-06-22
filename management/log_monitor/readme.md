\page log_monitor Log Monitor

The log monitor is responsible for printing all the ROS messages in the command line in which it is launched from. This is useful for debugging the robot behaviour is real-time when not every node is launched in the same computer.

# Startup

	rosrun log_monitor log_tool [OPTIONS]

You must specify one of -debug or -info or -warn or -error or -fatal as threshold
The option -only will show only the importance level selected.