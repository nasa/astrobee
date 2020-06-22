\page sys_monitor System Monitor

The system monitor is responsible for triggering fault responses and notifying the ground of all the faults occurring on the robot. The system monitor is only responsible for detecting heartbeat faults. All other faults should be detected by the nodes in the system.

# Startup
To be written

# Fault responses
All fault responses are astrobee commands. Each fault id will have a fault response which can be found in a fault table that the system monitor reads in upon startup. See the Fault Table section for more information.

# Triggering Faults
Freeflyer nodelets have the ability to assert faults and will do so upon detecting of a fault. By asserting a fault, the fault will be added to a list of faults in the freeflyer nodelet's heartbeat. The heartbeat will be sent to the system monitor and the system monitor will trigger the fault response of the asserted fault. The system monitor will also add the fault to its state which is sent to the ground.

# Clear faults
To clear faults, a freeflyer nodelet can either use the clear fault or clear all faults functions. These function will publish a need heartbeat message with the fault removed. The system monitor will detect the faults that have been removed and remove them from its state. 

# Blocking faults
Nodes that directly affect the motion of the robot are able to reject commands if faults are occuring. Nodes that don't directly affect the motion need the system monitor to express when a fault has occurred that affects the motion of the robot. These faults are categorized as blocking faults and change the system monitor's state from functional or fault to blocked. Executive will monitor the system monitor's state and transition into the fault operating state if the state is blocked.

# Fault table
The system monitor will read in a table on startup that contains all the information needed to detect heartbeat faults and trigger all faults. The table can be found in astrobee/config/sys_monitor.config. The table is organized by subsystem and then by node making it easy to find and add faults corresponding to a node.

## Entries
Each entry must contain a fault id, whether the fault is a warning, whether the fault is blocking, a response, and a description. Heartbeat detection information, such as time exepected between heartbeats and the number of misses allowed, will only be specified in heartbeat faults entries.

## Responses
A lua function named "command" was developed to help a table modifier add or change a response. The first argument is the name of the command and the remaining arguments are the command arguements. Numerical arguments require a second lua function to help express whether the value is an integer, float, double, long, vector or matrix. Please see the configuration file for more information.
