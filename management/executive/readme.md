\defgroup executive Executive
\ingroup management

The executive is responsible for keeping track of the operating and mobility states of the robot. It uses these states to accept or reject commands it receives. Please see https://babelfish.arc.nasa.gov/svn/freeflyer_docs/FlightSoftware/Software.xlsx for possible operating-mobility state combinations and which commands are accepted in the state combinations. The executive is also responsible for  for forwarding accepted commands to the correct nodes and informing the ground and system on the status of the command.

# Commanding
Except for the request and grab control commands, all commands go through the executive. Commands can come from the ground, guest science primary applications, and the system monitor.

# Fault Operating State
The executive will go into the fault operating state when executing a command from the system monitor. Once the command is finished, it will transiting back it the ready state. It will also go into the fault operating state when the system monitor changes it state to blocking. In this case, the executive will allow the commands currently executing to finish. If a plan is executing, the current station or segment will finish and then the plan will be paused. Most, if not all, mobility commands will be rejected until the fault blocking the system goes away. Please see the system monitor documentation for more information.

# Heartbeat Monitor
The executive monitors the system monitor's heartbeat. The executive will treat a missing system monitor heartbeat like it treats a blocking system monitor state. Please see the Fault Operating State section for more information. 
