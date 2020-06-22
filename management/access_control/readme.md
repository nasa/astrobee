\page access_control Access Control

The access control node is the command gate keeper of the robot. It is responsible for allowing and blocking commands from flowing through the robot. It is also responsible for granting control to operators.

# Taking Control
Only one operator can be in control of the robot at a time. However, any operator can take control at any time. The operator must first sent a request control command. Upon receiving this command, the access control node will generate a cookie and publish it in the access control state. The operator must then send a grab control command with the generated cookie. If using GDS, the Grab Control button will do all of this for the operator.

# Command Pass-Through Rules
The access control node allows all commands issued by the operator in control to flow to the executive. If an operator is not in control, the access control node will ack the command with a bad syntax completion status and a message stating that the operator does not have control. There is one exeception to this rule. Stop and idle propulsion commands are allowed through by any operator at any time.
