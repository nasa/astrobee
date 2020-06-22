\page eps_driver_test_battery_status Test : Battery status

The EPS subsystem is responsible for managing up to four rechargeable batteries, each of which operates on its own channel. At least one battery must be inserted to power the robot, with each subsequent battery extending the operating lifetime at the cost of some additional mass.

The EPS subsystem tracks the remaining capacity, current voltage and current of each battery. This information is continually broadcast to the topic `hw/eps/battery/state`, which is optionally prefixed by the name space for the robot on which the EPS is being run. Assuming the robot is running with name space `/`, the EPS telemetry can be displayed using the following command:

    rostopic echo /hw/eps/battery/state

If the flight software stack is not running, the `eps_tool` can be used as a debugging interface to configure the LED values. To do this, first run the tool in the following way:

    rosrun eps_driver eps_tool

Then, select option 10 and follow the instructions.