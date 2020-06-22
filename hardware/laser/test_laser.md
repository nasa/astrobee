\page laser_test_laser Test: Laser

The laser node is responsible for turning the laser on and off. It contains a test program to do this. The program must be run on the llp. The executable should be found in armhf/bin and is called laser test. To turn the laser on, run:

    ./armhf/bin/laser_test -init -on

To turn the laser off, run:

    ./armhf/bin/laser_test -noinit -off

If running on P4C, please add the following flag: -dev /dev/i2c-0

There are two ways to turn on and off the laser while running the fsw. The first way is by calling the laser enable service from the command line. To turn on the laser this way, run: 

    rosservice call /hw/laser/enable true

To turn off the laser, run:

    rosservice call /hw/laser/enable false

The second way is to send a power on or power off command to the executive. You can either do this by using the teleop command tab in GDS or publishing the command to the /command topic from the command line. To turn on the laser this way, run:

    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "powerOnItem", args: [{data_type: 5, s: Laser Pointer}]}'

To turn off the laser, run:

    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "powerOffItem", args: [{data_type: 5, s: Laser Pointer}]}'
