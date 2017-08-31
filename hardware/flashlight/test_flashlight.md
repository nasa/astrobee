\defgroup flashlight_test_flashlight Test: Flashlight
\ingroup flashlight

The flashlight node is responsible for turning the flashlights on and off and setting the brightness. It contains a test program to do this without running the fsw. The program must be run on the llp. The executable can be found in armhf/bin and is called flashlight test. To turn the front flashlight on, run:

    ./armhf/bin/flashlight_test -dev /dev/i2c-1 -addr 0x60 -init -on -br 100

To turn the front flashlight off, run:

    ./armhf/bin/flashlight_test -dev /dev/i2c-1 -addr 0x60 -noinit -off

To turn the back flashlight on, run:

    ./armhf/bin/flashlight_test -dev /dev/i2c-1 -addr 0x63 -init -on -br 100

To turn the back flashlight off, run: 

    ./armhf/bin/flashlight_test -dev /dev/i2c-1 -addr 0x63 -noinit -off


If running on P4C, please change the dev flag from /dev/i2c-1 to /dev/i2c-0

There are two ways to set the flashlight brightness in the fsw. The fsw is set up such that setting brightness to 0 turns the flashlight off and setting the brightness anywhere from 1 to 200 turns the flashlight on. The first way is by calling the set flashlight service from the command line. To turn the front flashlight on, run:

    rosservice call /hw/light_front/control 100

To turn the front flashlight off, run:

    rosservice call /hw/light_front/control 0

To turn the back flashlight on, run:

    rosservice call /hw/light_aft/control 100

To turn the back flashlight off, run:

    rosservice call /hw/light_aft/control 0

The second way is to send a set flashlight brightness command to the executive. You can either do this by using the teleop command tab in GDS or publishing the command to the /command topic from the command line. To turn the front flashlight on, run:

    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setFlashlightBrightness", subsys_name: "Astrobee", args: [{data_type: 5, s: Front}, {data_type: 2, f: 100}]}'

To turn the front flashlight off, run:

    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setFlashlightBrightness", subsys_name: "Astrobee", args: [{data_type: 5, s: Front}, {data_type: 2, f: 0}]}'

To turn the back flashlight on, run:

    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setFlashlightBrightness", subsys_name: "Astrobee", args: [{data_type: 5, s: Back}, {data_type: 2, f: 100}]}'

To turn the back flashlight off, run:

    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setFlashlightBrightness", subsys_name: "Astrobee", args: [{data_type: 5, s: Back}, {data_type: 2, f: 0}]}'
