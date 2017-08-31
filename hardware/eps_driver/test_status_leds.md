\defgroup eps_driver_test_status_leds Test : Status LEDs
\ingroup eps_driver

The EPS subsystem controls nine light emitting diodes (LEDs),  which flight software can use to communicate status visually to a user. Six of these LEDs communicate programmatic status, and the remaining three communicate whether the camera is streaming, whether the camera is turned on, or whether the microphone is turned. Each LED can be turned on or off; there is no dimming support. The LEDs are enumerated and colored as per the table below.

|       |   |   |        |
|------:|:-:|:-:|:-------|
|       |   | 6 | STREAM |
|       |   | 7 | CAMERA |
|       |   | 8 | MIC    |
| GREEN | 3 | 4 | ORANGE |
| GREEN | 2 | 5 | ORANGE |
| GREEN | 1 | 0 | ORANGE |

The LEDs are configured by a service call to the topic `hw/eps/configure_led_state`, which is optionally prefixed by the name space for the robot on which the EPS is being run. This service call takes the form shown in \ref ff_hw_msgs_ConfigureSystemLeds. For example, assuming the robot is running on name space `/`, to turn LED1 on, LED5 off, and leave the remaining LEDs untouched, type the following on the command line:

    rosservice call /hw/eps/configure_led_state {status0: 0, status1: 1, \
        status2: 0, status3: 0, status4: 0, status5: 2 \
        stream: 0, camera: 0, microphone: 0}

If the flight software stack is not running, the `eps_tool` can be used as a debugging interface to configure the LED values. To do this, first run the tool in the following way:

    rosrun eps_driver eps_tool

Then, select option 11 and follow the instructions.