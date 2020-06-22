\page pmc_actuator PMC actuator

# How to test the PMC - Using bare C++ program

On the `llp`, run the following program:
```
pmc_actuator_tool -h
```

# How to test the PMC - Using ROS Nodes

## Basic procedure

Open two ssh sessions on LLP

Terminal 1 on LLP:

    source setup.bash
    roslaunch pmc_actuator pmc_actuator.launch

Terminal 2 on LLP:

    source setup.bash
    rosrun pmc_actuator pmc_actuator_cmd_test

**Note:** Detailed procedure to run the sound tests is in the attached document `sound-test-process.txt`.

## Control the type of nozzle motions
`pmc_actuator_cmd_test` performs by default a triangular profile for the nozzles with the blowers at the nominal speed.

**WARNING: The instructions below are obsolete. Run `pmc_actuator_test -h` to check the new command line arguments! Text below is waiting for cleaning...**

The program accept up to 7 arguments that can override these defaults:

1. blower_speed_side1 (pwm value)
2. blower_speed_side2 (pwm value)
3. start_nozzle_position_side1 (pwm value)
4. start_nozzle_position_side2 (pwm value)
5. nozzle_increment_side1 (float)
6. nozzle_increment_side2 (float)
7. time_before_starting_sequence (seconds)

All nozzles on one propulsion module are given the same position and increment/decrement in a triangular pattern. An zero increment will simply keep the nozzle positions static.

For example, to have the left and right nozzles working in opposition at nominal blower speed:

    rosrun pmc_actuator pmc_actuator_cmd_test 207 207 25 96 1 -1 5

Side 1 will start at nozzle position 25 (fully closed) and increment at each cycle (62.5Hz) by 1. When it reaches the maximum value it starts to decrements. Side 2 will start at nozzle position 96 (fully open) and decrements at each cycle by 1. When it reaches the minimum value, it will start to increment. The nozzle positions keep oscillating following a triangular profile between the min and max acceptable nozzle positions.

Command all nozzles closed at 2000rpm:

    rosrun pmc_actuator pmc_actuator_cmd_test 166 166 25 25 0 0

Command all nozzles open at 3000rpm:

    rosrun pmc_actuator pmc_actuator_cmd_test 249 249 96 96 0 0

Triangular pattern with slow open/close:

    rosrun pmc_actuator pmc_actuator_cmd_test 207 207 25 25 0.2 0.2

## Disable the blowers
The PMC firmware cannot put the servo in idle mode. However, the shutdown command can turn off the motor controller on the blowers. So the servos can be controlled with the above commands without the blower spinning by issuing first this command:

    rosservice call /pmc_actuator/enable false

To re-enable the blowers:

    rosservice call /pmc_actuator/enable true

## PWM values

| Blower speed rpm | PWM value |
|-----------------:|:---------:|
| 2000             | 166       |
| 2500             | 207       |
| 2800             | 232       |
| 3000             | 249       |

---

| Nozzle position  | PWM value |
|-----------------:|:---------:|
| Closed           | 25        |
| Open             | 96        |

## Create arbitrary nozzle pattern and blower speeds
The program `pmc_actuator_feeder` allow to describe any sequence of motion using a csv file.

For example, the following command will cycle through each nozzle and open it for a few seconds:

    rosrun pmc_actuator pmc_actuator_feeder ./nozzle_identification.csv

    # File format:
    #   - 1 line per command
    #   - lines starting with # or space are ignored
    #   - 15 fields required, space as separator
    #   - fields:
    #     - time (in seconds, relative to the start of the program)
    #     - side 1 blower speed
    #     - side 2 nozzle position x 6
    #     - side 2 blower speed
    #     - side 2 nozzle position x 6


## Check telemetry from the PMCs
In a terminal with the correct ROS setup, just type:

    rostopic echo /pmc_actuator/telemetry
