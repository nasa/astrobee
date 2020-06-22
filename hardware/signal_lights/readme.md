\page signal_lights Signal light driver

The signal_lights driver controls the signal lights on the port and starboard
PMC. Since the exact behavior of the signal lights is a work-in-progress, this
proof of concept driver just illustrates full stack control of the lights.

Like many other drivers, the signal lights are controlled by a proxy library,
called 'signal_lights' which acts as an intermediary between high level calls
and i2c transactions.

The current ICD supports 5 programs, two of which are implemented:

  - 0: Not available yet
  - 1: Not available yet
  - 2: Not available yet
  - 3: Turn on all pixels to a specified color.
  - 4: Not available yet
  - 5: Turn on a single light at the index.

# Using the commandline tool

The color of the light is specified with the red (-r), green (-g) and blue (-b)
switches. Acceptable indexes (-d) run from 0 - 43 inclusive. The program is
selected with -p and the mode is selected with -m. The mode should always be
the default value of 1 unless s shutdown is required.

To turn all lights blue, type the following

    rosrun signal_lights signal_lights_tool -r 0 -g 0 -b 128 -p 3

To turn the light at index 5 (the sixth LED) type the following

    rosrun signal_lights signal_lights_tool -r 0 -g 128 -b 0 -p 5 -d 5

# Using the ROS interface

The signal_lights nodelet provides a single service-based interface for control,
located at /hw/sig/config. It takes the same arguments as the commandline tool,
where the -d switch maps to the 'arg' message variable.

For example, this turns LED at index 4 to white:

    rosservice call /hw/sig/config "{mode: 1, program: 5, brightness: 0, red: 128, green: 128, 
      blue: 128, arg: 4}" 
