\defgroup gncvisualizer GNC Visualizer
\ingroup tools

This tool visualizes the GNC status of the robot. For using the tool,
see the menu bar when the tool is open. There are many useful hotkeys,
indicated in the menu bar.

With no arguments, the tool listens to DDS topics already being published.
You can choose the communication method by setting the `--comm` argument, 
`ros` and `dds` options are available.

*Next arguments are currenlty only available on ROS mode.

With one of the arguments `--gantry`, `--granite`, `--bag`, and `--sim`, the appropriate
roslaunch file is also started.

Another argument, `--plan`, lets you specify a plan file that can be started by
pressing `p`.

