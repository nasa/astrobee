\defgroup astrobee Astrobee

# Folder description

The `astrobee` folder is the primary entry point into flight software. For
example, if you run `roslaunch astrobee <launch_file>` you are instructing ROS
to examine the 'launch' directory in this folder for a XML file called
`<launch_file>`, which describes how to start a specific experiment.

1. `config` - This folder holds all of the configuration files for flight
   software.
3. `launch` -  Used to launch the flight software stack. Most nodes export also
   export their own launch and test files.
4. `plans` - This folder holds plans, which are built using the Ground Data
   System (GDS) user interface tool.
5. `resources` - A directory containing all non-LUA resources used by nodes in
   the system. Additional files might be needed depending on the context.
6. `scripts` - A simple bash script to print out the environment variables,
   which can be used to check the context at any point in the launch sequence.

# Environment variables

* `ASTROBEE_RESOURCE_DIR`: An absolute path to all non-LUA system resources that
  are used by the nodes in the system. For example, this includes sparse maps,
  zone files, clutter maps, etc.
* `ASTROBEE_CONFIG_DIR`: An absolute path to all LUA config files that store
  input parameters for the nodes in the system.
* `ASTROBEE_ROBOT`: The class of robot. There must be a LUA file at location
  `$ASTROBEE_CONFIG_DIR/robots/$ASTROBEE_ROBOT.config`. This file defines
  intrinsics, extrinsics, hardware serial numbers and calibration info.
* `ASTROBEE_WORLD`: The world in which we are operating. There must be a LUA
  file at location `$ASTROBEE_CONFIG_DIR/worlds/$ASTROBEE_WORLD.config`. This
  file defines simulation parameters, zone files, ground truth calibration data,
  etc.
* `ASTROBEE_NODEGRAPH`: If set, this environment variables configures all
  ff_nodelets to be spawned in standalone mode. This mode results in a more easy
  to read node and topic structure that.

# Context determination

When launched, nodes must know the context in which they are being launched in
order to operate correctly. By context, we mean (a) the robot class being run,
(b) the world in which the robot is being run, and (c) paths to both a LUA
config and a resource directory. You have flexibility in how this is specified,
but note that we enforce the following strict precedence:

    /etc > environment variable > roslaunch arguments > default roslaunch values

For example, consider this launch process run on your local desktop (in this
case there will be no `/etc/robotname` file set by default)

    export ASTROBEE_ROBOT=p4d
    roslaunch astrobee sim.launch robot:=p4c

Based on the precedence rules, the environment variable will take precedence
over the roslaunch arguments. So, the ROS argument will be completely ignored
and a p4d robot will be launched. We advise that, unless you are doing a battery
of tests with the same context and want to avoid long roslaunch argument lists,
steer clear of environment variables.

The launch is carried out using the hierarchy of launch files shown below. As a
developer you can choose to launch at any points marked with a `[*]`. As a
convention, the context is passed down the hierarchy.

    [sim, replay, mgtf, granite, spawn]        [*]
        -> [sim_start]
        -> [astrobee]                          [*]
            -> [llp]
                -> [node] : [ff_nodelet]       [*]
            -> [mlp]
                -> [node] : [ff_nodelet]       [*]

At the package level, nodes inherit from ff_nodelet, and are launched using a
pattern defined by ff_nodelet.launch. This pattern respects our context
determination hierarchy.

# Default contexts

Assuming no environment variables are set or `/etc` files are created, the
default contexts defined by the launch files are the following:

* `astrobee.launch`: robot = {argument}, world = iss, drivers = true
* `sim.launch`: robot = sim, world = granite, drivers = false
* `mgtf.launch`: robot = p4c, world = mgtf, drivers = true
* `granite.launch`: robot = p4d, world = granite, drivers = true
* `replay.launch`: robot = p4d, world = granite, drivers = false
* `ff_nodelet.launch`: robot = sim, world = granite, drivers = {not applicable}

# Remotely launching nodes

It is possible to launch the MLP, LLP and simulator on remote devices. The
corresponding arguments are `mlp:=<ip>`, `llp:=<ip>` and `sim:=<ip>`. There are
two special values for `<ip>`:

* ip = `local` : launch on the local machine
* ip = `disabled` : disable this machine completely.

The {llp,mlp,sim} arguments with {IP,local,disabled} options is very powerful
and supports any configuration of robot with simulated nodes or hardware in the
loop.

# Launching only specific nodes

It is possible to specify on the command line the set of nodes to be launched
using the `nodes:=<comma_separated_list_of_nodes>` argument.

In this case, only the provided nodes will be launched on their destination
processors (llp or mlp). In addition, it is possible to avoid roslaunch to
perform any connection to a particular processor with the declaration
`{llp,mlp}:=disabled`. This is particularly useful if you need to test some
nodes on one processor and do not have access to the other processor.

For example, to test only the picoflexx cameras on the MLP, not attempting
connection to the LLP (in case it is absent from the test rig):

    roslaunch astrobee granite.launch llp:=disabled nodes:=pico_driver,framestore

# Launch file examples

1. Start a local granite simulation with one p4d robot on namespace '/'

    `roslaunch astrobee sim.launch`

2. Start a local iss simulation with one p4c robot on namespace '/honey'

    `roslaunch astrobee sim.launch world:=iss ns:=honey`

3. Add a second p4d robot to the simulation above on namespace '/bumble'

    `roslaunch astrobee spawn.launch world:=iss ns:=bumble`

4. Do a processor-in-the-loop simulation

    `roslaunch astrobee sim.launch llp:=10.42.0.10 mlp:=10.42.0.11`

5. Launch a simulator remotely but run flight software locally:

    `roslaunch astrobee sim.launch sim:=10.42.0.2`

6. Launch the robot 'honey' from systemd with hardware drivers:

    `roslaunch astrobee astrobee.launch drivers:=true robot:=honey llp:=<ip>`

7. Launch and introspect the epson_imu remotely on some custom llp
```
    roslaunch astrobee astrobee.launch mlp:=disabled llp:=10.42.0.31
        rqt:=true nodes:=epson_imu
```
8. Launch granite table and record key data to ~/.ros:

    `roslaunch astrobee granite.launch record:=true`

9. Replay the granite table data:

    `roslaunch astrobee replay  bag:=~/.ros/some.bag`

10. Test a single camera connected to a local machine. For this to work, the file
astrobee/config/cameras.config may need to be modified, with the nav_cam device
be set from /dev/nav_cam to /dev/video0. Also note that this is known not to
work reliably with VirtualBox. Run:
```
    roslaunch astrobee granite.launch mlp:=local llp:=disabled nodes:=nav_cam,framestore
```

11. To be able to interact with the robot in its environment using a
GUI program, either via teleoperation, or by writing and running a
plan, one can use the Astrobee Ground Control Station (GDS). See
https://babelfish.arc.nasa.gov/trac/freeflyerworkbench/wiki/HowToRunWorkbench
for detailed instructions for how to install and run GDS.

Once GDS is downloaded and extracted, its directory should be renamed
to $HOME/gds/latest (hence that directory must have the executable
`AstroBeeWB`).

One should edit the file NDDS_DISCOVERY_PEERS in the GDS directory
(see the wiki for its location) to specify the bot's mlp ip
address. If running in simulation, one should use 127.0.0.1. (After
this GDS needs to be restarted.)

Here's how to launch the robot in simulation using GDS:

 roslaunch astrobee sim.launch output:=screen world:=iss gds:=true

It is also possible to launch the robot software without GDS, and
start GDS separately. That way more options can be passed to it from
the command line that are not supported by the roslaunch interface.

# Roslaunch, [machine] tags, env scripts and environment variables

The propagation of environment variables through the roslaunch system is not an
obvious one to follow. As an example, consider the roslaunch snippet below,
taken from some file called test.launch.

    <env name="ASTROBEE_CONFIG_DIR"
         value="$(optenv ASTROBEE_CONFIG_DIR /path/b)"/>              [1]

    <machine name="mlp" timeout="10" default="true"
             address="10.42.0.32" user="astrobee" password="astrobee"
             env-loader="/home/astrobee/armhf/env_wrapper.sh"/>       [2]

    <env name="ASTROBEE_CONFIG_DIR"
         value="$(optenv ASTROBEE_CONFIG_DIR /path/d)" />             [3]

    <!-- POINT X -->
    <node pkg="astrobee" type="check_env.sh" name="check_env"/>       [4]

Assume the following line exists in /home/astrobee/armhf/env_wrapper.sh:

    export ASTROBEE_CONFIG_DIR=/path/c

The resulting value of environment variable `ASTROBEE_CONFIG_DIR` is obtained by
understand how roslaunch files are parsed. It turns out that the whole file is
(a) parsed once at the start on the host machine, and (b) environment variables
override and do not propagate. As a result of this, the `check_env.sh` script
sees `ASTROBEE_CONFIG_DIR=/path/a`.

## Takeaway points:

1. Roslaunch files are parsed once before launching takes place.
2. Parsing occurs on the on the host side, and so all calls to env() or optenv()
   obtain values on the host side!
3. Environment variables override all previous values, and their state does not
   persist across `<env>` calls. So, you cannot use `<optenv>` and `<env>` to
   preserve the value of an environment variable within a launch file.
4. Environment variables declare in roslaunch propagate through machine calls,
   and override values specified in the env_wrapper.
