\defgroup sim Simulation
\ingroup sim

This package provides the code required to simulate multiple Free-Flyers operating within the International Space Station (ISS). Our code is written as plugins for Gazebo, an open source robot simulator. In essence the plugins mimic the ros messages and services provided by real hardware, and as a result they act as drop-in replacements for the true hardware. Running a simulation is therefore as simple as loading all flight software subsystems except the hardware drivers, and spawning simulated hardware in stead of real drivers.

There is once key exception to the rule above: although we do simulate cameras, we do not simulate any vision-based localization. That is, for sparse mapping, optical flow and AR target tracking, in stead of running the feature detectors and localization algorithms on the images produced by the simulator, we draw features from the intersection of random rays cast through a virtual image plane with the environment. This approach provides immense performance gains that enables us to run simulations at many times wall clock speed.

At its core, Gazebo provides a headless server which loads a collection of shared library plugins and performs the simulation itself. One can optionally load the client interface which provides a high-fidelity rendering of the world, and an interface that enables basic interaction with the elements.

Note that if the simulation is run any speed over real-time then the DockCam and NavCam are disabled. We do this because there is large computational overhead required to generate the images: they are wide-angle cameras and hence require multiple projections to be taken and fused, which does not scale well as you increase the requested speed multiplier.

## Obtaining media

If you you are not cross-compiling, then the build process started with "make" will download and extract a correct version of the media automatically for you. This is done by the *description* package, as the media is shared between rviz and Gazebo. To see if the media was installed correctly, check that there are subdirectories in the *description/media* directory.

## Running the Simulator

To launch the simulation without a robot automatically inserted, use the following command:

    roslaunch astrobee sim.launch default:=false

Note that the following arguments are also supported:

To increase the simulation speed to 4x:

    speed:=4

To open an Gviz GUI:

    rviz:=true

To open the Gazebo GUI:

    sviz:=true

To view the GNC GUI:

    gviz:=true

To view the GDS GUI:

    gds:=true

Note that all GUIs have a substantial effect on performance and care should be taken when speeding up the simulation with GUIs enabled.

## Spawning robots in the simulator

To spawn a robot on the default "/" namespace at a given pose:

    roslaunch astrobee spawn.launch pose:="0 0 4.8 0 0 0 1"

You can also supply a namespace to the simulation. Note that we currently only support the empty namespace, "bumble", "queen" and "honey".

    roslaunch astrobee spawn.launch ns=honey pose:="2 0 4.8 0 0 0 1"
    roslaunch astrobee spawn.launch ns=bumble pose:="3 0 4.8 0 0 0 1"
    roslaunch astrobee spawn.launch ns=queen pose:="4 0 4.8 0 0 0 1"

Each robot's ROS topics, services and actions are now prefixed with the supplied namespace, which allows you to interact with them individually. So, for example, to move bumble to a new position you would do the following:

    rosrun mobility teleop -ns bumble -move -pos "4 0"

The teleop, arm_tool and dock_tool all support the -ns flag.

## Auto-spawning robots in a simulation

In many cases you'll only want a single robot spawned on the root namespace, so that you can interact with it either over GDS or through rviz. To do this, simply enable the debug robot:

    roslaunch astrobee sim.launch default:=true rviz:=true

The robot will be on the root namespace and placed at a default location. You should see it appear in rviz within a few seconds of launch.

## Collisions and performance

One of the great sources of computational complexity in simulation is the calculation of collisions between objects. This is especially hard when the collision is a function of two complex meshes. Our simulation optimizes for performance by approximating the Free Flyer's complex meshes with geometric primitives. The Free Flyer collision mesh can be thought of as a sequence of boxes connected by joints, which never self-collide. Collisions are checked between robots and the ISS / Dock meshes, as well as between robots.

## Frame consistency between simulation and perception

Every sensor on the Free Flyer has a pose with respect to the body frame, which we generally call the *extrinsics* of the sensor. Having a good estimate of the true extrinsics is required for localization, control, etc. On a real robot we would run a calibration procedure to estimate for these extrinsics. In simulation we control the extrinsics, which is both advantageous and disadvantageous: we can test the effect of extrinsic estimation error, but we have to ensure that there is consistency between the simulation and perception.

Achieving this consistency is difficult because of technical limitations with. There are several options, each having its own challenges:

1. Inject the correct transforms into the URDF so that the robot is spawned with the extrinsics set correctly -- auto-generation of the URDF is done using xacro. It is difficult to parse LUA variables from within xacro, especially because the robot might only be known at run-time through an environment variable, and not at parse time.
2. Have the plugins read the LUA to extract the transforms -- unfortunately the plugins are loaded into the gazebo server centrally. Since multiple heterogeneous robots might be loaded, the plugin does not know the robot config; it knows the namespace, but not the robot type.
3. Have the plugins listen to TF2 -- since framestore publishes namespace-prefixed transforms for a given Free Flyer, the plugin only needs to know its local frame and parent frame.
4. Just hard-code the transforms into the plugins, or as arguments to the plugins declared in the URDF -- leads to consistency problems.

We opted for (3) above, because it's relatively easy to implement, maintains consistency between simulation and perception, and integrates well with our existing frame transform engine. However, special care had to be taken with the regular, wide andgle and depth cameras. The complexity is handled by the FreeflyerModelPlugin and FreeflyerSensorPlugin abstract classes, provided the correct frame is used as the name for the plugin.

## Development and debugging

If you are actively debugging Gazebo plugins then you probably want to be able to launch the server in debug mode with logging enabled. To do this, change the following line in *simulation/launch/start_simulation.launch* from this...

    <node name="gazebo" pkg="astrobee_gazebo" type="start_server"
          respawn="false" output="screen"
          args="$(arg world) -s libgazebo_system_plugin_server.so" />

... to this ...

    <node name="gazebo" pkg="gazebo_ros" type="debug"
          respawn="false" output="screen"
          args="$(arg world) --verbose -s libgazebo_system_plugin_server.so" />


## Under the hood

There are actually three classes of robot in our world: the ISS, the Dock and the Free Flyer. The first two are simple static meshes, and are inserted once into the simulation so that they show in both rviz and Gazebo.

Every robot in the system is comprised of links and joints. The description of these links and joints are written as URDF to ROS parameters */dock/robot_description*, */iss/robot_description*, */%ns%/robot_description* for the Dock, ISS and FreeFlyer (where %ns% is the namespace). As its name suggests, writing these descriptions is the responsibility of the \ref description package. There is a good reason for having this done outside of the simulator -- in a real experiment, we might want to use KDL or a robot_state_publisher to use them calculate inertia or transform information.

**Abstract classes**

This is the class used for common traits all gazebo plugins have. Each plugin inherits either a *FreeflyerSensorPlugin* or a *FreeflyerModelPlugin*. Those both inherit from freeflyer nodelet so they produce heartbeats like all other freeflyer nodes. Each plugin also has its own queue thread for callbacks, allowing the plugins to subscribe and publish to multiple topics if necessary without any callback issues between plugins all running on the gzserver.

The extrinsics for each sensor is also setup based on the tf2 transform being published for the frame attached to the sensor in it's URDF. If a frame isn't specified, the sensor name will be used to attempt to find a proper transform. For most sensors the pose from tf2 is directly transfered as the sensor pose. However there is a discrepancy between the pose used for the flight software cameras and the pose used by gazebo for cameras. In the flight software a camera frame is defined by z pointing into the camera frame and x pointing the the right. In gazebo a camera is defined with z pointing up and x pointing into the camera frame. To transform from flight software to gazebo a rotation about x is needed followed by a rotation about z.

**Dock plugin**

This plugin does nothing beyond repositioning the dock dynamically in simulation based on the global TF2 transform.

**EPS plugin**

This simulates the dock state produced by the actual EPS. It continually checks how close the Free Flyer is to the dock. When the Free FLyer moves within a threshold distance, a virtual joint is created to lock the Free Flyer to the dock. The EPS then updates the state from DOCKING to DOCKED similar to the hardware driver. This provides sufficient information for the \ref dock behavior to dock and undock a real or simulated Free Flyer. To test

    rosrun dock dock_tool -ns %ns% -dock
    rosrun dock dock_tool -ns %ns% -undock

**Flashlight plugin**

This simulates front and aft facing flashlights. Unfortunately, Gazebo doesn't support model-fixed flashlights. So, for each flashlight a free-floating spot light is created in Gazebo, and the model plugin dynamically positions this light to be rigidly attached to the Free Flyer. To set the brightness of each light you can use the following commands:

    rosservice call /%ns%/hw/light_front/control "brightness: 0"
    rosservice call /%ns%/hw/light_aft/control "brightness: 200"

**Laser plugin**

To toggle the laser model:

    rosservice call /%ns%/hw/laser/enable "enabled: true"

**Perching arm plugin**

The perching arm plugin mimics the real hardware by accepting joint goals and broadcasting joint states to the /joint_goals and /joint_states respectively. In so doing it retains compatibility with the robot_state_publisher, allowing a consistent visualization between Gazebo and rviz. To test:

    rosrun arm arm_tool -ns %ns% -deploy
    rosrun arm arm_tool -ns %ns% -pan 45
    rosrun arm arm_tool -ns %ns% -open
    rosrun arm arm_tool -ns %ns% -stow

**PMC plugin**

This plugin subscribes to the PMC command topic, which includes an impeller speed and nozzle apertures. When a new PMC command is received, this information is passed, along with the battery voltage and current velocity, into a dynamic model that converts this information to a force and torque to apply to the Free Flyer. The dynamic model takes into account the delay between spinning the impellers up and down. This force and torque is applied to the Gazebo model, allowing its physics engine to update the state of the Free Flyer. In addition to publishing basic telemetry (current impeller speed, etc) the PMC plugin also tracks the state of the PMC, thereby realistically simulating ramp-up and ramp-down delay. To test:

    rostopic echo /%ns$/hw/pmc/state
    rostopic echo /%ns$/hw/pmc/telemetry

**NavCam and DockCam plugin**

These are forward and aft-facing greyscale cameras The plugin copies the image data from the gazebo camera sensor into a sensor_msgs::Image message and publishes is to the to the appropriate ROS topic. To test:

    rostopic hz /%ns$/hw/cam_nav
    rostopic hz /%ns$/hw/cam_dock

**HazCam and PerchCam plugin**

These are forward and aft facing depth cameras. The plugin copies the point cloud data from the gazebo depth camera sensor into a sensor_msgs::PointCloud2 message and publishes it to the appropriate ROS topic. To test:

    rostopic hz /%ns$/hw/depth_haz/points
    rostopic hz /%ns$/hw/depth_perch/points

**IMU plugin**

On every update of the gazebo world, this plugin obtains the true angular velocity and linear acceleration of the Gazebo IMU sensor and publishes this information as a sensor_msgs::Imu message on the appropriate ROS topic. To test:

    rostopic echo /%ns$/hw/imu

**Sparse Map plugin**

This publishes camera registration and feature messages for sparse mapping localization. The plugin constructs a virtual image plane and casts rays through randomly-sampled points on this plane. It then calculates the intersection of these rays with the environment to synthetically produce features (image and world coordinate correspondences). A timer is used to introduce an artificial delay between the registration and feature messages being published to reproduce the latency of actual localization. To test:

    rosrun mobility teleop -ns %ns% -loc ml
    rostopic hz /%ns$/loc/ml/registration
    rostopic hz /%ns$/loc/ml/features

**AR Tags plugin**

This publishes camera registration and feature messages for AR marker tracking localization. The plugin randomly samples coordinates on the AR target centered on the dock. It then back-projects these coordinates to a virtual image plane to obtain image coordinates. A timer is used to introduce an artificial delay between the registration and feature messages being published to reproduce the latency of actual localization. To use this the Free Flyer must be in front a dock, facing away from the dock. To test:

    rosrun mobility teleop -ns %ns% -loc ar
    rostopic hz /%ns$/loc/ar/registration
    rostopic hz /%ns$/loc/ar/features

**Ground truth plugin**

This model plugin publishes the truth values of the pose, twist and acceleration values. These can be used to close the control loop in place of localization, allowing use to separate estimation from control error.

    rosrun mobility teleop -ns %ns% -loc gt
    rostopic echo /%ns$/loc/truth/pose
    rostopic echo /%ns$/loc/truth/twist
    rostopic echo /%ns$/loc/truth/accel
