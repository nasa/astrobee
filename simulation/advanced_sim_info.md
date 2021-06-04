/page advanced-sim-info Advanced Simulation Information

# Rviz in Localization Only Mode

To run only localization nodes, and visualize localization running on a rosbag containing only imu data and NavCam images, run with the loc_only parameter.  This can be done as described below:
1: Move a map file for your chosen world to astrobee/resources/rviz.  For example, astrobee/resources/rviz/iss.map
2: Launch rviz using the loc_only command.  For example: 'roslaunch astrobee sim.launch default:=false rviz:=true loc_only:=true'
3: In a separate command prompt, launch the astrobee localization software, and provide a global path to your rosbag.  For example: 'roslaunch astrobee astrobee.launch robot:=bumble world:=iss loc_only:=true bag:=/path/to/bag/MyBag.bag'
There are several visualizations that can be enabled in the file astrobee/config/localization.config.  matched_features_on enables a live camera feed with all sparse mapping features in use overlayed over each frame.  all_features_on enables a live camera feed with all detected features in each frame overlayed.  map_cloud_on draws the entire stored sparse map in rviz.

# Collisions and performance

One of the great sources of computational complexity in simulation is the calculation of collisions between objects. This is especially hard when the collision is a function of two complex meshes. Our simulation optimizes for performance by approximating the Free Flyer's complex meshes with geometric primitives. The Free Flyer collision mesh can be thought of as a sequence of boxes connected by joints, which never self-collide. Collisions are checked between robots and the ISS / Dock meshes, as well as between robots.

# Frame consistency between simulation and perception

Every sensor on the Free Flyer has a pose with respect to the body frame, which we generally call the *extrinsics* of the sensor. Having a good estimate of the true extrinsics is required for localization, control, etc. On a real robot we would run a calibration procedure to estimate for these extrinsics. In simulation we control the extrinsics, which is both advantageous and disadvantageous: we can test the effect of extrinsic estimation error, but we have to ensure that there is consistency between the simulation and perception.

Achieving this consistency is difficult because of technical limitations. There are several options, each having its own challenges:

1. Inject the correct transforms into the URDF so that the robot is spawned with the extrinsics set correctly -- auto-generation of the URDF is done using xacro. It is difficult to parse LUA variables from within xacro, especially because the robot might only be known at run-time through an environment variable, and not at parse time.
2. Have the plugins read the LUA to extract the transforms -- unfortunately the plugins are loaded into the gazebo server centrally. Since multiple heterogeneous robots might be loaded, the plugin does not know the robot config; it knows the namespace, but not the robot type.
3. Have the plugins listen to TF2 -- since framestore publishes namespace-prefixed transforms for a given Free Flyer, the plugin only needs to know its local frame and parent frame.
4. Just hard-code the transforms into the plugins, or as arguments to the plugins declared in the URDF -- leads to consistency problems.

We opted for (3) above, because it's relatively easy to implement, maintains consistency between simulation and perception, and integrates well with our existing frame transform engine. However, special care had to be taken with the regular, wide angle and depth cameras. The complexity is handled by the FreeflyerModelPlugin and FreeflyerSensorPlugin abstract classes, provided the correct frame is used as the name for the plugin.

# Development and debugging

If you are actively debugging Gazebo plugins then you probably want to be able to launch the server in debug mode with logging enabled. To do this, change the following line in *simulation/launch/start_simulation.launch* from this...

    <node name="gazebo" pkg="astrobee_gazebo" type="start_server"
          respawn="false" output="screen"
          args="$(arg world) -s libgazebo_system_plugin_server.so" />

... to this ...

    <node name="gazebo" pkg="gazebo_ros" type="debug"
          respawn="false" output="screen"
          args="$(arg world) --verbose -s libgazebo_system_plugin_server.so" />

# Changing Simulation properties

All the simulation physical details are contained in the description folder.


**Changing the mass/inertia of the robot**

    description/description/model.urdf.xacro
    - Here is the parent file of all details related to the Astrobee's configurations in the simulation. You can change the properties of the base robot.

    description/description/macro_perching_arm.urdf.urdf.xacro
    - Here you can change the properties of the arm.

    description/description/model_carriage.urdf.urdf.xacro
    - When testing the simulation in the granite table, one can change the carriage properties, namely the kp, kd, mu1, mu2 and fdir

Customization: mass; origin (center of mass); inertia

**Changing the sensor's properties**
    All the Plugins that are contained in this folder are called from the model .xacro files in description/description, any change in the sensor parameters should be done there (explored further in next section).

# Under the hood

There are actually three classes of robot in our world: the ISS, the Dock and the Free Flyer. The first two are simple static meshes, and are inserted once into the simulation so that they show in both rviz and Gazebo.

Every robot in the system is comprised of links and joints. The description of these links and joints are written as URDF to ROS parameters */dock/robot_description*, */iss/robot_description*, */%ns%/robot_description* for the Dock, ISS and FreeFlyer (where %ns% is the namespace). As its name suggests, writing these descriptions is the responsibility of the \ref description package. There is a good reason for having this done outside of the simulator -- in a real experiment, we might want to use KDL or a robot_state_publisher to use them calculate inertia or transform information.

**Abstract classes**

This is the class used for common traits all gazebo plugins have. Each plugin inherits either a *FreeflyerSensorPlugin* or a *FreeflyerModelPlugin*. Those both inherit from freeflyer nodelet so they produce heartbeats like all other freeflyer nodes. Each plugin also has its own queue thread for callbacks, allowing the plugins to subscribe and publish to multiple topics if necessary without any callback issues between plugins all running on the gzserver.

The extrinsics for each sensor is also setup based on the tf2 transform being published for the frame attached to the sensor in it's URDF. If a frame isn't specified, the sensor name will be used to attempt to find a proper transform. For most sensors the pose from tf2 is directly transferred as the sensor pose. However there is a discrepancy between the pose used for the flight software cameras and the pose used by gazebo for cameras. In the flight software a camera frame is defined by z pointing into the camera frame and x pointing the the right. In gazebo a camera is defined with z pointing up and x pointing into the camera frame. To transform from flight software to gazebo a rotation about x is needed followed by a rotation about z.

**Dock plugin**

This plugin does nothing beyond repositioning the dock dynamically in simulation based on the global TF2 transform.

**EPS plugin**

This simulates the dock state produced by the actual EPS. It continually checks how close the Free Flyer is to the dock. When the Free Flier moves within a threshold distance, a virtual joint is created to lock the Free Flyer to the dock. The EPS then updates the state from DOCKING to DOCKED similar to the hardware driver. This provides sufficient information for the \ref dock to dock and undock a real or simulated Free Flyer. To test

    rosrun dock dock_tool -ns %ns% -dock
    rosrun dock dock_tool -ns %ns% -undock

Customization: description/description/model_eps.urdf.xacro
    rate:     Rate at which dock state checked
    distance: Threshold distance for magnetism
    delay:    Delay between contact and docking

**Flashlight plugin**

This simulates front and aft facing flashlights. Unfortunately, Gazebo doesn't support model-fixed flashlights. So, for each flashlight a free-floating spot light is created in Gazebo, and the model plugin dynamically positions this light to be rigidly attached to the Free Flyer. To set the brightness of each light you can use the following commands:

    rosservice call /%ns%/hw/light_front/control "brightness: 0"
    rosservice call /%ns%/hw/light_aft/control "brightness: 200"

Customization: description/description/model_flashlights.urdf.xacro
    rate:   doesn't do anything (should this be removed?)
    width:  width of the light
    height: height of the light
    depth:  depth of the light

**Laser plugin**

To toggle the laser model:

    rosservice call /%ns%/hw/laser/enable "enabled: true"

Customization: description/description/model_laser.urdf.xacro
    rate:  doesn't do anything (should this be removed?)
    range: range of the laser
    width: width of the laser

**Perching arm plugin**

The perching arm plugin mimics the real hardware by accepting joint goals and broadcasting joint states to the /joint_goals and /joint_states respectively. In so doing it retains compatibility with the robot_state_publisher, allowing a consistent visualization between Gazebo and rviz. To test:

    rosrun arm arm_tool -ns %ns% -deploy
    rosrun arm arm_tool -ns %ns% -pan 45
    rosrun arm arm_tool -ns %ns% -open
    rosrun arm arm_tool -ns %ns% -stow

Customization: description/description/macro_perching_arm.urdf.xacro
    rate:     rate at which the arm joints are published
    proximal: initial position of the proximal joint
    distal:   initial position of the distal joint
    gripper:  initial position of the gripper (0 is closed)

**PMC plugin**

This plugin subscribes to the PMC command topic, which includes an impeller speed and nozzle apertures. When a new PMC command is received, this information is passed, along with the battery voltage and current velocity, into a dynamic model that converts this information to a force and torque to apply to the Free Flyer. The dynamic model takes into account the delay between spinning the impellers up and down. This force and torque is applied to the Gazebo model, allowing its physics engine to update the state of the Free Flyer. In addition to publishing basic telemetry (current impeller speed, etc) the PMC plugin also tracks the state of the PMC, thereby realistically simulating ramp-up and ramp-down delay. To test:

    rostopic echo /%ns$/hw/pmc/state
    rostopic echo /%ns$/hw/pmc/telemetry

Customization: description/description/macro_pmc.urdf.xacro
    rate:                rate is not used and the plugin doesn't even read it
    bypass_blower_model: whether the commands in ctr are executed directly, don't put true unless there is a reason for it

**NavCam, DockCam, and SciCam plugins**

The NavCam and DockCam are forward and aft-facing grayscale fisheye cameras. The SciCam is a forward-facing camera with no distortion and typically of higher resolution than the other two. These plugins copy the image data from the gazebo camera sensor into sensor_msgs::Image messages and publishe them to the to the appropriate ROS topic. To test:

    rostopic hz /%ns$/hw/cam_nav
    rostopic hz /%ns$/hw/cam_dock
    rostopic hz /%ns$/hw/cam_sci

Each camera also publishes its pose and intrinsics. Those can be seen, for example for sci cam, as:

    rostopic echo /sim/sci_cam/pose
    rostopic echo /sim/sci_cam/info

By default, the cameras are disabled, for performance reasons. To enable one of more of them, one can edit

  astrobee/config/simulation/simulation.config

The sci cam is a special case. Even when enabled, so sci_cam_rate is positive, it will publish only camera pose info and no actual images until it receives a custom guest science command on topic /comm/dds/command to do so. (This can be overwritten however by setting sci_cam_continuous_picture_taking in simulation.config to true.)

If the camera images are incorrect, it can be because the simulator cannot keep up. Then the speed of simulation can be decreased, for example, with:

    speed:=0.5

The resolution of NavCam and DockCam is 320 by 240 pixels, as set in sensor_nav_cam.urdf.xacro and sensor_dock_cam.urdf.xacro. The resolution can be increased to 1280 x 960 to agree with the real-world cameras. The SciCam properties can be modified in sensor_sci_cam.urdf.xacro.

Customization: description/description/sensor_nav_cam.urdf.xacro
               description/description/sensor_dock_cam.urdf.xacro
               description/description/sensor_sci_cam.urdf.xacro
    update_rate:    camera update rate
    horizontal_fov: horizontal field of view
    image>width:    image width pixels
    image>height:   image height pixels
    image>format:   image format
    clip>near       nearest capture distance
    clip>far:       maximum captured distance
    distortion>k1:  camera distortion

**HazCam and PerchCam plugins**

These are forward and aft facing depth cameras. The plugin copies the point cloud data from the gazebo depth camera sensor into a sensor_msgs::PointCloud2 message and publishes it to the appropriate ROS topic. To test:

    rostopic hz /%ns$/hw/depth_haz/points
    rostopic hz /%ns$/hw/depth_perch/points

The haz cam plugin also publishes the haz cam camera pose and image intensity measured at the points in the cloud.

Customization: description/description/sensor_haz_cam.urdf.xacro
               description/description/sensor_perch_cam.urdf.xacro
    update_rate:    camera update rate
    horizontal_fov: horizontal field of view
    image>width:    image width pixels
    image>height:   image height pixels
    image>format:   image format
    clip>near       nearest capture distance
    clip>far:       maximum captured distance
    distortion>k1:  camera distortion

**IMU plugin**

On every update of the gazebo world, this plugin obtains the true angular velocity and linear acceleration of the Gazebo IMU sensor and publishes this information as a sensor_msgs::Imu message on the appropriate ROS topic. To test:

    rostopic echo /%ns$/hw/imu

Customization: description/description/sensor_imu.urdf.xacro
    pose:        pose of the imu wrt the body reference frame
    update_rate: imu update rate

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

Customization: description/description/sensor_imu.urdf.xacro
    rate:   rate of which the ground truth is published
    parent: the parent of the ground truth (world)
    child:  the child of the ground truth, the robot
    tf:     whether or not to publish the tf pose for visualization
    pose:   whether or not to publish the pose of the robot
    twist:  whether or not to publish linear and angular velocity
    acc:    whether or not to publish linear and angular acceleration

**Drag plugin**
The drag plugin implements the standard drag formula: D = -0.5 * rho * C_D * area * v * |v|

Customization: description/description/sensor_imu.urdf.xacro
    coefficient: drag coefficient C_D
    area:        area exposed against the movement, can be approximated
    density:     air density
