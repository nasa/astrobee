# Releases

## Release 0.16.0

  * project compiles with catkin
  * mapper performance improvement
  * imu_augmentor performance improvement
  * calibration improvements
  * multiple other fixes

## Release 0.15.2

  * added python linters black and isort in the CI pipeline
  * added perching arm reconnect service
  * added nav and dock cam bayer image output option
  * added code to report time for llp, mlp, hlp
  * added end of motion check 

## Release 0.15.1

  * Separated config files for DDS communications to avoid conflict
  * Increased plan name length
  * Pico proxy was added to the MLP launch file
  * System monitor now attempts to reload nodelets that die on startup
  * Other fixes/software and documentation improvements

## Release 0.15.0

  * Perching functional
  * Ubuntu 20 compatibility
  * Added astrobee to astrobee coms
  * Various other fixes/software improvements

## Release 0.14.3

  * Rotation fallback fix
  * Unified install instructions and Ubuntu 18 nasa install
  * Various other minor fixes
  * Note, Perching does not work in this release, use another version for this.

## Release 0.14.1

  * Dynamic IMU Filtering
  * Added measurement spacing for visual odometry
  * Tuning configurations
  * Bug fixes
  Note: Perching does not work in this release, use another version for this.

## Release 0.14.0

  * Added graph_localizer package
  * Added supporting packages for graph localizer including imu_augmetor, imu_integation, localization_common, localization_measurements
  * Added tools for graph localizer including graph_bag and imu_bias_tester
  * Added rviz plugins for graph localizer and associated packages in localization_rviz_plugins
  * Added gtsam debian
  Note: Perching does not work in this release, use another version for this.

## Release 0.13.0

  * Multiple updates to documentation, doxygen is exported to github pages
  * Adding Github actions to build and test the code and generate documentation
  * astrobee_media now a submodule
  * SciCam now generates compressed images
  * Added memory monitor
  * Cross compiling with docker
  * Other fixes

## Release 0.12.0

  * Localization visualization improvements
  * Changes for migration to github

## Release 0.11.1

  * Fix install error.
  * Color in simulated scicam.

## Release 0.11.0

  * Added Ubuntu 18 compatibility
  * Added dockerfiles for simplified install
  * Fixes to QP planner
  * Various other minor fixes

## Release 0.10.4

  * Histogram correction for sparse mapping.
  * Various other minor fixes.

## Release 0.10.3

  * Don't use sparse mapping features more than once.

## Release 0.10.2

  * Perching behavior implemented
  * Localization sparse mapping timeout increased to 10 minutes
  * Already there tolerances decreased to allow for smaller moves
  * Other minor bug fixes/software improvements

## Release 0.10.1

  * Removed the new signal lights nodes from launch

## Release 0.10.0

  * New control scheme for some commands
    - Independant control of camera streaming/recording parameters
    - Can start/stop recording data independantly of profile selection
    - Required minor IDL changes
  * Perching Arm driver functional with latest firmware
  * Support for many more commands in plan to allow easy "scripting"
  * Localization
    - Separated tunable minimal feature count for ML versus AR
    - Tuned EKF for better stability
    - Fixes in localization_manager behavior
  * Mapping
    - Prune "less" the features in the map in order to improve the search
      restults

## Release 0.9.3

* Many localization improvements making the EKF much more stable and resilient
  to loss of mapped landmarks features.
  - Improved tools to analyze localization bags
  - Fixed bug in two places where the noise computation was not multiplied by
    the focal length
  - Changed the selection of optical flow features to pass a better subset
    to the EKF
  - Tuned lot of the EFK parameters
* Added nodes to check the time synchronization between processors (with
  associated logging and fault generation)
* Re-positioned the dock location on ISS from the mapping activity.

## Release 0.9.2

 * Drop bad images from camera.

## Release 0.9.1

 * Honor "referenceFrame" for simpleMove6DOF commands (allows relative motion)
 * Status video/streaming LEDs controlled by the Executive
 * Default bagger profile tuned to include more core topics
 * Agent state publish current world information
 * Camera driver double buffer fix
 * As usual, other bug fixes too

## Release 0.8.0

 * Fixed simulator jump under some conditions on motion start
 * Data bagger system implemented
 * Command line teleoperation tools implemented
 * SciCam commanding implemented
 * Adjusted inertia parameters from flight unit measurements
 * Pre-configured ISS flight parameters
 * Signal lights communication implemented
 * Heroic documentation effort
 * As usual, many bug fixes

## Release 0.7.6

 * Fixes to sparse mapping and localization manager.

## Release 0.7.5

 * Various fixes for flight units.
 * Improvements to debians.

## Release 0.7.4

 * Fixed compile for vive again.

## Release 0.7.3

 * Fixed compile for vive.

## Release 0.7.2

 * Fix cmake bug.

## Release 0.7.1

 * Minor vive updates
 * Bumps to submodules.

## Release 0.7.0

 * Infrastructure to build the flight images completed, including HLP
    (Android) configuration methods not requiring touchscreen access
  * Inertia and masses reflecting the Flight Units characteristics
  * New values for flight modes optimized for the new propulsion units
  * Support for all functional tests (including Speedcam modes)
  * Support for all mobility tests (including plans)
  * Improved marker tracking simulation plugin allowing to analyze marker
    placement
  * Fixed a jerk motion while docking (real robot) due to camera occlusion
  * Executive:
    - Refactored for easier maintenance
    - Support data to disk configuration
    - Support telemetry rate command
    - Support localization mode switch and EKF commands
    - Manage guest science state
  * Real time analysis tool (gviz) ported to support DDS telemetry:
    allows the same capabilities of inspection for flight than on the ground.
  * New launch file feature to support backlisting / adding extra nodes
  * Bug fixes

## Release 0.6.1

Added config files for new robots.

## Release 0.6.0

Flight packaging candidate release

  - Indicator LEDs management.
  - All DDS commands to support remote tests implemented.
  - Adjustments and tools to support test procedures in place.

## Release 0.5.0

Build 3 - CERT Release

  - Everything ready for CERT testing
  - Improved simulator

## Release 0.4.5

Updated external user install instructions.

## Release 0.4.4

Remove astrobee-repo-version from publicly required packages list.

## Release 0.4.3

Fix decomputil debian, revert to older version.

## Release 0.4.2

This release fixes a bug where we attempted to build the smart dock even
if DDS is not installed.

## Release 0.4.1

This release fixes a bug in the smart dock interacting with GDS.

## Release 0.4.0

This release includes improvements of map building and camera calibration,
simulator performance and multi-robot support, new features for guest science,
and many internal changes.

### Map Building
  - Map building fixes.
  - Multiple maps assembly pipeline optimized.

### Sensor Calibration
  - Streamlined image cameras and depth cameras extrinsics relative to IMU

### Simulator
  - Completed support for docking / un-docking.
  - Default robot starts on-dock in the JEM.
  - Synthetic generation of optical flow features.
  - Support for launching a simulation without a robot.
  - Support for controlling simulation speed.
  - Substantially better runtime performance through simplified collision meshes.
  - More complete ISS mesh; improved dock and freeflyer meshes.

### Management
  - The system monitor now triggers heartbeat faults for all nodes in the system.
  - Management of operational limits.
  - Improved "mode" management including support for IDLE.

### Guest Science
  - A GS Manager stub was added so GS donâ€™t have to use Android to test their
  applications.
  - Guest Science Manager changed to create and pass the base data path to a GS
    APK.
  - A new method was added to the guest science library that will provide the
    base data path to a GS APK.
  - Better GS examples added.

### Internal (support for non-open source platform and avionics repos)
  - Mechanisms for local (robot) launch at startup.
  - Support for astrobee_avionics (firmware) improvements.
  - Smarter management of propulsion modules.
  - Support for Software/Firmware Upgrade, EMI and Network tests.

### Fixed from previous release
  - Fixed linear / angular acceleration limits in holonomic mode.
  - Fixed keep in and keep out zone checking.
  - DDS hanging on exit.

### Limitations
  - Granite simulation not functional.
  - The GS Manager Stub does not support yet all Android functionalities
  - Some performance limitations with multiple robots.
  - Some performance trouble with mobility on one computer.

## Release 0.3.2

The previous hotfix updated the debians, but did not update the version numbers of which
debians to install. This corrects that.

## Release 0.3.1

This hotifx fixes the build with ROS' new opencv release, which moved the opencv libraries
from /opt/ros/kinetic/lib to /opt/ros/kinetic/lib/PLATFORM, breaking our RPATHs.

## Release 0.3.0

This release addresses several bugs identified by Guest Scientists, simulator improvements as well as many internal changes.
The only API change is in the EkfState (removed augmented state).

### Simulator
  - Stability improvements
  - Better performance by optimized collision checking and new method for feature simulation
  - More detailed CAD models and textures

### Mobility
  - New mapper node builds an Octomap from the HazCam data
  - Collision checking performed at runtime agains the Octomap
  - Docking and undocking complete

### Limitations
  - Angular velocities/acceleration in GDS not consistent with FSW (will be fixed in GDS)
  - Keep in and keep out zones are not checked (during validation or execution)
  - Simulated docking in a beta stage and should not be used

## Release 0.2.0

### Simulator
  - Perching Arm motion (and dynamics) functional
  - PerchCam and HazCam depth sensors enabled
  - Improved performance

### Guest Science
  - Guest Science Manager implementation available on Android
  - Guest Science library for communication between the manager and guest apps
  - Executive can command Guest Science apps

### Architecture
  - GN&C decomposed into three separate ROS nodes for easier customization / extension:
    - EKF
    - Control
    - Force Allocation Module
  - Management of FlightModes handled on all levels (Exec, Mobility,
    GN&C and Propulsion)

### Localization
  - Feature map much smaller
  - Faster sparse mapping (3Hz on 2 cores only versus 2Hz on 4 cores)
  - Visualization tool for localization features
  - Calibration of depth cameras relative to IMU

### Mobility
  - Fixed 6DoF face-forward bug with the trapezoidal path planner
  - Asynchronous, state-based mobility and control pipeline with improved debugging output
  - Multiple speed gains now supported by FAM, with a controlled ramp up/down
  - Robot now starts in "off" flight mode by default
  - Asychronous, state-based procedures for
    - Arm control
    - Docking and undocking
    - Perching and unperching

### Executive
  - Changed to new motion system
  - Flight mode propagation

### Hardware drivers
  - Picoflexx driver produces depth images in addition to point clouds
  - Picoflexx internal core (Royale) driver bumped to v3.9.0 LTS
  - PMC actuator now determines its state (ramping up/down, ready) based on telemetry feedback

## Release 0.1.2

### Simulator
  - Use the same flight software stack that is run on the platform (but the HW
    drivers)
  - Dynamics of the Astrobee using Gazebo at 1KHz
  - One ISS module environment from Gazebo
  - GNC control running at 62.5Hz (same as real platform)
  - uses GNC Simulink blower propulsion module
  - EKF inputs:
    - IMU model (no noise in this release)
    - Sparse mapping features (sampled from point cloud)
    - Visual features from synthetic images for optical flow
    - Camera models (from Gazebo with radial distortion)
  - Flashlight and laser representation
  - Can run with Gazebo 7 GUI and RViz or headless
  - Supports muti-Astrobee simulation (no communication between them)
  - Collision between Astrobee and ISS walls simulated

#### Limitations
  - No AR target tracking
  - No Handrail detection
  - Depthcam point cloud disabled (efficiency)
  - Camera raytracing runs at 2Hz
  - Conservative limit on angular velocity
  - Arm simulation disabled
  - No noise in the system


### Guest Science
  - JAVA API generated from XP-JSON command dictionary
  - Commands can be send to the Executive using the API and provided ROS Java
    framework

#### Limitations
  - Android framework to support ROS Java in development
  - No guest science manager (life cycle of Guest Science apps)

### Localization
  - Localize without external infrastructure (beacons, etc.)
  - EKF works with following inputs:
    - Sparse mapping with BRISK (regular nav), ~2Hz
    - Optical flow ~15Hz
    - AR targets (docking)
    - Handrail detection (perching)
  - Produce 62.5Hz output
  - Localization manager allows switching safely between localization modes
  - Tools to calibrate intrinsics and extrinsics (IMU to Camera)
  - Tools to build maps from monocular vision (SURF + BRISK)
  - Tools to test localization performance from rosbag
  - Custom visualizer (gViz) for EFK inspection

#### Limitations
  - Sparse mapping runs only at 2Hz (vision processing + feature matching)
  - High distortion limits the usable features
  - Handrail localization needs improvements
  - No extrinsics calibration between depthcam and IMU
  - Better tuning for noise model and "lost" threshold required
  - No incremental map building
  - No tiling of large maps
  - No perched localization (low power localization when perched)

### Mobility
  - Provides 4 different motion types:
    - IDLE (drifting)
    - STOP (zeros velocity, hold position if not externally moved)
    - MOVE (moves to given end pose)
    - EXECUTE (follows a time/space trajectory)
  - Provides 2 motion planners:
    - trapezoidal (default)
    - QP planner
  - Move primitive are position based
  - Validation of trajectories: stationary end-point, hard limits (velocity and
    acceleration) and control frequency
  - Velocity control is not allowed

#### Limitations
  - On-board validation of trajectories against keep-in and keep-out not enabled
  - Planner type cannot be changed from ground (DDS)
  - Hazard detection not implemented

### Executive
  - Arbitrates commands regarding operating mode and current state
  - Sequencer supports plan execution
  - Supports essential commands (see list of currently supported commands)
    - Plan
    - Motion
    - Camera management
   - System management

#### Limitations
  - Full command dictionary is not implemented
  - Guest Science management not supported yet

### Fault Management
  - Faults are managed in a spreadsheet, converted to a fault table read by the
    system
  - Heart beat monitor for most nodes
  - System monitor framework can trigger responses to published faults

#### Limitations
  - Very few faults are published

### Framework
  - Leverages ROS framework for on-board framework
  - All software stack works on Ubuntu 16.04 on Intel and ArmHF (target
    platform)
  - Dependencies packaged as Debian
  - Software updates delivered as Debian
  - Unify launch file system supports multiple scenarios, on-robot, simulator or
    processor in the loop configuration
  - Leverages rViz and Gazebo 7 integration
  - Improved HTC Vive tracker, calibration procedure and integration for ground
    truth

#### Limitations
  - Documentation is incomplete
