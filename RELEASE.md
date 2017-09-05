# Astrobee Robot Software v1, release 0.1.2

## Simulator
  - Use the same flight software stack that is run on the platform (but the HW drivers)
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

### Limitations
  - No AR target tracking
  - No Handrail detection
  - Depthcam point cloud disabled (efficiency)
  - Camera raytracing runs at 2Hz
  - Conservative limit on angular velocity
  - Arm simulation disabled
  - No noise in the system


## Guest Science
  - JAVA API generated from XP-JSON command dictionary
  - Commands can be send to the Executive using the API and provided ROS Java framework

### Limitations
  - Android framework to support ROS Java in development
  - No guest science manager (life cycle of Guest Science apps)

## Localization
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

### Limitations
  - Sparse mapping runs only at 2Hz (vision processing + feature matching)
  - High distortion limits the usable features
  - Handrail localization needs improvements
  - No extrinsics calibration between depthcam and IMU
  - Better tuning for noise model and "lost" threshold required
  - No incremental map building
  - No tiling of large maps
  - No perched localization (low power localization when perched)

## Mobility
  - Provides 4 different motion types:
    - IDLE (drifting)
    - STOP (zeros velocity, hold position if not externally moved)
    - MOVE (moves to given end pose)
    - EXECUTE (follows a time/space trajectory)
  - Provides 2 motion planners:
    - trapezoidal (default)
    - QP planner
  - Move primitive are position based
  - Validation of trajectories: stationary end-point, hard limits (velocity and acceleration) and control frequency 
  - Velocity control is not allowed

### Limitations
  - On-board validation of trajectories against keep-in and keep-out not enabled
  - Planner type cannot be changed from ground (DDS)
  - Hazard detection not implemented

## Executive
  - Arbitrates commands regarding operating mode and current state
  - Sequencer supports plan execution
  - Supports essential commands (see list of currently supported commands)
    - Plan
    - Motion
    - Camera management
   - System management

### Limitations
  - Full command dictionary is not implemented
  - Guest Science management not supported yet

## Fault Management
  - Faults are managed in a spreadsheet, converted to a fault table read by the system
  - Heart beat monitor for most nodes
  - System monitor framework can trigger responses to published faults

### Limitations
  - Very few faults are published

## Framework
  - Leverages ROS framework for on-board framework
  - All software stack works on Ubuntu 16.04 on Intel and ArmHF (target platform)
  - Dependencies packaged as Debian
  - Software updates delivered as Debian
  - Unify launch file system supports multiple scenarios, on-robot, simulator or processor in the loop configuration
  - Leverages rViz and Gazebo 7 integration
  - Improved HTC Vive tracker, calibration procedure and integration for ground truth

### Limitations
  - Documentation is incomplete
