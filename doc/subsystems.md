\defgroup subsystems Subsystems
\ingroup doc

# Freeflyer Subsystems organization and topic name hierarchy

Definition of topic names are maintained in:<br>
  `shared/ff_util/ff_names.h`

## Management
  - directory: `managment`
  - topic prefix: `mgt`
  - include:
    - Executive
    - SystemMonitor
    - AccessControl
    - ResourceMonitors (CPU, disk, etc.) (or should they go to HW?)

## Localization
  - directory: `localization`
  - topic prefix: `loc`
  - We keep this name, everything necessary for localization, pose estimation is performed in the GNC EKF...
  - include:
    - Sparse Mapping Features
    - AR Tags
    - Handrail detect
    - Optical Flow
    - Localization Manager ?

## Mobility
  - directory: `mobility`
  - topic prefix: `mob`
  - include:
    - choreographer
    - planner
      + qp
      + trapezoidal
    - sentinel
    - mapper
    - perching
    - docking

## GN&C
  - directory: `gnc`
  - topic prefix: `gnc`
  - include:
    - EKF
    - Visual Odometry
    - Control

## Communication
  - directory: `communication`
  - topic prefix: `comm`
  - This is not a subsystem like the others... Does the bridge still get a prefix and publish all "FF-ROS-Commands" under it? [ted]
  - include:
    - Bridge
    - Msg / Srv / Action

## Drivers (hardware)
  - directory: `hardware`
  - topic prefix: `hw`
  - include all the LLP/MLP/HLP hardware "drivers"
  - topic naming:
    - hw/camera_dock
    - hw/camera_nav
    - hw/camera_sci
    - hw/depth_perch
    - hw/depth_haz
    - hw/imu
    - hw/pmc
    - hw/laser
    - hw/light_front
    - hw/light_aft
    - hw/arm
    - hw/eps
    - hw/signals ?
