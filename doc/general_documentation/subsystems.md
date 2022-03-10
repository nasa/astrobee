\page subsystems Subsystems

## Freeflyer Subsystems organization and topic name hierarchy

Definition of topic names are maintained in \subpage shared:<br>
  `shared/ff_util/ff_names.h`

### Management
  - \subpage management
  - directory: `managment`
  - topic prefix: `mgt`
  - include:
    - \ref executive
    - \ref sys_monitor
    - \ref access_control
    - \ref imagesampler
    - \ref imagesampler
    - \ref data_bagger
    - ResourceMonitors
      - \ref cpu_monitor
      - \ref disk_monitor
    - \ref log_monitor

### Localization
  - \subpage localization
  - directory: `localization`
  - topic prefix: `loc`
  - Everything necessary for localization
  - include:
    - Sparse Mapping Features
    - AR Tags
    - Handrail detect
    - Optical Flow
    - Localization Manager

### Mobility
  - \subpage mobility
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

### GN&C
  - \subpage gnc
  - directory: `gnc`
  - topic prefix: `gnc`
  - include:
    - Control

### Behaviors
  - \subpage beh
  - directory: `behaviors`
  - topic prefix: `beh`
  - include:
    - \ref arm
    - \ref dock
    - \ref perch
    - \ref states
    - \ref signal

### Communication
  - \subpage comms
  - directory: `communication`
  - topic prefix: `comm`
  - This is not a subsystem like the others... Does the bridge still get a prefix and publish all "FF-ROS-Commands" under it? [ted]
  - include:
    - Bridge
    - Msg / Srv / Action

### Drivers (hardware)
  - \subpage hw
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
