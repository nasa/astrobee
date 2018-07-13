\defgroup vive HTC Vive Localization
\ingroup tools

## Overview

The Vive tracking system was developed using ROS to allow real-time tracking of HTC's trackers/controllers with base stations (lighthouses). Vive uses infrared (IR) LEDs and lasers to estimate full 6D poses with mm accuracy. This system was designed for Astrobee.

The Vive working principle follows. The base station emits a synchronization IR flash that is detected by the photodiodes in the tracker/controller. From the duration of the sync flash, it's possible to decode the specifications of the lighthouse. Then the lighthouse starts sweeping the workspace with an IR laser which will eventually be detected by some of the photodiodes. From the time difference between the sync flash and the laser detection, it's possible to compute the angle of each photodiode for the vertical and horizontal sweep. As more photodiodes are detected, there's an improvement in the pose estimation. The tracker/controller also has an inertial measurement unit. HTC's software fuses the light and IMU data but in our solver, we ignore the latter one.

To pull the data from the tracker/controller, this system has ```vive_bridge``` which uses an external C library called ```libdeepdive```. This library pulls the times of the IR detection by the photodiodes, the IMU acceleration and angular velocity, the tracker/controller extrinsics, the lighthouse specs and some general system specs.

Another node manages all the data received from the tracker/controller, ```vive_server```. This subsystem is responsible for sending the data received from the bridge to the right module (described ahead), publishing the transforms and visual clues and manage the operating state. The node has two possible states:

* CALIBRATING - Getting the data, solving it and publishing it as poses;
* TRACKING - Getting the data and saving it for computing the Vive setup.

The tracker/controller itself has also multiple states it can be in. It has a LED that reports the corresponding state. A brief description follows:

* Orange -- charging;
* White -- fully charged;
* Blue -- not paired / not connected;
* Blinking Blue -- ready to pair / connect;
* Green -- paired and connected.

The lighthouses have several modes they may be on. Bellow, the use can encounter its possible configurations and modes:

* 2 lighthouses without sync cable -- one lighthouse must be in mode **b** and the other in **c**;
* 2 lighthouses with sync cable -- one lighthouse must be in mode **b** and the other in mode **A**;
* 1 lighthouse at 30 Hz -- mode **b**;
* 1 lighthouse at 60 Hz -- mode **A**.

## Requirements

### Configure udev and user permissions
Add a file called /et/udev/rules.d/80-htc-vive.rules with the following:

    KERNEL=="hidraw*", ATTRS{idVendor}=="0bb4", MODE="0666"
    KERNEL=="hidraw*", ATTRS{idVendor}=="28de", MODE="0666"
    KERNEL=="hidraw*", ATTRS{idVendor}=="0424", MODE="0666"

Ensure that your user is added to the plugdev Linux group.

### Libraries
To install this software the user will need the following libraries:

* git
* libjson0-dev

### Equipment
This software makes use of following Vive accessories:

* Tracker/Controller + respective USB dongle.
* Lighthouse

Without these, it won't work properly.

## Operating

Before starting Vive, the user should check if the trackers'/controllers' USB dongles or USB cables are properly plugged in and if they're turned on. If the LED in the tracker/controller is green, it means it's ready. If it's blue the user should start the pairing procedure. For that, the user must have Steam and SteamVR installed and proceed to pair the tracker/controller to the dongle.
To pair the tracker/controller to the dongle, the user should follow the procedure:

* Open Steam;
* Open SteamVR;
* Right-click on the SteamVR window;
* Choose ```Devices``` and than ```Pair Controller```;
* While it's searching for new devices to pair, the controller/tracker should have the LED blinking in blue. For that press the button on the tracker/controller for a few seconds until it's in the desired state.

The user may also choose to use the trackers connected to its computer directly with a USB cable, avoiding the pairing procedure.

This system will only start tracking without calibration if the lighthouse and vive frame are properly set in the ```<world_context>.config``` -- config LUA file. To change the vive frame, the user should change ```calibration``` and ```lighthouses``` to the correct serial numbers and coordinates (wrong transforms will degrade the performance of the algorithm).

    -- Configuration for the vive tracking system
    world_vive_parameters = {
      -- World transform from master lighthouse to world
      calibration = { t = vec3(0.0, 0.0, 0.0), r = quat4(0.0, 0.0, 0.0, 1.0) },
      -- List of lighthouses
      lighthouses = {
        { parent = "vive", frame="<lightouse ID>",
          t = vec3(0.0, 0.0, 0.0), r = quat4(0.0, 0.0, 0.0, 1.0) }
      },

Knowing the right transforms to put in the config file is not an easy procedure, that's why we provide a calibration procedure -- explained in Tools.

To start Vive the user should set:

    <...> vive:=true <...>

as in:

    roslaunch astrobee granite.launch vive:=true mlp:=disabled llp:=disabled output:=screen

The first thing that should appear is the system saying that it found the tracker:

    Read calibration data for tracker <tracker serial>
    Found watchman <tracker serial>

After finding the tracker, the system will wait until it's able to completely decode all the packages received from one lighthouse. These will include the calibration data and the lighthouse serial. Once it decodes everything, the user should see:

    Tracker # <tracker serial> rx config for LH <lighthouse serial> (id: <number>)

Now the user can calibrate the system or just stay in tracking mode. For multiple lighthouses, more time should be given, until both serials have appeared in the terminal.

## Tools

### Calibration

To calibrate the system, the user should follow a certain procedure. The calibration is performed with a body of trackers/controllers (1 or more). More than one tracker/controller should only be used if the relative transforms between them is known. 
The body of trackers/controllers should be filled in the LUA config file ```<world_context>.config``` before starting the system if the user wants to perform calibration.
The table ```offset``` contains the transform from the body frame to the wold frame.
The table ```parents``` contains the transforms from the trackers in the body frame to the tracker frame.

    -- The position of the body with respect to the tracker's frames
    extrinsics = {
      -- P4C is the combination of the starboard and port estimate
      {
        frame = "<body name>",
        offset = { t = vec3(0.0, 0.0, 0.0), r = quat4(0.0, 0.0, 0.0, 1.0) },
        parents = {
          { frame = "<tracker serial>",
            t = vec3(0.0, 0.0, 0.0), r = quat4(0.0, 0.0, 0.0, 1.0) }
        }
      }
    }

After setting up the LUA config file, the calibration procedure has two steps:

* Starting: the data is sent to and saved by the calibrator. To enable this mode, the user should open a new terminal and enter the following command:

    rosrun vive vive_tool -start

* Stopping: the calibrator stops saving the data and starts a procedure where it solves for the pose of every lighthouse in the world frame. After finding the poses it will return to the TRACKING mode. To start this procedure the user should use the following command:

    rosrun vive vive_tool -stop

The calibration data will be saved in a file in: ```astrobee/resources/vive_<context>.bin``` and will be automatically loaded the next time the system is launched. To overwrite it, the user only has to repeat the calibration procedure.

### Trigger

This tool is used to print in the terminal the latest available pose of the trackers specified as arguments. To use it enter the following command in a separate terminal:

    rosrun vive trigger_tool <tracker1> [<tracker2> ...]

### Speed

This tool prints the linear speed of the trackers specified as arguments in the terminal. To start it enter the following command in a separate terminal:

    rosrun vive speed_tool <tracker1> [<tracker2> ...]

## Modules

### Calibrator

The available calibrator, ```src/vive_calibrate.cc```, is called by the server when the system is in the CALIBRATING state to add new data and to compute the poses of the lighthouse in a chosen world frame. The calibration allows the use of multiple lighthouses.

### Solver

The current solver, ```src/vive_solve.cc```, is called by the server when in TRACKING state and there's new data. It then tries to find a good enough solution. It uses the data from all the lighthouses at the same time to improve the stability of the solution and decrease the jitter and jumps caused by the outliers and noise.

## Data

Being this a system with ROS as its backbone, the data is transmitted in topics. The available topics and a brief description of them follows:

* ```/loc/vive/light``` (```ff_msgs/ViveLight.msg```) -- contains the data corresponding to the laser detection (angles, tracker's serial number, ...)

* ```/loc/vive/trackers``` (```ff_msgs/ViveCalibrationTrackerArray.msg```) -- has the extrinsics of trackers' photodiodes and their IMU scale and bias.

* ```/loc/vive/imu``` (```sensor_msgs/Imu.msg```) - contains the accelerometer and the gyroscope data from the Imu.

* ```/loc/vive/lighthouses``` (```ff_msgs/ViveCalibrationLighthouseArray.msg```) -- has the lighthouse imperfections that may be used to improve the model of the system in the solver and calibrator.

* ```/loc/vive/general``` (```ff_msgs/ViveCalibrationGeneral.msg```) -- has the laser and the flash specifications needed for the Vive system.

* ```loc/vive/imu_markers``` (```visualization_msgs/Marker.msg```) -- visual aid for the IMU.

* ```loc/vive/light_markers``` (```visualization_msgs/MarkerArray.msg```) -- visual aid for the directions of each photodiode.

* ```loc/vive/tracker_markers``` (```visualization_msgs/MarkerArray.msg```) -- visual aid for the photodiodes.

* ```\tf``` (```geometry_msgs/TransformStamped.msg```) - the poses of all the moving parts: trackers and controllers.

* ```\tf_static``` (geometry_msgs/TransformStamped.msg) - poses of all the thing obtained from a calibration: lighthouses, auxiliary vive frame and world frame.

## Performance Evaluation

To evaluate the accuracy of the system, we used Astrobee on a granite level surface. We placed a tracker on top of it and moved it around. From that, we expected the height of the tracker to be constant. From this test, we obtained a maximum deviation of 5 mm and a standard deviation of 1 mm for the height.


For more information of the system, please consult: <*insert paper*>