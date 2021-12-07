\mainpage Astrobee Calibration

This folder contains various scripts for calibration.

# Setup
- Build and install the Astrobee code on the robot.
- Install Kalibr on your computer.

## Installation instructions for Kalibr for Ubuntu 18.04

sudo apt install python-rosinstall ipython python-software-properties \
        python-git ipython python-catkin-tools
sudo apt install libopencv-dev ros-melodic-vision-opencv

# Install pip and use it to install python packages

We assume that Python 2 is used.

curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
sudo python get-pip.py
sudo -H pip install testresources
sudo -H pip install python-igraph==0.8 --upgrade
sudo -H pip install  numpy==1.15.0 opencv-python==4.2.0.32

If necessary, pip and the other packages can be installed in user
space. The PYTHONPATH may need to be set for such packages.
The pip flags --user, --force, and --verbose may be useful.

Kalibr uses Python 2, and many packages are transitioning to Python 3,
so some effort may be needed to install the Python 2 dependencies.

# Build using catkin
export KALIBR_WS=$HOME/source/kalibr_workspace
mkdir -p $KALIBR_WS/src
cd $KALIBR_WS
source /opt/ros/melodic/setup.bash
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd $KALIBR_WS/src
git clone git@github.com:oleg-alexandrov/Kalibr.git

# This line takes care of catkin not finding numpy.
ln -s /usr/local/lib/python2.7/dist-packages/numpy/core/include/numpy $KALIBR_WS/src/Kalibr/Schweizer-Messer/numpy_eigen/include

catkin build -DCMAKE_BUILD_TYPE=Release -j4

# Prepare the environment. This is expected for all the steps below.

Run, for example:

    export SCRIPT_DIR=$SOURCE_PATH/scripts/calibrate
    export ASTROBEE_CONFIG_DIR=$SOURCE_PATH/astrobee/config

# The choice of calibration target

Two calibration target are in use. In the granite lab the April tag
target can be used. It is on the dock and is a combination of AR
tags. Its definition is in:

   $SOURCE_PATH/scripts/calibrate/config/granite_april_tag.yaml

Calibration on the ISS uses a checkerboard pattern. That one is
defined in:

  $SOURCE_PATH/scripts/calibrate/config/iss_checkerboard.yaml

and has 12 rows and 7 columns of black and white squares, each one
being of size 1 inch (2.54 cm).

# Regenerating the calibration target

This script only needs to be executed whenever the configuration of
the target is changed (number of tags, tag's position or id
changed). Normally this step should be skipped.

Run markers_to_Kalibr.py: Usage:

    export KALIBR_WS=$HOME/source/kalibr_workspace
    source $KALIBR_WS/devel/setup.bash
    cd $SCRIPT_DIR
    ./markers_to_Kalibr.py --config config_filename
  
config_filename: config file where the specifications of AR tags of
the target are defined. Default value is
dock_markers_specs.config. This will write the file:

   $SOURCE_PATH/scripts/calibrate/config/granite_april_tag.yaml

Note that $KALIBR_WS should be defined since the script will also
automatically add a header file to Kalibr's directory describing the
family of AR tags (Kalibr uses AprilTag library and not the Alvar
library used in the astrobee software).

The checkerboard target can be regenerated using Kalibr directly,
per 

https://github.com/ethz-asl/kalibr/wiki/calibration-targets

It is very important to remember that the yaml file storing this
configuration counts the internal corners, so if there are 12 rows of
squares, that corresponds to 11 internal corners.

# Intrinsics camera calibration

## Recording calibration data

Prepare the environment:

    source $BUILD_PATH/devel/setup.bash

To launch the nodes needed for calibration, do:

   cd $SCRIPT_DIR
   roslaunch calibration.launch

On the flight unit, more specifically on MLP, do:

   roslaunch astrobee astrobee.launch llp:=??? mlp:=??? \
     nodes:=calibration_nav_cam,calibration_dock_cam,calibration_imu,pico_driver,framestore

This will start the IMU driver, camera drivers, and an image
viewer. 

Astrobee has four cameras. On one side there is the nav camera, which
acquires high-resolution images, and the haz camera, which is a type of
depth camera, and acquires a lower-resolution "amplitude" image. On the
opposite face of Astrobee it has the high-resolution dock camera and
low-resolution perch camera, which is again of the depth type. We will
refer to the high-resolution nav and dock cameras as the HD camera.

To be able to record the amplitude, which is necessary for haz_cam
calibration, the file 

 /opt/astrobee/config/cameras.config

(on the robot's MLP, not on the local machine) needs to be edited
before calibration.launch is started. The api_key variable in the
picoflexx section should be set to the correct value. This will cause
the depth camera to run in L2 mode, and produce data on the "extended"
topic.

After calibration.launch is run, one should also invoke

  roslaunch $SOURCE_PATH/hardware/pico_driver/launch/pico_proxy.launch 
  
which will process the extended topic and produce the needed amplitude
data. Note that on the robot itself this file is stored at:

 /opt/astrobee/share/pico_driver/launch/pico_proxy.launch

When it is desired to record the amplitude for the perch camera,
one should pass to roslaunch the argument:

  topic:=/hw/depth_perch/extended

To calibrate nav_cam attached to a standalone computer, rather than
astrobee's built in one, one can do instead:

    roslaunch astrobee granite.launch mlp:=local llp:=disabled nodes:=nav_cam,framestore

(See $SOURCE_PATH/astrobee/readme.md if the device cannot be found.)

Face the robot towards where you can hold the AR tag in front of the
camera. It should be under bright lighting conditions where the AR tag
is clearly visible.

If no image viewer was started by now, run:

    rosrun rviz rviz

and then add the 'image' topic for cam_nav and cam_dock, and the
amplitude_int topic for the haz and perch cameras. 

If the image in amplitude_int does not look quite right, one may want
to restart pico_proxy with a different value for the amplitude_factor
variable, which is a scaling factor. That one can be passed to
roslaunch, for example, as:

    amplitude_factor:=200

Begin recording the bag. Example to record nav_cam and haz_cam:

    rosbag record /hw/cam_nav /hw/depth_haz/extended/amplitude_int

For the dock and perch cams, one should do:

      rosbag record /hw/cam_dock /hw/depth_perch/extended/amplitude_int

Move the AR tag in front of the camera. If the AR tag is only
partially visible in some frames that is fine. Try to cover the entire
field of view of both cameras (be aware that it varies from depth to
HD camera), and to have the tag in various orientations.

In good networking conditions, it is acceptable to record on your own
computer, not Astrobee. A high frame rate is not required.

Stop recording.

## Processing the data

Make sure the target configuration yaml file is in
$SCRIPT_DIR/config/. If there is no yaml file or the configuration is
not the current one, follow #Setup to calibrate with the dock AR
target.

Run

     export KALIBR_WS=$HOME/source/kalibr_workspace
     source $KALIBR_WS/devel/setup.bash
     $SCRIPT_DIR/intrinsics_calibrate.py robotname bagfile \
        --calibration_target targetname

Arguments: 
      - robotname is the robot's config file to edit (e.g., p4d, cert_unit, honey)
      - bagfile is the bag with the recorded data.
      - targetname is a yaml file describing the calibration target. Use either
        the checkerboard pattern or the April tag (the latter is the default).

Additional flags:

	--dock_cam: To calibrate the dock cam and perch cam pair. If
          not set, the script calibrates nav cam and haz cam.
	--depth_cam: To calibrate both the HD camera (nav or dock) and 
          respective depth camera (haz or perch). If not set, the script
          only calibrates the HD camera.
	--sci_cam: To calibrate together the nav, haz, and sci cameras.
          A bag having these may need to be pre-processed with
          dense_map/mapper/ros/tools/process_bag.py.
	--only_depth_cam: To calibrate only the depth camera (haz or perch).
        --from <value> --to <value>: Use bag data between these times,
          in seconds.
        --approx_sync <value> Time tolerance for approximate image 
          synchronization [s] (default: 0.02).
 	--verbose: To output additional information on the Kalibr's calibration. 
        --nav_cam_topic, --haz_cam_topic, --sci_cam_topic: Specify these topic
          names, overriding the defaults, which are /hw/cam_nav,
          /hw/depth_haz/extended/amplitude_int, /hw/cam_sci

The script will overwrite the intrinsics calibration in the specified
config file. It will also generate in the bagfile directory the
following files:

	.txt file with the calibration results
	.pdf report
	.yaml file with the estimated intrinsics parameters that will be used 
           as an input to the extrinsics calibration 
           (see # Extrinsic camera calibration -> ## Processing data)

If invoked for the nav and haz cameras, it will also save the
transform from the second to the first in the
'hazcam_to_navcam_transform' field in the robot config file, if that
field exists. If in addition invoked for the sci camera, it will also
update the field scicam_to_hazcam_transform if it exits.

## Advice on intrinsics calibration

It is strongly suggested that the intrinsics of the HD and depth
camera be calibrated separately. This makes the process much better
behaved. To calibrate the HD camera only, do not specify the flag
--depth_cam, and to calibrate the depth camera only, use
--only_depth_cam. The only time it may be needed to calibrate both of
these cameras together if it is desired to find the transform between
them, which is not necessary in regular operations.

It is suggested to examine the calibration results*txt file. If the
errors are too large or the camera parameters are implausible, perhaps
calibration need to be rerun with a new bag file. In either case it is
suggested to run calibration several times with different bags, and
pick the result with the smallest error.

Sometimes, if calibration fails, it can be attempted again several
times with the same bag and it may succeed at some point. The
indeterministic nature is likely due to the random shuffling being
done by the algorithm. One can also attempt to use just a portion of
the bag, using the --from and --to options.

The --verbose flag is useful to see if the calibration target corners
are detected properly. This may result in a crash on Ubuntu 18. 

The depth cameras can be tricky to calibrate, since they have lower
resolution and it is hard to do corner detection. One should try to
have the calibration target fill the camera field of view as much as
possible while still having all of it visible at most times.

# Extrinsic camera calibration

## Recording calibration data

- Detach the robot from its stand so it can be lifted freely.
- Attach the calibration target to a wall under bright light.
- Launch, as for intrinsics calibration, calibration.launch and pico_proxy.launch.

- Lift the robot and face the target.
- Begin recording the bag on the robot. The recording cannot have shocks from picking
  up and placing the robot down.
     Example: rosbag record /hw/cam_nav /hw/depth_haz/extended/amplitude_int /hw/imu
- Accelerate the robot rapidly along all axes of motion. Try to excite
  all axis of the IMU. Be careful not to drop the robot.
- Stop recording.
- Put the robot down.
- Copy the bag off of the robot. If the robot is docked, the wired
  network is used, which can be much faster then WiFi.

## Processing the data

Make sure the target configuration yaml file and the IMU yaml file are
in $SCRIPT_DIR/config/.

Run
     export KALIBR_WS=$HOME/source/kalibr_workspace
     source $KALIBR_WS/devel/setup.bash
     $SCRIPT_DIR/extrinsics_calibrate.py robotname intrinsicsYaml bagfile \
        --calibration_target targetname

Arguments: 
      - robotname is the robot's config file to edit (e.g., p4d, cert_unit, honey)
	intrinsicsYaml is a yaml file generated by the previous intrinsics calibration
      - bagfile is the bag with the recorded data.
      - targetname is a yaml file describing the calibration target. Use either
        the checkerboard pattern or the April tag (the latter is the default).

Additional flags:
 	--dock_cam: To calibrate dock cam and perch cam pair. If not
	  set, the script calibrates nav cam and haz cam.
        --from <value> --to <value>: Use bag data between these times, in seconds.
        --timeoffset-padding <value>: Maximum range in which the
          timeoffset may change during estimation, in seconds. See below
          for more info. (default: 0.01)
 	--verbose: To output additional information on Kalibr's
          calibration.

If the previous intrinsics calibration was run for the HD and depth
camera pair, then the output will generate extrinsics for both
cameras. If not, it will only generate for the nav or dock cam. The
script will overwrite the extrinsics calibration in the specified
robot's config file.  It will also generate some results reports in
the bagfile directory.

## Advice on extrinsics calibration

If kalibr crashes with the error "Spline Coefficient Buffer
Exceeded. Set larger buffer margins!" one should increase the value of
--timeoffset-padding, from the default of 0.01, perhaps to 0.02 or
even 0.1. Making this too large can result in the tool being very slow
and perhaps running out of memory.

As for intrinsics, it is suggested that each of the four cameras be
individually calibrated with the IMU to make the process better
behaved. Hence, one first calibrates the intrinsics of one camera (say
nav or haz), then runs extrinsics calibration for it (so, with the
IMU), before switching to a new camera. (A single bag can be used for
both cameras facing the same direction, or separate bags can be
acquired.) The tool will infer from its input .yaml file if to work
with the HD or depth camera.

Sometimes parts of the acquired bags could be causing problems to the
calibrator. One can then experiment using just a portion of the data
with the --from and --to flags.
