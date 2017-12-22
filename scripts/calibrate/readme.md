\mainpage Astrobee Calibration

This folder contains various scripts for calibration.

# Setup

- Build and install the Astrobee code on the robot.
- Install [kalibr](https://github.com/ethz-asl/kalibr) on your computer.

## Installation from sources for Ubuntu 16.04 notes
```
export KALIBR_WS=~/source/kalibr_workspace
sudo apt install python-rosinstall ipython python-software-properties python-git ipython python-catkin-tools
sudo apt install libopencv-dev ros-kinetic-vision-opencv
sudo -H pip install python-igraph --upgrade
sudo -H pip install openv-python
mkdir -p $KALIBR_WS/src
cd $KALIBR_WS
source /opt/ros/kinetic/setup.bash
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd $KALIBR_WS/src
git clone https://github.com/ethz-asl/Kalibr.git
catkin build -DCMAKE_BUILD_TYPE=Release -j4
```
# Intrinsic Camera Calibration

## Recording Calibration Data

- Run
      roslaunch calibration.launch
  to launch the IMU driver, camera drivers, and an image viewer.
- Face the robot towards where you can hold the AR tag in front of the camera.
  It should be under bright lighting conditions where the AR tag is clear.
- Begin recording the bag.
      rosbag record /hw/cam_nav
  In good networking conditions, it is fine to record on your own computer, not
  Astrobee. A high framework is not required.
- Move the AR tag in front of the camera. You must get some frames where all 
  squares are visible. If the AR tag is only partially visible in some frames
  that is fine. Try to cover the entire area of the image.
- Stop recording.

## Processing the Data

Run
    ./intrinsics_calibrate.py robotname bagfile
Where robotname is the config file to edit (e.g., p4d, honey) and bagfile is the
bag with the recorded extrinsics. The "-dock\_cam" flag can be passed for the
dock cam, and "-verbose" for additional information. The script will overwrite
the intrinsics calibration in the specified config file.

# Extrinsic Camera Calibration

## Recording Calibration Data

- Detach the robot from its stand so it can be lifted freely.
- Attach the april target to a wall under bright light.
- Run
      roslaunch calibration.launch
- Lift the robot and face the target.
- Begin recording the bag on the robot. The recording cannot have shocks from
  picking up and placing the robot down.
      rosbag record /hw/cam_nav /hw/imu
- Accelerate the robot rapidly along all axes of motion. Be careful not to drop
  the robot.
- Stop recording.
- Put the robot down.
- Copy the bag off of the robot.

## Processing the Data
