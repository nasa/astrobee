\page ekf GNC EKF Wrapper

The extended kalman filter tracks the robot's pose.

# Overview

The EKF is based on the augmented state extended kalman filter by Mourikis et. al.
We localize from IMU readings and visual odometry, together with one of sparse map features,
AR tags, or handrail detections. Only one of these last features can be used at a time.
The filter has three main steps:

* **Predict**: With every IMU measurement, the EKF forward predicts the robot's motion based on the IMU measurement (linear acceleration and angular velocity). The predict step must
be run a constant rate (currently 62.5 Hz). Every iteration
of the loop must run at this rate, so the performance of the code is critical.
* **Registration**: Camera images cannot be process within one iteration of the predict step.
Hence, we have a registration step when an image is captured. When a registration pulse is
sent to GNC, the EKF state is *augmented*, and we remember the robot's pose when the image
was captured, and capture the covariance to the rest of the state.
* **Update**: When an image is processsed, an observation is used to update the state.

For more details on how the EKF works, see our paper, *Localization from Visual Landmarks on a Free-Flying Robot.
Brian Coltin, Jesse Fusco, Zack Moratto, Oleg Alexandrov, and Robert Nakamura. IROS 2016*.

# Inputs

* IMU Readings, `/hw/imu`: The IMU message. Must be received at a constant rate.
* Sparse Map Features, `/loc/ml/features` and `/loc/ml/registration`:
The features and registration pulses from the sparse map. The features include image coordinates and corresponding
3D feature positions from the map.
* AR Tag Features, `/loc/ar/features` and `/loc/ar/registration`: The same as the
sparse map features, but for AR tags. These are assumed to come from the dock camera.
* Optical Flow Features: `/loc/of/features` and `/loc/of/registration`: These
features are used for visual odometry. They are not associated with a 3D position, but are tracked over time
with a unique associated ID. Inside the EKF wrapper we keep track of these points over time and
send appropriate features to the EKF.
* Handrail Features: `/loc/handrail/features` and `/loc/handrail/registration`: The features for
localization with respect to the handrail. These are point features like with the sparse map features, but also
include a direction of the handrail axis and a boolean indicating whether the position along this axis has been observed.

# Outputs

* `/gnc/ekf`, the body state. See the EkfState message documentation for details.
* The body tf2 transform.

# Services

* `/gnc/ekf/reset`: Resets the EKF, setting the pose based on the next incoming visual landmarks.
* `/gnc/ekf/initialize_bias`: Observes the accelerometer readings over the next several seconds,
sets the bias, and resets the EKF. This must only be called when the robot is stationary!
* `/gnc/ekf/set_input`: Switches the input mode of the EKF between sparse map features, AR tags,
and handrail features. Only one of these inputs can be used at a time. However, visual
odometry is always active.

# Threading Model

For good pose tracking, it is key to eliminate latency in IMU measurements and camera
registrations. That is why we use a multi-threaded ROS node, in which all of the
subscriptions record incoming data in a non-blocking manner. A separate thread loops continuously
and runs the GNC autocode step function whenever a new IMU measurement is received
(informed via a condition variable). This allows us to guarantee that all received
registration pulses and updates will be processed on the next tick.

Note that each step of the EKF must run within one IMU tick, or serious problems will arise.
All of the ROS subscriptions and threading code is put in `ekf_wrapper.cc`, while the core
functionality is presented in `ekf.cc`. This way the class in `ekf.cc` can be used in
other ways, for example, when processing a bag file.

# Selecting Visual Odometry Features

Largely, the EKF wrapper simply passes the inputs directly to the GNC matlab code. The one
exception is the visual odometry features. Here, the wrapper keeps track of which augmentations are
stored in the EKF, and the feature observations during those observations. It chooses which
augmentations to send to the EKF when, and then deletes the sent features and chooses new
augmentations. The goals are to send features spanning as wide as possible a period of time,
to drive the covariance down, and to never send the same observation twice (because this results in overconfidence).

