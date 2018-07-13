/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// ROS includes
#include <ros/ros.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// FSW includes
#include <config_reader/config_reader.h>

// FSW messages
#include <ff_msgs/VisualLandmarks.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/SetBool.h>

// Camera model
#include <camera/camera_model.h>
#include <camera/camera_params.h>

// STL includes
#include <string>

namespace gazebo {

class GazeboSensorPluginARTags : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginARTags() :
    FreeFlyerSensorPlugin("marker_tracking", "dock_cam", true),
      active_(true) {}

  ~GazeboSensorPluginARTags() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(ros::NodeHandle *nh,
    sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginARTags requires a parent camera sensor.\n";
      return;
    }

    // Check that we have a mono camera
    if (sensor_->Camera()->ImageFormat() != "L8")
      ROS_FATAL_STREAM("Camera format must be L8");

    // Build the nav cam Camera Model
    config_.AddFile("cameras.config");
    config_.AddFile("dock_markers_specs.config");
    config_.AddFile("simulation/ar_tags.config");
    if (!config_.ReadFiles()) {
        ROS_ERROR("Failed to read config files.");
        return;
    }

    if (!config_.GetReal("drawing_width", &width_))
      NODELET_ERROR("Could not read the drawing_width parameter.");

    if (!config_.GetReal("drawing_height", &height_))
      NODELET_ERROR("Could not read the drawing_height parameter.");

    if (!config_.GetReal("rate", &rate_))
      NODELET_ERROR("Could not read the rate parameter.");

    if (!config_.GetReal("delay_camera", &delay_camera_))
      NODELET_ERROR("Could not read the delay_camera parameter.");

    if (!config_.GetReal("delay_features", &delay_features_))
      NODELET_ERROR("Could not read the delay_features parameter.");

    if (!config_.GetUInt("num_samp", &num_samp_))
      NODELET_ERROR("Could not read the num_samp parameter.");

    if (!config_.GetUInt("num_features", &num_features_))
      NODELET_ERROR("Could not read the num_features parameter.");

    // Create a publisher for the registration messages
    pub_reg_ = nh->advertise<ff_msgs::CameraRegistration>(
      TOPIC_LOCALIZATION_AR_REGISTRATION, 1);

    // Create a publisher for the feature messages
    pub_feat_ = nh->advertise<ff_msgs::VisualLandmarks>(
      TOPIC_LOCALIZATION_AR_FEATURES, 1);

    // Enable mapped landmarks
    srv_enable_ = nh->advertiseService(SERVICE_LOCALIZATION_AR_ENABLE,
      &GazeboSensorPluginARTags::EnableService, this);

    // Timer triggers registration
    timer_registration_ = nh->createTimer(ros::Duration(ros::Rate(rate_)),
      &GazeboSensorPluginARTags::SendRegistration, this, false, true);

    // Timer triggers features
    timer_features_ = nh->createTimer(ros::Duration(0.8 / rate_),
      &GazeboSensorPluginARTags::SendFeatures, this, true, false);

    // Only do this once
    msg_feat_.header.frame_id = std::string(FRAME_NAME_WORLD);
    msg_reg_.header.frame_id = std::string(FRAME_NAME_WORLD);
  }

  // Enable or disable the feature timer
  bool EnableService(ff_msgs::SetBool::Request & req,
                     ff_msgs::SetBool::Response & res) {
    active_ = req.enable;
    res.success = true;
    return true;
  }

  // Manage the extrinsics based on the sensor type
  bool GetDockLocation(Eigen::Affine3d & wTd) {
    // Create a buffer and listener for TF2 transforms
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);
    // Get extrinsics from framestore
    try {
      // Lookup the transform for this sensor
      geometry_msgs::TransformStamped tf = buffer.lookupTransform(
        "world", "dock/body", ros::Time(0));
      // Handle the transform for all sensor types
      wTd = (
        Eigen::Translation3d(
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z) *
        Eigen::Quaterniond(
          tf.transform.rotation.w,
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z));
    } catch (tf2::TransformException &ex) {
      return false;
    }
    return true;
  }

  // Called when featured must be sent
  void SendRegistration(ros::TimerEvent const& event) {
    if (!active_ || !ExtrinsicsFound()) return;

    // Add a short delay between the features and new registration pulse
    timer_features_.stop();
    timer_features_.start();

    // Send off the registration pulse
    msg_reg_.header.stamp = ros::Time::now() + ros::Duration(delay_camera_);
    msg_reg_.camera_id++;
    pub_reg_.publish(msg_reg_);
    ros::spinOnce();

    // Copy over the camera id to the feature message
    msg_feat_.camera_id = msg_reg_.camera_id;

    // In order to get meaningful data we need a dock location
    Eigen::Affine3d wTd;
    if (!GetDockLocation(wTd)) {
      timer_features_.stop();
      return;
    }

    // Handle the transform for all sensor types
    Eigen::Affine3d wTb = (
        Eigen::Translation3d(
          GetModel()->GetWorldPose().pos.x,
          GetModel()->GetWorldPose().pos.y,
          GetModel()->GetWorldPose().pos.z) *
        Eigen::Quaterniond(
          GetModel()->GetWorldPose().rot.w,
          GetModel()->GetWorldPose().rot.x,
          GetModel()->GetWorldPose().rot.y,
          GetModel()->GetWorldPose().rot.z));
    Eigen::Affine3d bTs = (
        Eigen::Translation3d(
          sensor_->Pose().Pos().X(),
          sensor_->Pose().Pos().Y(),
          sensor_->Pose().Pos().Z()) *
        Eigen::Quaterniond(
          sensor_->Pose().Rot().W(),
          sensor_->Pose().Rot().X(),
          sensor_->Pose().Rot().Y(),
          sensor_->Pose().Rot().Z()));
    Eigen::Affine3d wTs = wTb * bTs;

    // Initialize the camera paremeters
    static camera::CameraParameters cam_params(&config_, "dock_cam");
    static camera::CameraModel camera(Eigen::Vector3d(0, 0, 0),
      Eigen::Matrix3d::Identity(), cam_params);

    // Assemble the feature message
    msg_feat_.pose.position.x = wTs.translation().x();
    msg_feat_.pose.position.y = wTs.translation().y();
    msg_feat_.pose.position.z = wTs.translation().z();
    Eigen::Quaterniond q(wTs.rotation());
    msg_feat_.pose.orientation.w = q.w();
    msg_feat_.pose.orientation.x = q.x();
    msg_feat_.pose.orientation.y = q.y();
    msg_feat_.pose.orientation.z = q.z();
    msg_feat_.landmarks.clear();

    // Create a new ray in the world
    size_t i = 0;
    for (; i < num_samp_ && msg_feat_.landmarks.size() < num_features_; i++) {
      // Draw a random image coordinate
      Eigen::Vector3d pt_d(0,
        (static_cast<double>(rand() % 1000) / 1000 - 0.5) * width_ / 2000,    // NOLINT
        (static_cast<double>(rand() % 1000) / 1000 - 0.5) * height_ / 2000);  // NOLINT

      // Get the camera coordinate of this fake feature
      Eigen::Vector3d pt_w = wTd * pt_d;
      Eigen::Vector3d pt_c = wTs.inverse() * pt_w;

      // Check if the feature is in the field of view
      if (!camera.IsInFov(pt_c))
        continue;

      // Get the image plane coordinates
      Eigen::Vector2d uv = camera.ImageCoordinates(pt_c);

      // Create the landmark message
      ff_msgs::VisualLandmark landmark;
      landmark.x = pt_w.x();
      landmark.y = pt_w.y();
      landmark.z = pt_w.z();
      landmark.u = uv[0];
      landmark.v = uv[1];
      msg_feat_.landmarks.push_back(landmark);
    }
  }

  // Send features
  void SendFeatures(ros::TimerEvent const& event) {
    if (!active_ || !ExtrinsicsFound()) return;
    msg_feat_.header.stamp = ros::Time::now();
    pub_feat_.publish(msg_feat_);
  }

 private:
  config_reader::ConfigReader config_;
  ros::Publisher pub_reg_, pub_feat_;
  ros::ServiceServer srv_enable_;
  ros::Timer timer_registration_, timer_features_;
  std::shared_ptr<sensors::WideAngleCameraSensor> sensor_;
  bool active_;
  ff_msgs::CameraRegistration msg_reg_;
  ff_msgs::VisualLandmarks msg_feat_;
  double rate_;
  double delay_camera_;
  double delay_features_;
  double width_;
  double height_;
  unsigned int num_features_;
  unsigned int num_samp_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginARTags)

}   // namespace gazebo
