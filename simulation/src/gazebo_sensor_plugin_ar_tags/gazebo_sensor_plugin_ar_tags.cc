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

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>

// FSW includes
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>

// FSW messages
#include <ff_msgs/msg/visual_landmarks.hpp>
#include <ff_msgs/msg/visual_landmark.hpp>
#include <ff_msgs/msg/camera_registration.hpp>
#include <ff_msgs/srv/set_bool.hpp>
namespace ff_msgs {
typedef msg::VisualLandmarks VisualLandmarks;
typedef msg::VisualLandmark VisualLandmark;
typedef msg::CameraRegistration CameraRegistration;
typedef srv::SetBool SetBool;
}  // namespace ff_msgs

// Camera model
#include <camera/camera_model.h>
#include <camera/camera_params.h>

// STL includes
#include <string>

namespace gazebo {
FF_DEFINE_LOGGER("gazebo_sensor_plugin_ar_tags");

class GazeboSensorPluginARTags : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginARTags() : FreeFlyerSensorPlugin("marker_tracking", "dock_cam", true), active_(true) {}

  ~GazeboSensorPluginARTags() {}

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(NodeHandle& nh, sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    // Get a link to the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);
    if (!sensor_) {
      gzerr << "GazeboSensorPluginARTags requires a parent camera sensor.\n";
      return;
    }

    // Check that we have a mono camera
    if (sensor_->Camera()->ImageFormat() != "L8") FF_FATAL_STREAM("Camera format must be L8");

    // Build the nav cam Camera Model
    config_.AddFile("cameras.config");
    config_.AddFile("geometry.config");
    config_.AddFile("dock_markers_specs.config");
    config_.AddFile("dock_markers_world.config");
    config_.AddFile("simulation/ar_tags.config");
    if (!config_.ReadFiles()) {
      FF_ERROR("Failed to read config files.");
      return;
    }

    if (!config_.GetReal("drawing_width", &width_)) FF_ERROR("Could not read the drawing_width parameter.");

    if (!config_.GetReal("drawing_height", &height_)) FF_ERROR("Could not read the drawing_height parameter.");

    if (!config_.GetReal("rate", &rate_)) FF_ERROR("Could not read the rate parameter.");

    if (!config_.GetReal("delay_camera", &delay_camera_)) FF_ERROR("Could not read the delay_camera parameter.");

    if (!config_.GetReal("delay_features", &delay_features_)) FF_ERROR("Could not read the delay_features parameter.");

    if (!config_.GetUInt("num_samp", &num_samp_)) FF_ERROR("Could not read the num_samp parameter.");

    if (!config_.GetUInt("num_features", &num_features_)) FF_ERROR("Could not read the num_features parameter.");

    if (!config_.GetTable("markers_world", &markers_)) FF_ERROR("Could not read the markers_world parameter.");

    Eigen::Vector3d world_t_dock;
    Eigen::Quaterniond world_Q_dock;
    if (!msg_conversions::config_read_transform(&config_, "world_dock_transform", &world_t_dock, &world_Q_dock))
      FF_ERROR("Could not read world_T_dock transform.");
    world_T_dock_ = Eigen::Isometry3d::Identity();
    world_T_dock_.translation() = world_t_dock;
    world_T_dock_.linear() = world_Q_dock.toRotationMatrix();

    for (int marker_i = 0; marker_i < markers_.GetSize(); marker_i++) {
      config_reader::ConfigReader::Table current_marker;
      markers_.GetTable(marker_i + 1, &current_marker);
      static const char* const corner_keys[] = {"top_left", "top_right", "bottom_left", "bottom_right"};
      for (size_t corner_keys_i = 0; corner_keys_i < 4; corner_keys_i++) {
        config_reader::ConfigReader::Table current_position;
        current_marker.GetTable(corner_keys[corner_keys_i], &current_position);
        double position_x, position_y, position_z;
        current_position.GetReal(1, &position_x);
        current_position.GetReal(2, &position_y);
        current_position.GetReal(3, &position_z);
        marker_positions_.push_back(Eigen::Vector3d(position_x, position_y, position_z));
      }
    }

    // Create a publisher for the registration messages
    pub_reg_ = FF_CREATE_PUBLISHER(nh, ff_msgs::CameraRegistration, TOPIC_LOCALIZATION_AR_REGISTRATION, 1);

    // Create a publisher for the feature messages
    pub_feat_ = FF_CREATE_PUBLISHER(nh, ff_msgs::VisualLandmarks, TOPIC_LOCALIZATION_AR_FEATURES, 1);

    // Only do this once
    msg_feat_.header.frame_id = std::string(FRAME_NAME_DOCK);
    msg_reg_.header.frame_id = std::string(FRAME_NAME_DOCK);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(NodeHandle& nh) {
    // Enable mapped landmarks
    srv_enable_ = nh->create_service<ff_msgs::SetBool>(SERVICE_LOCALIZATION_AR_ENABLE,
      std::bind(&GazeboSensorPluginARTags::EnableService, this, std::placeholders::_1, std::placeholders::_2));

    // Timer triggers registration
    timer_registration_.createTimer(1.0 / rate_, std::bind(&GazeboSensorPluginARTags::SendRegistration, this), nh,
                                    false, true);

    // Timer triggers features
    timer_features_.createTimer(0.8 / rate_, std::bind(&GazeboSensorPluginARTags::SendFeatures, this), nh, true, false);
  }

  // Enable or disable the feature timer
  bool EnableService(const std::shared_ptr<ff_msgs::srv::SetBool::Request> req,
                     std::shared_ptr<ff_msgs::srv::SetBool::Response> res) {
    active_ = req->enable;
    res->success = true;
    return true;
  }

  // Called when featured must be sent
  void SendRegistration() {
    if (!active_) return;

    // Add a short delay between the features and new registration pulse
    timer_features_.stop();
    timer_features_.start();

    // Send off the registration pulse
    msg_reg_.header.stamp = GetTimeNow() + rclcpp::Duration::from_seconds(delay_camera_);
    msg_reg_.camera_id++;
    pub_reg_->publish(msg_reg_);

    // Copy over the camera id to the feature message
    msg_feat_.camera_id = msg_reg_.camera_id;

    // Handle the transform for all sensor types
    Eigen::Isometry3d world_T_body =
      (Eigen::Translation3d(GetModel()->WorldPose().Pos().X(), GetModel()->WorldPose().Pos().Y(),
                            GetModel()->WorldPose().Pos().Z()) *
       Eigen::Quaterniond(GetModel()->WorldPose().Rot().W(), GetModel()->WorldPose().Rot().X(),
                          GetModel()->WorldPose().Rot().Y(), GetModel()->WorldPose().Rot().Z()));

    msg_feat_.header.stamp = GetTimeNow();

    Eigen::Isometry3d body_T_dock_cam =
      (Eigen::Translation3d(sensor_->Pose().Pos().X(), sensor_->Pose().Pos().Y(), sensor_->Pose().Pos().Z()) *
       Eigen::Quaterniond(sensor_->Pose().Rot().W(), sensor_->Pose().Rot().X(), sensor_->Pose().Rot().Y(),
                          sensor_->Pose().Rot().Z()));
    Eigen::Isometry3d dock_T_dock_cam = world_T_dock_.inverse() * world_T_body * body_T_dock_cam;

    // Initialize the camera paremeters
    static camera::CameraParameters cam_params(&config_, "dock_cam");
    static camera::CameraModel camera(Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity(), cam_params);

    // Assemble the feature message
    msg_feat_.pose.position.x = dock_T_dock_cam.translation().x();
    msg_feat_.pose.position.y = dock_T_dock_cam.translation().y();
    msg_feat_.pose.position.z = dock_T_dock_cam.translation().z();
    Eigen::Quaterniond q(dock_T_dock_cam.rotation());
    msg_feat_.pose.orientation.w = q.w();
    msg_feat_.pose.orientation.x = q.x();
    msg_feat_.pose.orientation.y = q.y();
    msg_feat_.pose.orientation.z = q.z();
    msg_feat_.landmarks.clear();

    // Create a new ray in the world
    size_t i = 0;
    for (; i < marker_positions_.size(); i++) {
      // Point in the current camera frame -- this should normally be calculated
      // using PnP on the points themselves. However, it's easier to just pull
      // this information from the simulation ground truth.
      Eigen::Vector3d pt_d = marker_positions_[i];
      Eigen::Vector3d pt_c = dock_T_dock_cam.inverse() * pt_d;

      // Check if the feature is in the field of view
      if (!camera.IsInFov(pt_c)) continue;

      // Get the image plane coordinates
      Eigen::Vector2d uv = camera.ImageCoordinates(pt_c);

      // Create the landmark message
      ff_msgs::VisualLandmark landmark;
      landmark.x = pt_d.x();
      landmark.y = pt_d.y();
      landmark.z = pt_d.z();
      landmark.u = uv[0];
      landmark.v = uv[1];
      msg_feat_.landmarks.push_back(landmark);
    }
  }

  // Send features
  void SendFeatures() {
    if (!active_) return;
    pub_feat_->publish(msg_feat_);
  }

 private:
  config_reader::ConfigReader config_;

  rclcpp::Publisher<ff_msgs::CameraRegistration>::SharedPtr pub_reg_;
  rclcpp::Publisher<ff_msgs::VisualLandmarks>::SharedPtr pub_feat_;
  rclcpp::Service<ff_msgs::SetBool>::SharedPtr srv_enable_;
  ff_util::FreeFlyerTimer timer_registration_, timer_features_;
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
  config_reader::ConfigReader::Table markers_;
  std::vector<Eigen::Vector3d> marker_positions_;
  Eigen::Isometry3d world_T_dock_;
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorPluginARTags)

}  // namespace gazebo
