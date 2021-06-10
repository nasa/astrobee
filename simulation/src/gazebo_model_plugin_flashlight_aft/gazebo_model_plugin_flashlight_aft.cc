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

// Transformation helper code
#include <tf2_ros/transform_listener.h>

// RVIZ visualization
#include <visualization_msgs/MarkerArray.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// Freeflyer messages
#include <ff_hw_msgs/SetFlashlight.h>

// STL includes
#include <string>
#include <map>

namespace gazebo {

class GazeboModelPluginFlashlightAft : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginFlashlightAft() : FreeFlyerModelPlugin("flashlight_aft",
    "flashlight_aft", true), rate_(10.0),
      width_(0.03), height_(0.02), depth_(0.005) {}

  ~GazeboModelPluginFlashlightAft() {
    if (update_) {
      #if GAZEBO_MAJOR_VERSION > 7
      update_.reset();
      #else
      event::Events::DisconnectWorldUpdateBegin(update_);
      #endif
    }
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Get parameters from the SDF
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    if (sdf->HasElement("width"))
      width_ = sdf->Get<double>("width");
    if (sdf->HasElement("height"))
      height_ = sdf->Get<double>("height");
    if (sdf->HasElement("depth"))
      depth_ = sdf->Get<double>("depth");

    // Use the message system to toggle visibility of visual elements
    gz_ = transport::NodePtr(new transport::Node());
    gz_->Init();
    pub_visual_ = gz_->Advertise<msgs::Visual>("~/visual");
    pub_factory_ = gz_->Advertise<msgs::Light>("~/factory/light");
    pub_light_ = gz_->Advertise<msgs::Light>("~/light/modify");

    // For the RVIZ marker array
    pub_rviz_ = nh->advertise<visualization_msgs::MarkerArray>(
      TOPIC_HARDWARE_LIGHTS_RVIZ, 0, true);

    // Rotate from the flaslight frame to the visual frame
    pose_ = ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    srv_ = nh->advertiseService(SERVICE_HARDWARE_LIGHT_AFT_CONTROL,
      &GazeboModelPluginFlashlightAft::ToggleCallback, this);
  }


  // Manage the extrinsics based on the sensor type
  bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf) {
    if (!tf) {
      ROS_WARN("Front flashlight extrinsics are null");
      return false;
    }

    // Create the rviz marker
    marker_.header.stamp = ros::Time::now();
    marker_.header.frame_id = GetFrame();
    marker_.ns = GetFrame("flashlight_aft", "_");
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::CUBE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = pose_.Pos().X();
    marker_.pose.position.y = pose_.Pos().Y();
    marker_.pose.position.z = pose_.Pos().Z();
    marker_.pose.orientation.x = pose_.Rot().X();
    marker_.pose.orientation.y = pose_.Rot().Y();
    marker_.pose.orientation.z = pose_.Rot().Z();
    marker_.pose.orientation.w = pose_.Rot().W();
    marker_.scale.x = depth_;
    marker_.scale.y = width_;
    marker_.scale.z = height_;
    marker_.color.a = 0.0;
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(marker_);
    pub_rviz_.publish(msg);

    // Aggregate pose
    pose_ = pose_ + ignition::math::Pose3d(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);

    // Create the Gazebo visual
    visual_.set_name(GetFrame("flashlight_aft_visual", "_"));
    visual_.set_parent_name(GetModel()->GetLink()->GetScopedName());
    msgs::Geometry *geometry = visual_.mutable_geometry();
    geometry->set_type(msgs::Geometry::BOX);
    msgs::Set(geometry->mutable_box()->mutable_size(),
      ignition::math::Vector3d(depth_, width_, height_));
    visual_.mutable_material()->mutable_script()->set_name("Astrobee/Flashlight");
    msgs::Set(visual_.mutable_pose(), pose_);
    visual_.set_is_static(false);
    visual_.set_visible(true);
    visual_.set_cast_shadows(false);
    visual_.set_transparency(1.0);
    pub_visual_->Publish(visual_);

    // Create the gazebo light
    light_.set_name(GetFrame("flashlight_aft_light", "_"));
    light_.set_type(msgs::Light::SPOT);
    light_.set_attenuation_constant(1.0);
    light_.set_attenuation_linear(0.02);
    light_.set_attenuation_quadratic(0.0);
    light_.set_range(10);
    light_.set_cast_shadows(false);
    light_.set_spot_inner_angle(0.6);
    light_.set_spot_outer_angle(2.2);
    light_.set_spot_falloff(1.0);

    // For some reason common::Color stopped existing in later versions
    #if GAZEBO_MAJOR_VERSION < 11
    msgs::Set(light_.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
    msgs::Set(light_.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));
    #endif

    #if GAZEBO_MAJOR_VERSION > 7
    msgs::Set(light_.mutable_pose(), pose_ +
       GetModel()->GetLink()->WorldPose());
    #else
    msgs::Set(light_.mutable_pose(), pose_ +
       GetModel()->GetLink()->GetWorldPose().Ign());
    #endif
    pub_factory_->Publish(light_);

    // Modify the new entity to be only visible in the GUI
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(
      &GazeboModelPluginFlashlightAft::WorldUpdateBegin, this));

    // Success
    return true;
  }

  // Called when the laser needs to be toggled
  bool ToggleCallback(ff_hw_msgs::SetFlashlight::Request &req,
                      ff_hw_msgs::SetFlashlight::Response &res) {
    // Update the alpha channel in rviz
    marker_.header.stamp = ros::Time::now();
    marker_.color.a = static_cast<double>(req.brightness) / 200.0;
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(marker_);
    pub_rviz_.publish(msg);

    // Update Gazebo visual
    visual_.set_transparency(1.0 - marker_.color.a);
    pub_visual_->Publish(visual_);

    // Update the gazebo light
    light_.set_attenuation_constant(1.0 - marker_.color.a);
    pub_light_->Publish(light_);

    // Print response
    res.success = true;
    res.status_message = "Flashlight toggled successfully";
    return true;
  }

  // Called when a new entity is created
  void WorldUpdateBegin() {
    #if GAZEBO_MAJOR_VERSION > 7
    msgs::Set(light_.mutable_pose(), pose_ +
       GetModel()->GetLink()->WorldPose());
    #else
    msgs::Set(light_.mutable_pose(), pose_ +
       GetModel()->GetLink()->GetWorldPose().Ign());
    #endif
    pub_light_->Publish(light_);
  }

 private:
  double rate_, width_, height_, depth_;
  transport::NodePtr gz_;
  transport::PublisherPtr pub_visual_, pub_factory_, pub_light_;
  event::ConnectionPtr update_;
  ros::Timer timer_;
  ros::ServiceServer srv_;
  ros::Publisher pub_rviz_;
  visualization_msgs::Marker marker_;
  msgs::Light light_;
  msgs::Visual visual_;
  ignition::math::Pose3d pose_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginFlashlightAft)

}   // namespace gazebo
