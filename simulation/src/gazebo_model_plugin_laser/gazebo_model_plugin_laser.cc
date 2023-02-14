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

// Transformation helper code
#include <tf2_ros/transform_listener.h>

// RVIZ visualization
#include <visualization_msgs/msg/marker.hpp>
namespace visualization_msgs {
typedef msg::Marker Marker;
}  // namespace visualization_msgs

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// Freeflyer messages
#include <ff_hw_msgs/srv/set_enabled.hpp>

// STL includes
#include <string>
#include <thread>

namespace gazebo {

FF_DEFINE_LOGGER("gazebo_model_plugin_laser");

class GazeboModelPluginLaser : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginLaser() : FreeFlyerModelPlugin("laser", "laser", true),
    rate_(10.0), range_(50.0), width_(0.0025) {}

  ~GazeboModelPluginLaser() {
    if (update_) {
      update_.reset();
    }
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle& nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    if (sdf->HasElement("range"))
      range_ = sdf->Get<double>("range");
    if (sdf->HasElement("width"))
      width_ = sdf->Get<double>("width");

    // Use the message system to toggle visibility of visual elements
    gz_ = transport::NodePtr(new transport::Node());
    gz_->Init();
    pub_gazebo_ = gz_->Advertise<msgs::Visual>("~/visual");

    // For the RVIZ marker array
    pub_rviz_ = FF_CREATE_PUBLISHER(nh, visualization_msgs::Marker, TOPIC_HARDWARE_LASER_RVIZ, 0);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(NodeHandle& nh) {
    // Advertise the presence of the laser
    srv_ = nh->create_service<ff_hw_msgs::srv::SetEnabled>(SERVICE_HARDWARE_LASER_ENABLE,
      std::bind(&GazeboModelPluginLaser::ToggleCallback, this,
          std::placeholders::_1, std::placeholders::_2));
  }

  // Manage the extrinsics based on the sensor type
  bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf) {
    if (!tf) {
      FF_WARN("Laser extrinsics are null");
      return false;
    }

    // Rotate from the cylinder frame Z fwd to X fwd
    ignition::math::Pose3d pose(
      range_ / 2, 0, 0, 0.7071067811, 0, 0.7071067811, 0);

    // Aggregate pose
    pose = pose + ignition::math::Pose3d(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);

    // Create a visual
    visual_.set_name(GetFrame("laser_visual", "_"));
    visual_.set_parent_name(GetModel()->GetLink()->GetScopedName());
    msgs::Geometry *geometry = visual_.mutable_geometry();
    geometry->set_type(msgs::Geometry::CYLINDER);
    geometry->mutable_cylinder()->set_radius(width_);
    geometry->mutable_cylinder()->set_length(range_);
    visual_.mutable_material()->mutable_script()->set_name("Astrobee/Laser");
    msgs::Set(visual_.mutable_pose(), pose);
    visual_.set_is_static(false);
    visual_.set_visible(false);
    visual_.set_cast_shadows(false);
    visual_.set_transparency(0.1);
    visual_.set_laser_retro(false);
    pub_gazebo_->Publish(visual_);

    // Setup boilerplate marke code for rviz
    marker_.header.stamp = GetTimeNow();
    marker_.header.frame_id = GetFrame("laser");
    marker_.ns = GetFrame("laser_visual", "_");
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::CYLINDER;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = range_ / 2;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0.7071067811;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 0.7071067811;
    marker_.scale.x = width_ * 2;
    marker_.scale.y = width_ * 2;
    marker_.scale.z = range_;
    marker_.color.a = 0.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;
    pub_rviz_->publish(marker_);

    // Modify the new entity to be only visible in the GUI
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(
      &GazeboModelPluginLaser::WorldUpdateBegin, this));

    // Success
    return true;
  }

  // Called when the laser needs to be toggled
  bool ToggleCallback(const std::shared_ptr<ff_hw_msgs::srv::SetEnabled::Request> req,
                      const std::shared_ptr<ff_hw_msgs::srv::SetEnabled::Response> res) {
    // Update the visual marker
    visual_.set_name(GetFrame("laser_visual", "_"));
    visual_.set_visible(req->enabled);
    pub_gazebo_->Publish(visual_);

    // Update RVIZ marker
    marker_.action = visualization_msgs::Marker::MODIFY;
    marker_.header.stamp = GetTimeNow();
    marker_.color.a = (req->enabled ? 0.9 : 0.0);
    pub_rviz_->publish(marker_);

    // Print success and return
    res->success = true;
    res->status_message = "Laser toggled successfully";
    return true;
  }

  // Called when a new entity is created
  void WorldUpdateBegin() {
    rendering::ScenePtr scene = rendering::get_scene();
    if (!scene || !scene->Initialized())
      return;
    rendering::VisualPtr visual =
      scene->GetVisual(GetFrame("laser_visual", "_"));
    if (!visual)
      return;
    visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    if (update_) {
      update_.reset();
    }
  }

 private:
  double rate_, range_, width_;
  transport::NodePtr gz_;
  transport::PublisherPtr pub_gazebo_;
  event::ConnectionPtr update_;
  rclcpp::Service<ff_hw_msgs::srv::SetEnabled>::SharedPtr srv_;
  rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr pub_rviz_;
  visualization_msgs::Marker marker_;
  msgs::Visual visual_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginLaser)

}   // namespace gazebo
