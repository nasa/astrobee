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

// Eigen includes
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// STL includes
#include <string>
#include <map>

namespace gazebo {

class GazeboModelPluginFlashlights : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginFlashlights() : FreeFlyerModelPlugin("flashlights", ""),
    rate_(10.0), width_(0.03), height_(0.02), depth_(0.005) {}

  ~GazeboModelPluginFlashlights() {
    // Gazebo 7.x -> 9.x migration
    // event::Events::DisconnectWorldUpdateEnd(connection_);
    connection_.reset();
    // end Gazebo 7.x -> 9.x migration
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

    // Create the front toggle service
    srv_f_ = nh->advertiseService<ff_hw_msgs::SetFlashlight::Request,
      ff_hw_msgs::SetFlashlight::Response>(SERVICE_HARDWARE_LIGHT_FRONT_CONTROL,
        boost::bind(&GazeboModelPluginFlashlights::ToggleCallback, this,
          _1, _2, "flashlight_front"));

    // Create the aft toggle service
    srv_a_ = nh->advertiseService<ff_hw_msgs::SetFlashlight::Request,
      ff_hw_msgs::SetFlashlight::Response>(SERVICE_HARDWARE_LIGHT_AFT_CONTROL,
        boost::bind(&GazeboModelPluginFlashlights::ToggleCallback, this,
          _1, _2, "flashlight_aft"));

    // Called before each iteration of simulated world update
    // Gazebo 7.x -> 9.x migration
    // next_tick_ = GetWorld()->GetSimTime();
    next_tick_ = GetWorld()->SimTime();
    // end Gazebo 7.x -> 9.x migration
    connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(
      &GazeboModelPluginFlashlights::UpdateCallback, this));
  }

  // Called on every discrete time tick in the simulated world
  void UpdateCallback() {
    // Keep throttline callbacks
    // Gazebo 7.x -> 9.x migration
    // if (GetWorld()->GetSimTime() < next_tick_)
    //   return;
    if (GetWorld()->SimTime() < next_tick_)
      return;
    // end Gazebo 7.x -> 9.x migration
    next_tick_ += 1.0 / rate_;
    // Update the flashlight
    Update("flashlight_front");
    Update("flashlight_aft");
    // Don't bother updating rviz if we haven't created the lights yet
    if (markers_.empty()) return;
    // Update the marker header
    visualization_msgs::MarkerArray markers;
    markers.markers.clear();
    std::map<std::string, visualization_msgs::Marker>::iterator it;
    for (it = markers_.begin(); it != markers_.end(); it++)
      markers.markers.push_back(it->second);
    pub_rviz_.publish(markers);
  }

  // Update implementation
  void Update(std::string const& name) {
    // Create a buffer and listener for TF2 transforms
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);
    // Check that we have the light
    if (lights_.find(name) == lights_.end()) {
      try {
        // Find the transform for this light
        geometry_msgs::TransformStamped tf = buffer.lookupTransform(
          GetFrame("body"), GetFrame(name), ros::Time(0));
        // Add a light
        lights_[name] = ignition::math::Pose3d(
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z,
          tf.transform.rotation.w,
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z);
        // Add a marker
        markers_[name].header.stamp = ros::Time();
        markers_[name].header.frame_id = GetFrame(name);
        markers_[name].ns = name;
        markers_[name].id = markers_.size();
        markers_[name].type = visualization_msgs::Marker::CUBE;
        markers_[name].action = visualization_msgs::Marker::ADD;
        markers_[name].pose.position.x = 0;
        markers_[name].pose.position.y = 0;
        markers_[name].pose.position.z = 0;
        markers_[name].pose.orientation.x = 0;
        markers_[name].pose.orientation.y = 0;
        markers_[name].pose.orientation.z = 0;
        markers_[name].pose.orientation.w = 1;
        markers_[name].scale.x = depth_;
        markers_[name].scale.y = width_;
        markers_[name].scale.z = height_;
        markers_[name].color.a = 0.0;
        markers_[name].color.r = 1.0;
        markers_[name].color.g = 1.0;
        markers_[name].color.b = 1.0;
        // Create the Gazebo visual
        msgs::Visual msg_v;
        msg_v.set_name(name);
        msg_v.set_parent_name(GetModel()->GetLink()->GetScopedName());
        msgs::Geometry *geometry = msg_v.mutable_geometry();
        geometry->set_type(msgs::Geometry::BOX);
        msgs::Set(geometry->mutable_box()->mutable_size(),
          ignition::math::Vector3d(depth_, width_, height_));
        msg_v.mutable_material()->mutable_script()->set_name("Astrobee/Flashlight");
        // Gazebo 7.x -> 9.x migration
        // msgs::Set(msg_v.mutable_pose(),
        //   lights_[name] + GetModel()->GetLink()->GetWorldPose().Ign());
        msgs::Set(msg_v.mutable_pose(),
          lights_[name] + GetModel()->GetLink()->WorldPose());
        // end Gazebo 7.x -> 9.x migration
        msg_v.set_is_static(false);
        msg_v.set_visible(true);
        msg_v.set_cast_shadows(false);
        msg_v.set_transparency(1.0);
        pub_visual_->Publish(msg_v);
        // Create the gazebo light
        msgs::Light msg_l;
        msg_l.set_name(GetModel()->GetScopedName() + "_" + name + "_light");
        msg_l.set_type(msgs::Light::SPOT);
        msg_l.set_attenuation_constant(1.0);
        msg_l.set_attenuation_linear(0.02);
        msg_l.set_attenuation_quadratic(0.0);
        msg_l.set_range(10);
        msg_l.set_cast_shadows(false);
        msg_l.set_spot_inner_angle(0.6);
        msg_l.set_spot_outer_angle(2.2);
        msg_l.set_spot_falloff(1.0);
        // Gazebo 7.x -> 9.x migration
        // msgs::Set(msg_l.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
        // msgs::Set(msg_l.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));
        // msgs::Set(msg_l.mutable_pose(),
        //   lights_[name] + GetModel()->GetLink()->GetWorldPose().Ign());
        msgs::Set(msg_l.mutable_diffuse(), ignition::math::Color(0.5, 0.5, 0.5, 1));
        msgs::Set(msg_l.mutable_specular(), ignition::math::Color(0.1, 0.1, 0.1, 1));
        msgs::Set(msg_l.mutable_pose(),
          lights_[name] + GetModel()->GetLink()->WorldPose());
        // end Gazebo 7.x -> 9.x migration
        pub_factory_->Publish(msg_l);
      // Silently ignore all invalid transform
      } catch (tf2::TransformException &ex) {}
    } else {
      // Update the Gazebo visual
      msgs::Visual msg_v;
      msg_v.set_name(name);
      msg_v.set_parent_name(GetModel()->GetLink("body")->GetScopedName());
      // Gazebo 7.x -> 9.x migration
      // msgs::Set(msg_v.mutable_pose(),
      //   lights_[name] + GetModel()->GetLink()->GetWorldPose().Ign());
      msgs::Set(msg_v.mutable_pose(),
        lights_[name] + GetModel()->GetLink()->WorldPose());
      // end Gazebo 7.x -> 9.x migration
      pub_visual_->Publish(msg_v);
      // Update the gazebo light
      msgs::Light msg_l;
      msg_l.set_name(GetModel()->GetScopedName() + "_" + name + "_light");
      // Gazebo 7.x -> 9.x migration
      // msgs::Set(msg_l.mutable_pose(),
      //   ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0)
      //     + lights_[name] + GetModel()->GetLink()->GetWorldPose().Ign());
      msgs::Set(msg_l.mutable_pose(),
        ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0)
          + lights_[name] + GetModel()->GetLink()->WorldPose());
      // end Gazebo 7.x -> 9.x migration
      pub_light_->Publish(msg_l);
      // Update the marker
      markers_[name].header.stamp = ros::Time();
      markers_[name].header.frame_id = GetFrame(name);
    }
  }

  // Called when the laser needs to be toggled
  bool ToggleCallback(ff_hw_msgs::SetFlashlight::Request &req,
    ff_hw_msgs::SetFlashlight::Response &res, std::string const& name) {
    // Update the alpha channel in rviz
    markers_[name].color.a = static_cast<double>(req.brightness) / 200.0;
    // Update Gazebo visual
    msgs::Visual msg_v;
    msg_v.set_name(name);
    msg_v.set_parent_name(GetModel()->GetLink("body")->GetScopedName());
    msg_v.set_transparency(1.0 - markers_[name].color.a);
    // Gazebo 7.x -> 9.x migration
    // msgs::Set(msg_v.mutable_pose(),
    //   lights_[name] + GetModel()->GetLink()->GetWorldPose().Ign());
    msgs::Set(msg_v.mutable_pose(),
      lights_[name] + GetModel()->GetLink()->WorldPose());
    // end Gazebo 7.x -> 9.x migration
    pub_visual_->Publish(msg_v);
    // Update the gazebo light
    msgs::Light msg_l;
    msg_l.set_name(GetModel()->GetScopedName() + "_" + name + "_light");
    msg_l.set_attenuation_constant(1.0 - markers_[name].color.a);
    // Gazebo 7.x -> 9.x migration
    // msgs::Set(msg_l.mutable_pose(),
    //   ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0)
    //     + lights_[name] + GetModel()->GetLink()->GetWorldPose().Ign());
    msgs::Set(msg_l.mutable_pose(),
      ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0)
        + lights_[name] + GetModel()->GetLink()->WorldPose());
    // end Gazebo 7.x -> 9.x migration
    pub_light_->Publish(msg_l);
    // Print response
    res.success = true;
    res.status_message = "Flashlight toggled successfully";
    return true;
  }

 private:
  double rate_, width_, height_, depth_;
  common::Time next_tick_;
  transport::NodePtr gz_;
  transport::PublisherPtr pub_visual_, pub_factory_, pub_light_;
  event::ConnectionPtr connection_;
  ros::NodeHandle nh_;
  ros::ServiceServer srv_f_, srv_a_;
  ros::Publisher pub_rviz_;
  std::map<std::string, ignition::math::Pose3d> lights_;
  std::map<std::string, visualization_msgs::Marker> markers_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginFlashlights)

}   // namespace gazebo
