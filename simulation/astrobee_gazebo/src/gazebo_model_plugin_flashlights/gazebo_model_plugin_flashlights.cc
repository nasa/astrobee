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
  GazeboModelPluginFlashlights() : FreeFlyerModelPlugin(NODE_FLASHLIGHTS),
    rate_(10.0), width_(0.03), height_(0.02), depth_(0.005) {}
  virtual ~GazeboModelPluginFlashlights() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {
    // Create a buffer and listener for TF2 transforms
    static tf2_ros::TransformListener listener(buffer_);

    // Get parameters
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
    pub_ = gz_->Advertise<msgs::Visual>("~/visual");
    pub_factory_ = gz_->Advertise<msgs::Light>("~/factory/light");
    pub_light_ = gz_->Advertise<msgs::Light>("~/light/modify");

    // For the RVIZ marker array
    pub_rviz_ = nh->advertise<visualization_msgs::MarkerArray>(
      TOPIC_HARDWARE_LIGHTS_RVIZ, 0, true);

    // Publish / create the markers
    pub_rviz_.publish(markers_);

    // Defer the extrinsics setup to allow plugins to load
    timer_ = nh->createTimer(ros::Duration(1.0), boost::bind(
      &GazeboModelPluginFlashlights::CreateCallback, this, _1,
        nh), true, true);
  }

  // Create macro
  size_t Create(std::string const& name) {
    // Create the RVIZ marker
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time();
    marker.header.frame_id = GetFrame(name);
    marker.ns = name;
    marker.id = markers_.markers.size();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = depth_;
    marker.scale.y = width_;
    marker.scale.z = height_;
    marker.color.a = 0.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    markers_.markers.push_back(marker);

    // Listen for the transform
    try {
      // Lookup the transform for this sensor
      geometry_msgs::TransformStamped tf = buffer_.lookupTransform(
        GetFrame("body"), GetFrame(name), ros::Time(0));
      // Handle the transform for all sensor types
      poses_[marker.id] = ignition::math::Pose3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);
      gzmsg << "Extrinsics set for " << name << "\n";
    } catch (tf2::TransformException &ex) {
      gzmsg << "No extrinsics for " << name << "\n";
    }

    // Create the Gazebo visual
    msgs::Visual msg_v;
    msg_v.set_name(name);
    msg_v.set_parent_name(GetModel()->GetLink()->GetScopedName());
    msgs::Geometry *geometry = msg_v.mutable_geometry();
    geometry->set_type(msgs::Geometry::BOX);
    msgs::Set(geometry->mutable_box()->mutable_size(),
      ignition::math::Vector3d(depth_, width_, height_));
    msg_v.mutable_material()->mutable_script()->set_name("Astrobee/Flashlight");
    msgs::Set(msg_v.mutable_pose(),
      poses_[marker.id] + GetModel()->GetLink()->GetWorldPose().Ign());
    msg_v.set_is_static(false);
    msg_v.set_visible(true);
    msg_v.set_cast_shadows(false);
    msg_v.set_transparency(1.0);
    pub_->Publish(msg_v);

    // Create the gazebo light
    msgs::Light msg_l;
    msg_l.set_name(name + "_light");
    msg_l.set_type(msgs::Light::SPOT);
    msg_l.set_attenuation_constant(1.0);
    msg_l.set_attenuation_linear(0.02);
    msg_l.set_attenuation_quadratic(0.0);
    msg_l.set_range(10);
    msg_l.set_cast_shadows(false);
    msg_l.set_spot_inner_angle(0.6);
    msg_l.set_spot_outer_angle(2.2);
    msg_l.set_spot_falloff(1.0);
    msgs::Set(msg_l.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
    msgs::Set(msg_l.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));
    msgs::Set(msg_l.mutable_pose(),
      poses_[marker.id] + GetModel()->GetLink()->GetWorldPose().Ign());
    pub_factory_->Publish(msg_l);

    // Return the ID to be used later
    return static_cast<size_t>(marker.id);
  }

  // Update implementation
  void Update(std::string const& name, size_t id) {
    // Update the RVIZ visual
    markers_.markers[id].header.stamp = ros::Time();
    markers_.markers[id].action = visualization_msgs::Marker::MODIFY;

    // Update the Gazebo visual
    static msgs::Visual msg_v;
    msg_v.set_name(name);
    msg_v.set_parent_name(GetModel()->GetLink("body")->GetScopedName());
    msgs::Set(msg_v.mutable_pose(),
      poses_[id] + GetModel()->GetLink()->GetWorldPose().Ign());
    pub_->Publish(msg_v);

    // Update the gazebo light
    static msgs::Light msg_l;
    msg_l.set_name(name + "_light");
    msgs::Set(msg_l.mutable_pose(),
      ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0)
        + poses_[id] + GetModel()->GetLink()->GetWorldPose().Ign());
    pub_light_->Publish(msg_l);
  }

  // Manage the extrinsics based on the sensor type
  void CreateCallback(const ros::TimerEvent&, ros::NodeHandle* nh) {
    // Create the lights
    size_t id_f = Create("flashlight_front");
    size_t id_a = Create("flashlight_aft");

    // Create the service for the front flashlight
    srv_f_ = nh->advertiseService<ff_hw_msgs::SetFlashlight::Request,
      ff_hw_msgs::SetFlashlight::Response>(SERVICE_HARDWARE_LIGHT_FRONT_CONTROL,
        boost::bind(&GazeboModelPluginFlashlights::ToggleCallback, this,
          _1, _2, "flashlight_front", id_f));

    // Create the service for the aft flashlight
    srv_a_ = nh->advertiseService<ff_hw_msgs::SetFlashlight::Request,
      ff_hw_msgs::SetFlashlight::Response>(SERVICE_HARDWARE_LIGHT_AFT_CONTROL,
        boost::bind(&GazeboModelPluginFlashlights::ToggleCallback, this, _1, _2,
          "flashlight_aft", id_a));

    // Called before each iteration of simulated world update
    next_tick_ = GetWorld()->GetSimTime();
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(
      &GazeboModelPluginFlashlights::UpdateCallback, this, id_f, id_a));
  }

  // Called when the laser needs to be toggled
  bool ToggleCallback(ff_hw_msgs::SetFlashlight::Request &req,
    ff_hw_msgs::SetFlashlight::Response &res, std::string const& n, size_t id) {
    // Update RVIZ
    markers_.markers[id].header.stamp = ros::Time();
    markers_.markers[id].color.a = static_cast<double>(req.brightness) / 200.0;
    pub_rviz_.publish(markers_);

    // Update Gazebo visual
    static msgs::Visual msg_v;
    msg_v.set_name(n);
    msg_v.set_parent_name(GetModel()->GetLink("body")->GetScopedName());
    msg_v.set_transparency(1.0 - markers_.markers[id].color.a);
    msgs::Set(msg_v.mutable_pose(),
      poses_[id] + GetModel()->GetLink()->GetWorldPose().Ign());
    pub_->Publish(msg_v);

    // Update the gazebo light
    static msgs::Light msg_l;
    msg_l.set_name(n + "_light");
    msg_l.set_attenuation_constant(1.0 - markers_.markers[id].color.a);
    msgs::Set(msg_l.mutable_pose(),
      ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0)
        + poses_[id] + GetModel()->GetLink()->GetWorldPose().Ign());
    pub_light_->Publish(msg_l);

    // Print response
    res.success = true;
    res.status_message = "Flashlight toggled successfully";
    return true;
  }

  // Called on every discrete time tick in the simulated world
  void UpdateCallback(size_t id_f, size_t id_a) {
    // Throttle callback rate
    if (GetWorld()->GetSimTime() < next_tick_)
      return;
    next_tick_ += 1.0 / rate_;

    // Update Gazebo
    Update("flashlight_front", id_f);
    Update("flashlight_aft", id_a);

    // Update RVIZ
    pub_rviz_.publish(markers_);
  }

 private:
  double rate_, width_, height_, depth_;
  common::Time next_tick_;
  transport::NodePtr gz_;
  transport::PublisherPtr pub_, pub_factory_, pub_light_;
  msgs::Visual msg_;
  event::ConnectionPtr update_;
  ros::Timer timer_;
  ros::NodeHandle nh_;
  ros::ServiceServer srv_f_, srv_a_;
  ros::Publisher pub_rviz_;
  std::map<size_t, ignition::math::Pose3d> poses_;
  visualization_msgs::MarkerArray markers_;
  tf2_ros::Buffer buffer_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginFlashlights)

}   // namespace gazebo
