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
#include <visualization_msgs/Marker.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// Freeflyer messages
#include <ff_hw_msgs/SetEnabled.h>

// STL includes
#include <string>
#include <thread>

namespace gazebo {

class GazeboModelPluginLaser : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginLaser() : FreeFlyerModelPlugin("laser", ""),
    rate_(10.0), range_(50.0), width_(0.0025) {}

  ~GazeboModelPluginLaser() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    if (sdf->HasElement("range"))
      range_ = sdf->Get<double>("range");
    if (sdf->HasElement("width"))
      width_ = sdf->Get<double>("width");

    // Rotate from the cylinder frame Z fwd to X fwd
    pose_ = ignition::math::Pose3d(
      range_ / 2, 0, 0, 0.7071067811, 0, 0.7071067811, 0);

    // Use the message system to toggle visibility of visual elements
    gz_ = transport::NodePtr(new transport::Node());
    gz_->Init();
    pub_ = gz_->Advertise<msgs::Visual>("~/visual");

    // Create the visual
    msgs::Visual msg;
    msg.set_name("laser_visual");
    msg.set_parent_name(GetModel()->GetLink()->GetScopedName());
    msgs::Geometry *geometry = msg.mutable_geometry();
    geometry->set_type(msgs::Geometry::CYLINDER);
    geometry->mutable_cylinder()->set_radius(width_);
    geometry->mutable_cylinder()->set_length(range_);
    msg.mutable_material()->mutable_script()->set_name("Astrobee/Laser");
    // Gazebo 7.x -> 9.x migration
    // msgs::Set(msg.mutable_pose(), pose_ + GetModel()->GetLink()->GetWorldPose().Ign());
    msgs::Set(msg.mutable_pose(), pose_ + GetModel()->GetLink()->WorldPose());
    // end Gazebo 7.x -> 9.x migration
    msg.set_is_static(false);
    msg.set_visible(false);
    msg.set_cast_shadows(false);
    msg.set_transparency(0.1);

    // Publish the message
    pub_->Publish(msg);

    // For the RVIZ marker array
    pub_rviz_ = nh->advertise<visualization_msgs::Marker>(
      TOPIC_HARDWARE_LASER_RVIZ, 0, true);

    // Setup boilerplate marke code for rviz
    marker_.header.stamp = ros::Time();
    marker_.header.frame_id = GetFrame("laser");
    marker_.ns = "laser_visual";
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
    pub_rviz_.publish(marker_);

    // Now switch to mofidy mode
    marker_.action = visualization_msgs::Marker::MODIFY;

    // Setup boilerplate code for gazebo
    msg_.set_name("laser_visual");
    msg_.set_parent_name(GetModel()->GetLink()->GetScopedName());
    msg_.set_visible(false);

    // Advertise the presence of the laser
    srv_ = nh->advertiseService(SERVICE_HARDWARE_LASER_ENABLE,
      &GazeboModelPluginLaser::ToggleCallback, this);

    // Defer the extrinsics setup to allow plugins to load
    connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(
      &GazeboModelPluginLaser::ExtrinsicsCallback, this));
  }

  // Manage the extrinsics based on the sensor type
  void ExtrinsicsCallback() {
    // Create a buffer and listener for TF2 transforms
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);
    // Get extrinsics from framestore
    try {
      // Lookup the transform for this sensor
      geometry_msgs::TransformStamped tf = buffer.lookupTransform(
        GetFrame("body"), GetFrame("laser"), ros::Time(0));
      // Handle the transform for all sensor types
      pose_ = pose_ + ignition::math::Pose3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);
      // Kill the connection
      // Gazebo 7.x -> 9.x migration
      // event::Events::DisconnectWorldUpdateEnd(connection_);
      // // Update the connection
      // next_tick_ = GetWorld()->GetSimTime();
      connection_.reset();
      // Update the connection
      next_tick_ = GetWorld()->SimTime();
      // end Gazebo 7.x -> 9.x migration

      connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboModelPluginLaser::UpdateCallback, this));
      gzmsg << "Extrinsics set for laser\n";
    } catch (tf2::TransformException &ex) {
      gzmsg << "No extrinsics for laser\n";
    }
  }

  // Called when the laser needs to be toggled
  bool ToggleCallback(ff_hw_msgs::SetEnabled::Request &req,
                      ff_hw_msgs::SetEnabled::Response &res) {
    // Update Gazebo
    msg_.set_visible(req.enabled);
    // Gazebo 7.x -> 9.x migration
    // msgs::Set(msg_.mutable_pose(),
    //   pose_ + GetModel()->GetLink()->GetWorldPose().Ign());
    msgs::Set(msg_.mutable_pose(),
      pose_ + GetModel()->GetLink()->WorldPose());
    // end Gazebo 7.x -> 9.x migration
    pub_->Publish(msg_);

    // Update RVIZ
    marker_.header.stamp = ros::Time();
    marker_.color.a = (req.enabled ? 0.9 : 0.0);
    pub_rviz_.publish(marker_);

    // Print success and return
    res.success = true;
    res.status_message = "Laser toggled successfully";
    return true;
  }

  // Called on every discrete time tick in the simulated world
  void UpdateCallback() {
    // Throttle callback rate
    // Gazebo 7.x -> 9.x migration
    // if (GetWorld()->GetSimTime() < next_tick_)
    //   return;
    if (GetWorld()->SimTime() < next_tick_)
      return;
    // end Gazebo 7.x -> 9.x migration
    next_tick_ += 1.0 / rate_;

    // Update gazebo
    // Gazebo 7.x -> 9.x migration
    // msgs::Set(msg_.mutable_pose(),
    //   pose_ + GetModel()->GetLink()->GetWorldPose().Ign());
    msgs::Set(msg_.mutable_pose(),
      pose_ + GetModel()->GetLink()->WorldPose());
    // Gazebo 7.x -> 9.x migration
    pub_->Publish(msg_);

    // Update RVIZ
    marker_.header.stamp = ros::Time();
    pub_rviz_.publish(marker_);
  }

 private:
  double rate_, range_, width_;
  common::Time next_tick_;
  transport::NodePtr gz_;
  transport::PublisherPtr pub_;
  msgs::Visual msg_;
  event::ConnectionPtr connection_;
  ros::Timer timer_;
  ros::NodeHandle nh_;
  ros::ServiceServer srv_;
  ros::Publisher pub_rviz_;
  ignition::math::Pose3d pose_;
  visualization_msgs::Marker marker_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginLaser)

}   // namespace gazebo
