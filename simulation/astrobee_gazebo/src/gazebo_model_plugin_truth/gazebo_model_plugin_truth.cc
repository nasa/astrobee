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

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>

// Tf2 includes
#include <tf2_ros/transform_broadcaster.h>

// Messages
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

// STL includes
#include <string>

namespace gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the forced to be applied to the rigid body
class GazeboModelPluginTruth : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginTruth() : FreeFlyerModelPlugin("truth", false), rate_(62.5) {}

  virtual ~GazeboModelPluginTruth() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
    // If we specify a frame name different to our sensor tag name
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");

    // Setup TF2 message
    msg_.header.frame_id = "world";
    msg_.child_frame_id = GetFrame("truth");

    // Ground truth
    pub_truth_pose_ = nh->advertise<geometry_msgs::PoseStamped>(
      TOPIC_LOCALIZATION_TRUTH, 1);

    // Called before each iteration of simulated world update
    next_tick_ = GetWorld()->GetSimTime();
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboModelPluginTruth::Update, this));
  }

  // Called on simulation reset
  virtual void Reset() {
    next_tick_ = GetWorld()->GetSimTime();
  }

  // Called on every discrete time tick in the simulated world
  virtual void Update() {
    if (GetWorld()->GetSimTime() >= next_tick_) {
      next_tick_ += 1.0 / rate_;
      static tf2_ros::TransformBroadcaster br;
      msg_.header.stamp = ros::Time::now();
      msg_.transform.translation.x = GetModel()->GetWorldPose().pos.x;
      msg_.transform.translation.y = GetModel()->GetWorldPose().pos.y;
      msg_.transform.translation.z = GetModel()->GetWorldPose().pos.z;
      msg_.transform.rotation.x = GetModel()->GetWorldPose().rot.x;
      msg_.transform.rotation.y = GetModel()->GetWorldPose().rot.y;
      msg_.transform.rotation.z = GetModel()->GetWorldPose().rot.z;
      msg_.transform.rotation.w = GetModel()->GetWorldPose().rot.w;
      br.sendTransform(msg_);
      // Pose
      ros_truth_pose_.header = msg_.header;
      ros_truth_pose_.pose.position.x = GetModel()->GetWorldPose().pos.x;
      ros_truth_pose_.pose.position.y = GetModel()->GetWorldPose().pos.y;
      ros_truth_pose_.pose.position.z = GetModel()->GetWorldPose().pos.z;
      ros_truth_pose_.pose.orientation.x = GetModel()->GetWorldPose().rot.x;
      ros_truth_pose_.pose.orientation.y = GetModel()->GetWorldPose().rot.y;
      ros_truth_pose_.pose.orientation.z = GetModel()->GetWorldPose().rot.z;
      ros_truth_pose_.pose.orientation.w = GetModel()->GetWorldPose().rot.w;
      pub_truth_pose_.publish(ros_truth_pose_);
    }
  }

 private:
  double rate_;
  common::Time next_tick_;
  geometry_msgs::TransformStamped msg_;
  event::ConnectionPtr update_;
  geometry_msgs::PoseStamped ros_truth_pose_;
  geometry_msgs::TwistStamped ros_truth_twist_;
  geometry_msgs::AccelStamped ros_truth_accel_;
  ros::Publisher pub_truth_pose_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginTruth)

}   // namespace gazebo
