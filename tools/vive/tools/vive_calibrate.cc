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

// Command line flags
#include <gflags/gflags.h>
#include <gflags/gflags_completions.h>

// Include RPOS
#include <ros/ros.h>

// Listen for transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

// FSW includes
#include <msg_conversions/msg_conversions.h>

// Messages
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <ff_msgs/Trigger.h>

// Eigen C++ includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// C++ STL inclues
#include <iostream>
#include <string>

// Gflag definitions
DEFINE_string(name, "marker", "Calibration tool frame name'");
DEFINE_double(height, 0.0062, "Calibration tool z position in global frame");

// Trannsormation buffer
tf2_ros::Buffer buffer_;

// Data structure containing average points
geometry_msgs::TransformStamped c_, x_, p_;

// Get the position and mark it
bool Mark(ff_msgs::Trigger::Request & req, ff_msgs::Trigger::Response & res,
  geometry_msgs::TransformStamped & tf) {
  try {
    // DIRECTLY FROM TF2 DOCUMENTATION...
    /////////////////////////////////////////////////////////////////////////////////
    // Transforms are specified in the direction such that they will transform     //
    // the coordinate frame "frame_id" into "child_frame_id" in                    //
    // tf::StampedTransform and geometry_msgs/TransformStamped. This is the        //
    // same transform which will take data from "child_frame_id" into "frame_id".  //
    /////////////////////////////////////////////////////////////////////////////////
    // The direction of the transform returned will be from the target_frame to    //
    // the source_frame. Which if applied to data, will transform data in the      //
    // source_frame into the target_frame.                                         //
    /////////////////////////////////////////////////////////////////////////////////
    // USGAE: lookupTransform(TARGET|FRAME_ID, SOURCE|CHILD_FRAME_ID, ...)
    geometry_msgs::TransformStamped old_tf = tf;
    tf = buffer_.lookupTransform("vive", FLAGS_name, ros::Time(0));
    // Print the transform
    ROS_INFO_STREAM("Transform:");
    ROS_INFO_STREAM(tf);
    Eigen::Vector3d vec(tf.transform.translation.x - old_tf.transform.translation.x,
                        tf.transform.translation.y - old_tf.transform.translation.y,
                        tf.transform.translation.z - old_tf.transform.translation.z);
    ROS_INFO_STREAM("Distance from last point in mm: " << vec.norm()*1000.0);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_STREAM("Could not get transform");
  }
  return true;
}

// Mark the X axis
bool MarkX(ff_msgs::Trigger::Request & req, ff_msgs::Trigger::Response & res) {
  return Mark(req, res, x_);
}

// Mark the corner
bool MarkC(ff_msgs::Trigger::Request & req, ff_msgs::Trigger::Response & res) {
  return Mark(req, res, c_);
}

// Mark the plane axis
bool MarkP(ff_msgs::Trigger::Request & req, ff_msgs::Trigger::Response & res) {
  return Mark(req, res, p_);
}

// Solve for the origin transform
bool Solve(ff_msgs::Trigger::Request &req, ff_msgs::Trigger::Response &res) {
  // Extract the points
  Eigen::Vector3d x = msg_conversions::ros_to_eigen_vector(x_.transform.translation);
  Eigen::Vector3d c = msg_conversions::ros_to_eigen_vector(c_.transform.translation);
  Eigen::Vector3d p = msg_conversions::ros_to_eigen_vector(p_.transform.translation);
  // Calculate the direction vectors
  Eigen::Vector3d v_x = (x - c).normalized();
  Eigen::Vector3d v_p = (c - p).normalized();
  Eigen::Vector3d v_z = v_x.cross(v_p).normalized();
  Eigen::Vector3d v_y = v_z.cross(v_x).normalized();
  // Obtain the rotation from vive -> world
  Eigen::Matrix3d vRc = Eigen::Matrix3d::Zero();
  vRc.block<3, 1>(0, 0) = v_x;
  vRc.block<3, 1>(0, 1) = v_y;
  vRc.block<3, 1>(0, 2) = v_z;
  Eigen::Quaterniond q(vRc);
  Eigen::Vector3d wPc(-(x - c).norm() / 2.0, (c - p).norm() / 2.0, -FLAGS_height);
  // Print out some debug code for a sanity check
  ROS_INFO_STREAM("X width: " << (x - c).norm());
  ROS_INFO_STREAM("Y width: " << (p - c).norm());
  // Eigen::Vector3d wPv = -q.inverse().matrix()*c + wPc;
  Eigen::Vector3d wPv = -q.inverse().matrix()*c + wPc;
  Eigen::Quaterniond wRv = q.inverse();
  // Send the message
  geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = ros::Time::now();
  tfs.header.frame_id = "world";
  tfs.child_frame_id = "vive";
  tfs.transform.rotation = msg_conversions::eigen_to_ros_quat(wRv);
  tfs.transform.translation = msg_conversions::eigen_to_ros_vector(wPv);
  // Now send the transform
  static tf2_ros::StaticTransformBroadcaster br;
  br.sendTransform(tfs);
  // Success!
  return true;
}

// Main entry point for application
int main(int argc, char *argv[]) {
  // Gather some data from the command
  google::SetUsageMessage("Usage: rosrun vive vive_calibrate <opts>");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize a ros node
  ros::init(argc, argv, "vive_calibrate", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  // Setup the TF2 listener for transforms
  tf2_ros::TransformListener listener(buffer_);
  // Default value from lab
  x_.transform.translation.x = -0.781168;
  x_.transform.translation.y =-0.816707;
  x_.transform.translation.z = -0.910298;
  c_.transform.translation.x = -0.748132;
  c_.transform.translation.y = -0.830513;
  c_.transform.translation.z = -2.7531;
  p_.transform.translation.x = 1.12511;
  p_.transform.translation.y = -0.822793;
  p_.transform.translation.z = -2.7224;
  // Declare services that help with marking, solving and clearing
  ros::ServiceServer srv_x = nh.advertiseService("/vive/set_x", MarkX);
  ros::ServiceServer srv_p = nh.advertiseService("/vive/set_p", MarkP);
  ros::ServiceServer srv_c = nh.advertiseService("/vive/set_c", MarkC);
  ros::ServiceServer srv_s = nh.advertiseService("/vive/solve", Solve);
  ros::spin();
  // Finish commandline flags
  google::ShutDownCommandLineFlags();
  // Make for great success
  return 0;
}
