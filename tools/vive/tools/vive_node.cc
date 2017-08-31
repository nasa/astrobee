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

// Standard messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// TF2 includes
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// FSW messages
#include <ff_msgs/ControlState.h>

// Config reader
#include <config_reader/config_reader.h>

// Message conversions
#include <msg_conversions/msg_conversions.h>

// OpenVR includes
#include <openvr.h>

// C++11 ST includes
#include <string>
#include <map>
#include <thread>
#include <mutex>

// The interface to the OpenVR application programmer interface
vr::IVRSystem *vr_ = nullptr;

// Map of tracking id - > serial
std::map<vr::TrackedDeviceIndex_t, std::string> map_;
std::mutex map_mutex_;

// Information about how to fuse multiple parents into a single frame
struct Fusion {
  std::string frame;                  // Frame name
  ros::Publisher pub;                 // Truth publisher
  std::vector<std::string> parents;   // Parent frames
};
std::map<int, Fusion> fusion_;

// Buffer for querying transforms
tf2_ros::Buffer buffer_;

// Convert a transform from the Vive frame to TF2 type
void TransformConversion(float (*M)[4], tf2::Vector3 & T, tf2::Quaternion & q) {
  static tf2::Matrix3x3 R;
  R.setValue(M[0][0], M[0][1], M[0][2],
             M[1][0], M[1][1], M[1][2],
             M[2][0], M[2][1], M[2][2]);
  R.getRotation(q);
  T.setValue(M[0][3], M[1][3], M[2][3]);
}

// Called whenever a new measurement must be generated
void TimerCallback(const ros::TimerEvent&) {
  // Check that we have a valid VR interface handle
  if (vr_ == nullptr) {
    ROS_WARN("TimerCallback called without a valid vr interface handle");
    return;
  }
  // Statically allocate some data structures to avoid reallocation
  static tf2_ros::TransformBroadcaster br;
  static vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];  // NOLINT
  static char str[vr::k_unMaxPropertyStringSize];
  static geometry_msgs::TransformStamped tfs, tfe, tfw;
  static geometry_msgs::PoseStamped pose;
  static tf2::Transform tf_w, tf_e;
  static tf2::Quaternion q, r;
  static tf2::Vector3 T;
  // Get poses for up to the maximum number of devices
  vr_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated,
    0, poses, vr::k_unMaxTrackedDeviceCount);
  // Iterate over all possible devices
  for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
    // Check the class of the device being tracked
    switch (vr_->GetTrackedDeviceClass(i)) {
    case vr::TrackedDeviceClass_Invalid:            // Not valid
    case vr::TrackedDeviceClass_DisplayRedirect:    // Display redirection
      continue;
    case vr::TrackedDeviceClass_HMD:                // HMD
    case vr::TrackedDeviceClass_Controller:         // Controller
    case vr::TrackedDeviceClass_GenericTracker:     // Tracker
    case vr::TrackedDeviceClass_TrackingReference:  // Lighthouse
      break;
    }
    // Check that its connected, the pose is valid and that tracking is running OK
    if (poses[i].eTrackingResult != vr::TrackingResult_Running_OK
    || !poses[i].bDeviceIsConnected
    || !poses[i].bPoseIsValid) continue;
    // Get the serial number of this device being tracked
    if (!vr_->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String,
      str, vr::k_unMaxPropertyStringSize)) {
      ROS_WARN_STREAM("Could not query serial number for device " << i << ". Ignoring.");
      continue;
    }
    // Add the serial number to the device table
    map_mutex_.lock();
    if (map_.find(i) == map_.end()) {
      ROS_INFO_STREAM("Detected endpoint " << i << " with serial " << str);
      map_[i] = str;
    }
    map_mutex_.unlock();
    // Convert the Vive transform to a TF2 transform
    TransformConversion(poses[i].mDeviceToAbsoluteTracking.m, T, q);
    // Send the transform
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "vive";
    tfs.child_frame_id = std::string(str);
    tfs.transform.translation.x = T.x();
    tfs.transform.translation.y = T.y();
    tfs.transform.translation.z = T.z();
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    br.sendTransform(tfs);
  }
  // Fusion phase - merge the overconstrained truth solutions into a single value
  for (std::map<int, Fusion>::iterator it = fusion_.begin(); it != fusion_.end(); it++) {
    size_t counter = 0;
    // Calculate the average transformation
    for (std::vector<std::string>::iterator jt = it->second.parents.begin();
      jt != it->second.parents.end(); ++jt) {
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
        tfs = buffer_.lookupTransform("vive", *jt, ros::Time(0));
        // Get the cumulative ratio
        double ratio = 1.0 / static_cast<double>(++counter);
        // Running average of position is easy
        tfe.transform.translation.x = ratio * tfs.transform.translation.x
                                    + (1.0 - ratio) * tfe.transform.translation.x;
        tfe.transform.translation.y = ratio * tfs.transform.translation.y
                                    + (1.0 - ratio) * tfe.transform.translation.y;
        tfe.transform.translation.z = ratio * tfs.transform.translation.z
                                    + (1.0 - ratio) * tfe.transform.translation.z;
        // Running average of quaternions is not so easy. See:
        // https:://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872_2007014421.pdf
        double dotproduct = tfe.transform.rotation.x * tfs.transform.rotation.x
                          + tfe.transform.rotation.y * tfs.transform.rotation.y
                          + tfe.transform.rotation.z * tfs.transform.rotation.z
                          + tfe.transform.rotation.w * tfs.transform.rotation.w;
        double sign = (dotproduct < 0.0 ? -1.0 : 1.0);
        // Normalize the quaternion
        q = tf2::Quaternion(
          sign * ratio * tfs.transform.rotation.x + (1.0 - ratio) * tfe.transform.rotation.x,
          sign * ratio * tfs.transform.rotation.y + (1.0 - ratio) * tfe.transform.rotation.y,
          sign * ratio * tfs.transform.rotation.z + (1.0 - ratio) * tfe.transform.rotation.z,
          sign * ratio * tfs.transform.rotation.w + (1.0 - ratio) * tfe.transform.rotation.w);
        q.normalize();
        // Convert from tf2 type
        tfe.transform.rotation.x = q.x();
        tfe.transform.rotation.y = q.y();
        tfe.transform.rotation.z = q.z();
        tfe.transform.rotation.w = q.w();
      }
      catch (tf2::TransformException &ex) {}
    }
    // Send the average transformation
    if (counter > 0) {
      // Send a TF2 message
      tfe.header.stamp = ros::Time::now();
      tfe.header.frame_id = "vive";
      tfe.child_frame_id = it->second.frame;
      br.sendTransform(tfe);
      // Send the world coordinate
      try {
        // Look up the transform
        tfw = buffer_.lookupTransform("world", "vive", ros::Time(0));
        // Combine the transforms and dump the result into the pose message
        tf2::convert(tfw.transform, tf_w);
        tf2::convert(tfe.transform, tf_e);
        // Combine the world -> vive and vive -> sensor transforms
        tf_e = tf_w * tf_e;
        // Make sure the pose header is good
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = tf_e.getOrigin().x();
        pose.pose.position.y = tf_e.getOrigin().y();
        pose.pose.position.z = tf_e.getOrigin().z();
        pose.pose.orientation.x = tf_e.getRotation().getAxis().x();
        pose.pose.orientation.y = tf_e.getRotation().getAxis().y();
        pose.pose.orientation.z = tf_e.getRotation().getAxis().z();
        pose.pose.orientation.w = tf_e.getRotation().getW();
        // Send the pose
        it->second.pub.publish(pose);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
      }
    } else {
      // ROS_WARN_STREAM("No transforms for fusion frame " << it->second.frame);
    }
  }
}

// Thread to manage incoming events that doens't block the main callback thread
void EventThread(bool *alive) {
  // Data to store the event and pose
  vr::VREvent_t event;
  vr::TrackedDevicePose_t pose;
  geometry_msgs::TransformStamped tfs;
  // Keep checking for events and pose updates
  while (*alive) {
    // Try and grab an event. If false, there were no events.
    if (!vr_->PollNextEventWithPose(vr::TrackingUniverseRawAndUncalibrated,
      &event, sizeof(event), &pose)) continue;
    // Print the event type
    switch (event.eventType) {
    case vr::VREvent_ButtonUnpress:
      ROS_INFO("Button pressed!");
      break;
    default:
      continue;
    }
    // TODO(andrew) : Don't hard-code the pointer name
    // TODO(andrew) : Use the pose supplied in the event
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
      tfs = buffer_.lookupTransform("world", "pointer", ros::Time(0));
      ROS_INFO_STREAM(tfs);
    } catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM("Position requested but pointer not found");
    }
  }
}

// Main entry point for tracking node
int main(int argc, char **argv) {
  // Default parameters
  double rate = 1.0;
  int retries = 5;
  // Initialize the ROS nodehandle
  ros::init(argc, argv, "vive");
  ros::NodeHandle nh;
  // Create a static transform broadcaster
  tf2_ros::TransformListener listener(buffer_);
  buffer_.setUsingDedicatedThread(true);
  tf2_ros::StaticTransformBroadcaster br;
  // Read configuration
  config_reader::ConfigReader config_params;
  config_params.AddFile("tools/vive.config");
  if (!config_params.ReadFiles())
    ROS_FATAL("Could not read the config file.");
  config_reader::ConfigReader::Table table;
  if (!config_params.GetTable("vive", &table))
    ROS_FATAL("Could not get the targets parameters.");
  if (!table.GetReal("rate", &rate))
    ROS_FATAL("Could not read the 'rate' parameter.");
  if (!table.GetInt("retries", &retries))
    ROS_FATAL("Could not read the 'retries' parameter.");
  // Create the static broadcasters
  config_reader::ConfigReader::Table transforms;
  if (!table.GetTable("transforms", &transforms))
    ROS_FATAL("Could not get the transforms parameters.");
  for (int i = 0; i < transforms.GetSize(); i++) {
    static config_reader::ConfigReader::Table transform, rotation, translation;
    if (!transforms.GetTable(i + 1, &transform))
      ROS_FATAL("Could not read row of transforms table");
    static std::string parent;
    if (!transform.GetStr("parent", &parent))
      ROS_FATAL("Could not read parameter: parent");
    static std::string child;
    if (!transform.GetStr("child", &child))
      ROS_FATAL("Could not read parameter: child");
    if (!transform.GetTable("translation", &translation))
      ROS_FATAL("Could not get the translation parameters.");
    if (!transform.GetTable("rotation", &rotation))
      ROS_FATAL("Could not get the rotation parameters.");
    static Eigen::Quaterniond q;
    if (!msg_conversions::config_read_quat(&rotation, &q))
      ROS_FATAL("Could not convert table to eigen quaternion");
    static Eigen::Vector3d T;
    if (!msg_conversions::config_read_vector(&translation, &T))
      ROS_FATAL("Could not convert table to eigen vector");
    // Normalize again just in case
    q.normalize();
    // Send the static transform
    static geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = parent;
    tfs.child_frame_id = child;
    tfs.transform.translation.x = T.x();
    tfs.transform.translation.y = T.y();
    tfs.transform.translation.z = T.z();
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    br.sendTransform(tfs);
  }
  // Create fusion metadata
  config_reader::ConfigReader::Table fusion;
  if (!table.GetTable("fusion", &fusion))
    ROS_FATAL("Could not get the fusion parameters.");
  for (int i = 0; i < fusion.GetSize(); i++) {
    config_reader::ConfigReader::Table row, parents;
    if (!fusion.GetTable(i + 1, &row))
      ROS_FATAL("Could not read row of fusion table");
    std::string topic;
    if (!row.GetStr("topic", &topic))
      ROS_FATAL("Could not read parameter: topic");
    fusion_[i].pub = nh.advertise<geometry_msgs::PoseStamped>(topic, 1);
    if (!row.GetStr("frame", &fusion_[i].frame))
      ROS_FATAL("Could not read parameter: frame");
    if (!row.GetTable("parents", &parents))
      ROS_FATAL("Could not get the parents parameters.");
    for (int j = 0; j < parents.GetSize(); j++) {
      config_reader::ConfigReader::Table group;
      if (!parents.GetTable(j + 1, &group))
        ROS_FATAL("Could not read row of parents table");
      std::string frame;
      if (!group.GetStr("frame", &frame))
        ROS_FATAL("Could not read parameter: frame");
      fusion_[i].parents.push_back(frame);
    }
  }
  // Initialize the VR system
  vr::EVRInitError error;
  do {
    // Check that we haven't exceeded the retry count
    if (retries-- < 0) {
      ROS_ERROR_STREAM("Could not connnect within 5 seconds.");
      return 1;
    }
    // Innocent before being treated as guilty
    error = vr::VRInitError_None;
    // Try and initialize
    vr_ = vr::VR_Init(&error, vr::VRApplication_Scene);
    // If the connection was not successful
    if (error != vr::VRInitError_None)
      ros::Duration(1.0).sleep();
  // Keep going
  } while (error != vr::VRInitError_None);
  // Initialize a single ROS timer
  ros::Timer timer = nh.createTimer(
    ros::Duration(ros::Rate(100)), TimerCallback, false, true);
  // Start listening for events in a separate thread
  bool alive = true;
  std::thread event_thread(EventThread, &alive);
  // Keep spinning until ROS shutdown
  ros::spin();
  // Rejoin thre thread
  alive = false;
  event_thread.join();
  // Destroy VR interface
  vr::VR_Shutdown();
  // Success!
  return 0;
}
