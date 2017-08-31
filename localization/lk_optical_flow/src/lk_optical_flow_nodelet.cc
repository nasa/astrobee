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

#include <lk_optical_flow/lk_optical_flow_nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ff_msgs/CameraRegistration.h>

namespace lk_optical_flow {

LKOpticalFlowNodelet::LKOpticalFlowNodelet(void) : ff_util::FreeFlyerNodelet(NODE_OPTICAL_FLOW) {}

void LKOpticalFlowNodelet::Initialize(ros::NodeHandle* nh) {
  camera_id_ = 0;
  inst_.reset(new LKOpticalFlow());

  // Initialize lua config reader
  config_.AddFile("optical_flow.config");
  config_.AddFile("cameras.config");
  ReadParams();
  config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
    config_.CheckFilesUpdated(std::bind(&LKOpticalFlowNodelet::ReadParams, this));
  }, false, true);

  // Create all our subscribers and publishers
  image_transport::ImageTransport img_transp(*nh);
  img_sub_ = img_transp.subscribe(TOPIC_HARDWARE_NAV_CAM, 1, &LKOpticalFlowNodelet::ImageCallback, this);
  if (debug_view_) {
    image_transport::ImageTransport img_transp_priv(*nh);
    img_pub_ = img_transp_priv.advertise(TOPIC_LOCALIZATION_OF_DEBUG, 1);
  }

  feature_pub_ = nh->advertise<ff_msgs::Feature2dArray>(TOPIC_LOCALIZATION_OF_FEATURES, 1);
  reg_pub_ = nh->advertise<ff_msgs::CameraRegistration>(TOPIC_LOCALIZATION_OF_REGISTRATION, 1);

  enable_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_OF_ENABLE, &LKOpticalFlowNodelet::EnableService, this);
}

void LKOpticalFlowNodelet::ReadParams() {
  // Read config files into lua
  if (!config_.ReadFiles()) {
    ROS_ERROR("Error loading optical flow parameters.");
    return;
  }

  if (!config_.GetBool("debug_view", &debug_view_))
    ROS_FATAL("Unspecified debug_view.");

  inst_->ReadParams(&config_);
}

bool LKOpticalFlowNodelet::EnableService(ff_msgs::SetBool::Request & req, ff_msgs::SetBool::Response & res) {
  if (req.enable) {
    image_transport::ImageTransport img_transp(getNodeHandle());
    img_sub_ = img_transp.subscribe(TOPIC_HARDWARE_NAV_CAM, 1, &LKOpticalFlowNodelet::ImageCallback, this);
  } else {
    img_sub_.shutdown();
  }
  res.success = true;
  return true;
}


void LKOpticalFlowNodelet::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Signal to the EKF that we are processing this image. (The EKF must perform
  // state augmentation at this instant.)
  ff_msgs::CameraRegistration r;
  ros::Time timestamp = ros::Time::now();
  r.header = std_msgs::Header();
  r.header.stamp = timestamp;
  r.camera_id = ++camera_id_;
  reg_pub_.publish(r);
  ros::spinOnce();

  // actually process the optical flow image
  ff_msgs::Feature2dArray features;
  inst_->OpticalFlow(msg, &features);
  features.camera_id = camera_id_;

  // Publish corners
  feature_pub_.publish(features);
  // Publish optical flow image
  if (debug_view_)
    img_pub_.publish(inst_->ShowDebugWindow(msg));
}

};  // namespace lk_optical_flow

PLUGINLIB_DECLARE_CLASS(lk_optical_flow, LKOpticalFlowNodelet,
    lk_optical_flow::LKOpticalFlowNodelet, nodelet::Nodelet)

