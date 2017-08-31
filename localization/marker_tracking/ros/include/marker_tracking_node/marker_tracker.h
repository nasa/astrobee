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
#ifndef MARKER_TRACKING_NODE_MARKER_TRACKER_H_
#define MARKER_TRACKING_NODE_MARKER_TRACKER_H_

#include <marker_tracking/arconfigio.h>
#include <marker_tracking/marker_detector.h>

#include <camera/camera_params.h>
#include <ff_msgs/SetBool.h>
#include <ff_msgs/VisualLandmarks.h>

#include <image_transport/image_transport.h>
#include <ros/publisher.h>

#include <string>

namespace marker_tracking_node {

class MarkerTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit MarkerTracker(ros::NodeHandle* nh, ros::NodeHandle* private_nh,
                         const std::string& nm);
  ~MarkerTracker();

  void PublishVO(struct timeval const& timestamp);

 private:
  void VideoCallback(const sensor_msgs::ImageConstPtr& image_msg);
  bool EnableService(ff_msgs::SetBool::Request& req,
                     ff_msgs::SetBool::Response& res);
  void ReadParams(void);
  int EstimatePose(ff_msgs::VisualLandmarks* msg);

  config_reader::ConfigReader config_;
  ros::Timer config_timer_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer enable_srv_;
  ros::Publisher landmark_publisher_, reg_publisher_;

  marker_tracking::ARTagMap ar_tags_;
  const std::string& nodelet_name_;
  camera::CameraParameters camera_param_;
  std::shared_ptr<marker_tracking::MarkerCornerDetector> detector_;
};

};  // namespace marker_tracking_node

#endif  // MARKER_TRACKING_NODE_MARKER_TRACKER_H_
