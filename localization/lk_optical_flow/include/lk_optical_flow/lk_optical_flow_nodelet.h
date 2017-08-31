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

#ifndef LK_OPTICAL_FLOW_LK_OPTICAL_FLOW_NODELET_H_
#define LK_OPTICAL_FLOW_LK_OPTICAL_FLOW_NODELET_H_

#include <lk_optical_flow/lk_optical_flow.h>
#include <nodelet/nodelet.h>

#include <ff_msgs/SetBool.h>
#include <ff_util/ff_nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace lk_optical_flow {

class LKOpticalFlowNodelet : public ff_util::FreeFlyerNodelet {
 public:
  LKOpticalFlowNodelet();

 protected:
  virtual void Initialize(ros::NodeHandle* nh);

 private:
  void ReadParams(void);
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  bool EnableService(ff_msgs::SetBool::Request & req, ff_msgs::SetBool::Response & res);

  boost::shared_ptr<LKOpticalFlow> inst_;

  config_reader::ConfigReader config_;
  ros::Timer config_timer_;

  int camera_id_;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  ros::Publisher feature_pub_, reg_pub_;
  ros::Subscriber filtered_feature_sub_;
  ros::ServiceServer enable_srv_;

  bool debug_view_;
};
}  // namespace lk_optical_flow

#endif  // LK_OPTICAL_FLOW_LK_OPTICAL_FLOW_NODELET_H_
