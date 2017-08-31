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

#ifndef LOCALIZATION_NODE_LOCALIZATION_NODELET_H_
#define LOCALIZATION_NODE_LOCALIZATION_NODELET_H_

#include <localization_node/localization.h>

#include <sparse_mapping/sparse_map.h>
#include <config_reader/config_reader.h>

#include <ff_msgs/SetBool.h>
#include <ff_util/ff_nodelet.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <thread>

namespace localization_node {

class LocalizationNodelet : public ff_util::FreeFlyerNodelet {
 public:
  LocalizationNodelet();
  virtual ~LocalizationNodelet();

 protected:
  virtual void Initialize(ros::NodeHandle* nh);

 private:
  void ReadParams(void);
  void Run(void);
  void Localize(void);
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  bool EnableService(ff_msgs::SetBool::Request & req, ff_msgs::SetBool::Response & res);

  std::shared_ptr<Localizer> inst_;
  std::shared_ptr<sparse_mapping::SparseMap> map_;
  std::shared_ptr<std::thread> thread_;
  config_reader::ConfigReader config_;
  ros::Timer config_timer_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer enable_srv_;
  ros::Publisher registration_publisher_, landmark_publisher_;
  bool enabled_;
  int count_;

  cv_bridge::CvImageConstPtr image_ptr_;

  volatile bool processing_image_;
  pthread_mutex_t mutex_features_;
  pthread_cond_t cond_features_;
};

};  // namespace localization_node

#endif  // LOCALIZATION_NODE_LOCALIZATION_NODELET_H_

