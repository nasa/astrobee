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

#ifndef EKF_BAG_EKF_BAG_H_
#define EKF_BAG_EKF_BAG_H_

#include <config_reader/config_reader.h>
#include <ekf/ekf.h>
#include <ff_util/ff_names.h>
#include <lk_optical_flow/lk_optical_flow.h>
#include <localization_node/localization.h>
#include <sparse_mapping/sparse_map.h>

#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>

#include <string>

namespace ekf_bag {

class EkfBag {
 public:
  EkfBag(const char* bagfile, const char* mapfile, bool run_ekf = true,
         bool gen_features = true, const char* biasfile = NULL,
         std::string image_topic = std::string(TOPIC_HARDWARE_NAV_CAM));
  virtual ~EkfBag(void);

  void Run(void);

 protected:
  virtual void ReadParams(config_reader::ConfigReader* config);

  virtual void UpdateImu(const ros::Time& time, const sensor_msgs::Imu& imu);
  virtual void UpdateImage(const ros::Time& time,
                           const sensor_msgs::ImageConstPtr& image);
  virtual void UpdateGroundTruth(const geometry_msgs::PoseStamped& pose);

  virtual void UpdateEKF(const ff_msgs::EkfState& state) {}
  // called when image received, not when finished processing
  virtual void UpdateOpticalFlow(const ff_msgs::Feature2dArray& of);
  virtual void UpdateSparseMap(const ff_msgs::VisualLandmarks& vl);
  virtual void UpdateOpticalFlowReg(const ff_msgs::CameraRegistration& reg);
  virtual void UpdateSparseMapReg(const ff_msgs::CameraRegistration& reg);

  rosbag::Bag bag_;
  sparse_mapping::SparseMap map_;
  localization_node::Localizer loc_;

  lk_optical_flow::LKOpticalFlow of_;

  ekf::Ekf ekf_;

  ros::Time bag_start_time_;

  // most recent ground truth pose
  geometry_msgs::Pose ground_truth_;

 private:
  void EstimateBias(void);

  bool run_ekf_;
  bool gen_features_;
  const char* bias_file_;
  std::string image_topic_;
  // configuration parameters
  float sparse_map_delay_, of_delay_;

  // variables used for sending visual features
  bool processing_of_, processing_sparse_map_;
  int of_id_, vl_id_;                      // message ids
  ros::Time of_send_time_, vl_send_time_;  // time to send features
  ff_msgs::Feature2dArray of_features_;    // save to send later
  ff_msgs::VisualLandmarks vl_features_;
};

}  // end namespace ekf_bag

#endif  // EKF_BAG_EKF_BAG_H_
