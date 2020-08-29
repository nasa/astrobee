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
#ifndef IMU_AUGMENTOR_IMU_AUGMENTOR_NODELET_H_
#define IMU_AUGMENTOR_IMU_AUGMENTOR_NODELET_H_

#include <ff_msgs/EkfState.h>
#include <ff_util/ff_nodelet.h>
#include <imu_augmentor/imu_augmentor_wrapper.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>

namespace imu_augmentor {
class ImuAugmentorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  ImuAugmentorNodelet();

 private:
  void Initialize(ros::NodeHandle* nh) final;

  void SubscribeAndAdvertise(ros::NodeHandle* nh);

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  void LocalizationStateCallback(const ff_msgs::EkfState::ConstPtr& loc_msg);

  void PublishLatestImuAugmentedLocalizationState();

  imu_augmentor::ImuAugmentorWrapper imu_augmentor_wrapper_;
  ros::Subscriber imu_sub_, state_sub_;
  ros::Publisher state_pub_;
};
}  // namespace imu_augmentor

#endif  // IMU_AUGMENTOR_IMU_AUGMENTOR_NODELET_H_
