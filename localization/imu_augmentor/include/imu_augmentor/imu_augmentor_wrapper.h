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
#ifndef IMU_AUGMENTOR_IMU_AUGMENTOR_WRAPPER_H_
#define IMU_AUGMENTOR_IMU_AUGMENTOR_WRAPPER_H_

#include <imu_augmentor/imu_augmentor.h>
#include <localization_measurements/imu_measurement.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>

namespace imu_augmentor {
class ImuAugmentorWrapper {
 public:
  ImuAugmentorWrapper();

  void LocalizationPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& localization_pose_msg);

  void ImuCallback(const sensor_msgs::Imu& imu_msg);

 private:
  std::unique_ptr<ImuAugmentor> imu_augmentor_;
  Eigen::Isometry3d body_T_imu_;
  Eigen::Vector3d gravity_vector_;
};
}  // namespace imu_augmentor
#endif  // IMU_AUGMENTOR_IMU_AUGMENTOR_WRAPPER_H_
