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

#ifndef EXECUTIVE_UTILS_SEQUENCER_SV_POSE_VEL_ACCEL_H_
#define EXECUTIVE_UTILS_SEQUENCER_SV_POSE_VEL_ACCEL_H_

#include <stdint.h>

#include <Eigen/Core>

/* Defines heler functions and a class for the "ControlState" version
 * of a state vector.
 */

namespace sequencer {
namespace sv {
namespace pose_vel_accel {

extern const char* kName;

struct StateVector {
 public:
  uint32_t sec;
  uint32_t nsec;
  Eigen::Matrix<float, 19, 1> vec;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Helper functions to pull out the different parts of a state vector
template <typename Derived>
Eigen::Block<Derived, 3, 1> GetPosition(Eigen::MatrixBase<Derived>& state) {  // NOLINT
  return state.template block<3, 1>(0, 0);
}
template <typename Derived>
Eigen::Block<Derived, 3, 1> GetVelocity(Eigen::MatrixBase<Derived>& state) {  // NOLINT
  return state.template block<3, 1>(3, 0);
}
template <typename Derived>
Eigen::Block<Derived, 3, 1> GetAcceleration(Eigen::MatrixBase<Derived>& state) {  // NOLINT
  return state.template block<3, 1>(6, 0);
}
template <typename Derived>
Eigen::Block<Derived, 4, 1> GetQuaternion(Eigen::MatrixBase<Derived>& state) {  // NOLINT
  return state.template block<4, 1>(9, 0);
}
template <typename Derived>
Eigen::Block<Derived, 3, 1> GetAngularVelocity(Eigen::MatrixBase<Derived>& state) {  // NOLINT
  return state.template block<3, 1>(13, 0);
}
template <typename Derived>
Eigen::Block<Derived, 3, 1> GetAngularAcceleration(Eigen::MatrixBase<Derived>& state) {  // NOLINT
  return state.template block<3, 1>(16, 0);
}

// Const variants
template <typename Derived>
Eigen::Block<const Derived, 3, 1> GetPosition(Eigen::MatrixBase<Derived> const& state) {
  return state.template block<3, 1>(0, 0);
}
template <typename Derived>
Eigen::Block<const Derived, 3, 1> GetVelocity(Eigen::MatrixBase<Derived> const& state) {
  return state.template block<3, 1>(3, 0);
}
template <typename Derived>
Eigen::Block<const Derived, 3, 1> GetAcceleration(Eigen::MatrixBase<Derived> const& state) {
  return state.template block<3, 1>(6, 0);
}
template <typename Derived>
Eigen::Block<const Derived, 4, 1> GetQuaternion(Eigen::MatrixBase<Derived> const& state) {
  return state.template block<4, 1>(9, 0);
}
template <typename Derived>
Eigen::Block<const Derived, 3, 1> GetAngularVelocity(Eigen::MatrixBase<Derived> const& state) {
  return state.template block<3, 1>(13, 0);
}
template <typename Derived>
Eigen::Block<const Derived, 3, 1> GetAngularAcceleration(Eigen::MatrixBase<Derived> const& state) {
  return state.template block<3, 1>(16, 0);
}

}  // end namespace pose_vel_accel
}  // end namespace sv
}  // end namespace sequencer

#endif  // EXECUTIVE_UTILS_SEQUENCER_SV_POSE_VEL_ACCEL_H_

