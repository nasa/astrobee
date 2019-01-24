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

#ifndef LOCALIZATION_VIVE_LOCALIZATION_SRC_VIVE_SOLVER_H_
#define LOCALIZATION_VIVE_LOCALIZATION_SRC_VIVE_SOLVER_H_

// ROS includes
#include <ros/ros.h>

// Common messages
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

// Our messages
#include <ff_hw_msgs/ViveLight.h>
#include <ff_hw_msgs/ViveLighthouses.h>
#include <ff_hw_msgs/ViveTrackers.h>

// Ceres and logging
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// STL
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <numeric>

// Local includes
#include "./vive.h"

/**
 * \ingroup localization
 */
namespace vive_localization {

////////////////////////
// ROTATION FUNCTIONS //
////////////////////////

// Helper function to apply a transform b = Ra + t
template <typename T> inline
void TransformInPlace(const T transform[6], T x[3]) {
  T tmp[3];
  ceres::AngleAxisRotatePoint(&transform[3], x, tmp);
  x[0] = tmp[0] + transform[0];
  x[1] = tmp[1] + transform[1];
  x[2] = tmp[2] + transform[2];
}

// Helper function to invert a transform a = R'(b - t)
template <typename T> inline
void InverseTransformInPlace(const T transform[6], T x[3]) {
  T aa[3], tmp[3];
  tmp[0] = x[0] - transform[0];
  tmp[1] = x[1] - transform[1];
  tmp[2] = x[2] - transform[2];
  aa[0] = -transform[3];
  aa[1] = -transform[4];
  aa[2] = -transform[5];
  ceres::AngleAxisRotatePoint(aa, tmp, x);
}

// Helper function to combine / multiply to transforms
// B ( A ( x ) )  = (B * A) (x)
//                = Rb * (Ra x + Ta) + Tb
//                = Rb * Ra * x + (Rb * Ta + Tb)
//                 | ROTATION |   | TRANSLATION |
template <typename T> inline
void CombineTransforms(const T b[6], const T a[6], T ba[6]) {
  T b_q[4], a_q[4], ba_q[4];
  // Calculate translation component
  for (size_t i = 0; i < 3; i++)
    ba[i] = a[i];
  TransformInPlace(b, ba);
  // Calculate rotation component
  ceres::AngleAxisToQuaternion(&a[3], a_q);
  ceres::AngleAxisToQuaternion(&b[3], b_q);
  ceres::QuaternionProduct(b_q, a_q, ba_q);
  ceres::QuaternionToAngleAxis(ba_q, &ba[3]);
}

// Helper function to calcuate the error between two transforms
template <typename T> inline
void TransformError(const T a[6], const T b[6], T err[6]) {
  T a_aa[3], b_aa[3], a_q[4], b_q[4], q[4];
  for (size_t i = 0; i < 3; i++) {
    err[i] = a[i] - b[i];
    a_aa[i] =  a[3+i];
    b_aa[i] = -b[3+i];
  }
  ceres::AngleAxisToQuaternion(a_aa, a_q);
  ceres::AngleAxisToQuaternion(b_aa, b_q);
  ceres::QuaternionProduct(a_q, b_q, q);
  ceres::QuaternionToAngleAxis(q, &err[3]);
}

////////////////////////////
// SMOOTHING COST FUNCTOR //
////////////////////////////

// Trajectory is a sequence of states
typedef std::map<ros::Time, double[6]> Trajectory;  // pos, att

// Residual error between sequential poses
struct SmoothCost {
  explicit SmoothCost(double smooth) : smooth_(smooth) {}
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const prev_state,   // PREV state
                  const T* const next_state,   // NEXT state
                  T* residual) const {
    // Error between two transforms
    TransformError(prev_state, next_state, residual);
    // Weighting
    for (size_t i = 0; i < 6; i++)
      residual[i] *= T(smooth_);
    // Success!
    return true;
  }

 private:
  double smooth_;   // Smoothing factor -- just a weighting
};

////////////////////////
// LIGHT COST FUNCTOR //
////////////////////////

//  Bundle of sensor measurements at a given instant in time
typedef std::map<ros::Time,               // Time
          std::map<uint8_t,                   // Lighthouse
            std::map<std::string,           // Tracker
              std::map<uint8_t,             // Sensor
                std::map<uint8_t,           // Axis
                  std::vector<double>       // Sample
                >
              >
            >
          >
        > LightMeasurements;

// Residual error between predicted angles to a lighthouse
struct LightCost {
  explicit LightCost(uint16_t sensor, double angles[2], bool correct)
    : sensor_(sensor), correct_(correct) {
    obs_[0] = angles[0];
    obs_[1] = angles[1];
  }
  // Called by ceres-solver to calculate error
  template <typename T>
  bool operator()(const T* const vTl,         // Lighthouse -> vive
                  const T* const vTb,         // Body -> vive
                  const T* const bTh,         // Head -> body
                  const T* const tTh,         // Head -> tracking (light)
                  const T* const sensors,     // Lighthouse calibration
                  const T* const params,      // Tracker extrinsics
                  T* residual) const {
    // Get the sensor position in the tracking frame
    T x[3], angle[2];
    x[0] = sensors[6 * sensor_ + 0];
    x[1] = sensors[6 * sensor_ + 1];
    x[2] = sensors[6 * sensor_ + 2];
    // Project the sensor position into the lighthouse frame
    InverseTransformInPlace(tTh, x);    // light -> head
    TransformInPlace(bTh, x);           // head -> body
    TransformInPlace(vTb, x);           // body -> world
    InverseTransformInPlace(vTl, x);    // vive -> lighthouse
    // Predict the angles - Note that the
    Predict(params, x, angle, correct_);
    // The residual angle error for the specific axis
    residual[0] = angle[0] - T(obs_[0]);
    residual[1] = angle[1] - T(obs_[1]);
    // Success!
    return true;
  }

 private:
  // Internal variables
  uint16_t sensor_;  // Sensor
  double obs_[2];    // Observation
  bool correct_;     // Whether to correct the light measurements
};

//////////////////
// SOLVER CLASS //
//////////////////

// Class to solve the localization problem offline
class Solver {
 public:
  explicit Solver(ros::NodeHandle nh);
  void Add(ff_hw_msgs::ViveTrackers::ConstPtr msg, ros::Time const& t);
  void Add(ff_hw_msgs::ViveLighthouses::ConstPtr msg, ros::Time const& t);
  void Add(ff_hw_msgs::ViveLight::ConstPtr msg, ros::Time const& t);
  void Add(geometry_msgs::PoseStamped::ConstPtr msg, ros::Time const& t);
  bool Calibrate();
  bool Refine();
  bool Register();
  bool Solve();
  void Publish(ros::Publisher & pub,
    std::string const& frame, std::map<ros::Time, double[6]> const& x);
  void PrintTransform(const double transform[6]);
  void GetTruth(nav_msgs::Path & path);
  void GetVive(nav_msgs::Path & path);

 private:
  double res_, smooth_, min_angle_, max_angle_, duration_;
  size_t  count_;
  bool correct_, calibrate_, refine_, register_,
    refine_extrinsics_, refine_sensors_, refine_head_, refine_params_;
  ceres::Solver::Options options_;          // Solver options
  double wTv_[6];                           // Registation
  std::map<std::string, Eigen::Affine3d> calibration_;
  LighthouseMap lighthouses_;               // Lighthouse info
  TrackerMap trackers_;                     // Tracker info
  Trajectory trajectory_;                   // Trajectory
  Trajectory correction_;                   // Corrections
  LightMeasurements light_;                 // Light measurements
  ros::Publisher pub_vive_, pub_ekf_;       // Path publisher for trajectory
};

}  // namespace vive_localization

#endif  // LOCALIZATION_VIVE_LOCALIZATION_SRC_VIVE_SOLVER_H_
