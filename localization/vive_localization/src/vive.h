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

#ifndef LOCALIZATION_VIVE_LOCALIZATION_SRC_VIVE_H_
#define LOCALIZATION_VIVE_LOCALIZATION_SRC_VIVE_H_

// FSW code
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>
#include <ff_util/ff_names.h>

// Messages
#include <geometry_msgs/TransformStamped.h>
#include <ff_hw_msgs/ViveLighthouses.h>
#include <ff_hw_msgs/ViveTrackers.h>
#include <ff_hw_msgs/ViveLight.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/VisualLandmarks.h>

// We will use OpenCV to bootstrap solution
#include <opencv2/calib3d/calib3d.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <string>
#include <vector>
#include <map>

/**
 * \ingroup localization
 */
namespace vive_localization {

// Universal constants
static constexpr size_t NUM_SENSORS = 32;

// ESSENTIAL STRUCTURES

// Lighthouse parameters
enum Params {
  PARAM_PHASE,
  PARAM_TILT,
  PARAM_GIB_PHASE,
  PARAM_GIB_MAG,
  PARAM_CURVE,
  NUM_PARAMS
};

// Lighthouse parameters
enum Errors {
  ERROR_GYR_BIAS,
  ERROR_GYR_SCALE,
  ERROR_ACC_BIAS,
  ERROR_ACC_SCALE,
  NUM_ERRORS
};

enum Motors {
  MOTOR_VERTICAL,
  MOTOR_HORIZONTAL,
  NUM_MOTORS
};

// Lighthouse data structure
struct Lighthouse {
  std::string serial;
  double vTl[6];
  double params[NUM_MOTORS*NUM_PARAMS];
  bool ready;
};
typedef std::map<uint8_t, Lighthouse> LighthouseMap;

// Tracker data structure
struct Tracker {
  double bTh[6];
  double tTh[6];
  double tTi[6];
  double sensors[NUM_SENSORS*6];
  double errors[NUM_ERRORS][3];
  bool ready;
};
typedef std::map<std::string, Tracker> TrackerMap;

// Pulse measurements
struct Measurement {
  double wTb[6];
  ff_hw_msgs::ViveLight light;
};
typedef std::map<ros::Time, Measurement> MeasurementMap;

// Correction data structure
typedef std::map<ros::Time, geometry_msgs::TransformStamped> CorrectionMap;

// TRANSFORM ENGINE

void SendStaticTransform(geometry_msgs::TransformStamped const& tfs);

void SendDynamicTransform(geometry_msgs::TransformStamped const& tfs);

void SendTransforms(
  std::string const& frame_world,   // World name
  std::string const& frame_vive,    // Vive frame name
  std::string const& frame_body,    // Body frame name
  double registration[6],
  LighthouseMap const& lighthouses, TrackerMap const& trackers);

// Convert a ceres to an Eigen transform
Eigen::Affine3d CeresToEigen(double ceres[6], bool invert = false);

// Convert a Euler angle to a Quaternion
Eigen::Quaterniond toQuaternion(double roll, double pitch, double yaw);

// CONFIG RELATED CALLS

// Read Vive trajectory modification parameter
bool ReadModificationVector(config_reader::ConfigReader *config,
  Eigen::Vector3d & modification_vector,
  Eigen::Quaterniond & modification_quaternion);

// Read lighthouse data
bool ReadLighthouseConfig(config_reader::ConfigReader *config,
  std::map<std::string, Eigen::Affine3d> & lighthouses);

// Read tracker data
bool ReadTrackerConfig(config_reader::ConfigReader *config,
  std::map<std::string, Tracker> & trackers);

// Read registration data
bool ReadRegistrationConfig(config_reader::ConfigReader *config, double T[6]);

// REUSABLE CALLS

void LighthouseCallback(ff_hw_msgs::ViveLighthouses::ConstPtr const& msg,
  LighthouseMap & lighthouses);

void TrackerCallback(ff_hw_msgs::ViveTrackers::ConstPtr const& msg,
  TrackerMap & trackers);

// Estimate a pose from (image) <-> (world) correspondences
bool SolvePnP(std::vector<cv::Point3f> const& obj,
              std::vector<cv::Point2f> const& img,
              Eigen::Affine3d & transform);

// TRACKING ROUTINES

// This algorithm solves the Procrustes problem in that it finds an affine transform
// (rotation, translation, scale) that maps the "in" matrix to the "out" matrix
// Code from: https://github.com/oleg-alexandrov/projects/blob/master/eigen/Kabsch.cpp
// License is that this code is release in the public domain... Thanks, Oleg :)
template <typename T>
static bool Kabsch(
  Eigen::Matrix<T, 3, Eigen::Dynamic> in,
  Eigen::Matrix<T, 3, Eigen::Dynamic> out,
  Eigen::Transform<T, 3, Eigen::Affine> &A, bool allowScale) {
  // Default output
  A.linear() = Eigen::Matrix<T, 3, 3>::Identity(3, 3);
  A.translation() = Eigen::Matrix<T, 3, 1>::Zero();
  // A simple check to see that we have a sufficient number of correspondences
  if (in.cols() < 4) {
    // ROS_WARN("Visualeyez needs to see at least four LEDs to track");
    return false;
  }
  // A simple check to see that we have a sufficient number of correspondences
  if (in.cols() != out.cols()) {
    // ROS_ERROR("Same number of points required in input matrices");
    return false;
  }
  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  T dist_in = T(0.0), dist_out = T(0.0);
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= T(0.0) || dist_out <= T(0.0))
    return true;
  T scale = T(1.0);
  if (allowScale) {
    scale = dist_out/dist_in;
    out /= scale;
  }
  // Find the centroids then shift to the origin
  Eigen::Matrix<T, 3, 1> in_ctr = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> out_ctr = Eigen::Matrix<T, 3, 1>::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= T(in.cols());
  out_ctr /= T(out.cols());
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }
  // SVD
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Cov = in * out.transpose();
  Eigen::JacobiSVD < Eigen::Matrix < T, Eigen::Dynamic, Eigen::Dynamic > > svd(Cov,
    Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Find the rotation
  T d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > T(0.0))
    d = T(1.0);
  else
    d = T(-1.0);
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix<T, 3, 3> R = svd.matrixV() * I * svd.matrixU().transpose();
  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);
  // Success
  return true;
}

// Given a point in space, predict the lighthouse angle
template <typename T>
static void Predict(T const* params, T const* xyz, T* ang, bool correct) {
  for (size_t i = 0; i < 2; i++) {
    T a =  xyz[2];
    T b = -xyz[0];
    T c =  xyz[1];
    if (i == 1) {
      a =  xyz[2];
      b = -xyz[1];
      c = -xyz[0];
    }
    // Ideal angle in radians [0, 2pi] relative to the start of the sweep
    ang[i] = atan2(a, b);
    // Perturb this angle based on the base station parameters
    if (correct) {
      T const* p = &params[i*NUM_PARAMS];
      // Motor correction
      ang[i] -= p[PARAM_PHASE];
      ang[i] -= asin((c * tan(p[PARAM_TILT])) / sqrt(a*a + b*b));
      // Lens correction
      ang[i] -= p[PARAM_GIB_MAG] * cos(ang[i] + p[PARAM_GIB_PHASE]);
      ang[i] -= p[PARAM_CURVE] * atan2(c, a) * atan2(c, a);
    }
  }
}

// Given the lighthouse angle, predict the point in space
template <typename T>
static void Correct(T const* params, T * angle, bool correct) {
  if (correct) {
    T ideal[2], pred[2], xyz[3];
    ideal[0] = angle[0];
    ideal[1] = angle[1];
    for (size_t i = 0; i < 10; i++) {
      xyz[0] = tan(ideal[0]);
      xyz[1] = tan(ideal[1]);
      xyz[2] = T(1.0);
      Predict(params, xyz, pred, correct);
      ideal[0] += (angle[0] - pred[0]);
      ideal[1] += (angle[1] - pred[1]);
    }
    angle[0] = ideal[0];
    angle[1] = ideal[1];
  }
}

}  // namespace vive_localization

#endif  // LOCALIZATION_VIVE_LOCALIZATION_SRC_VIVE_H_
