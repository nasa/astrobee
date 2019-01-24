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

#include <ekf_bag/ekf_bag_csv.h>

#include <camera/camera_params.h>
#include <Eigen/Core>

namespace ekf_bag {

Eigen::Vector3f QuatToEuler(const geometry_msgs::Quaternion & q) {
  Eigen::Vector3f euler;
  float q2q2 = q.y * q.y;
  euler.x() = atan2(2 * (q.x * q.w + q.y * q.z), 1 - 2 * (q.x * q.x + q2q2));
  float arg = std::max(-1.0, std::min(1.0, 2 * (q.y * q.w - q.x * q.z)));
  euler.y() = asin(arg);
  euler.z() = atan2(2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q2q2 + q.z * q.z));
  return euler;
}

Eigen::Vector3f QuatToEuler(const Eigen::Quaternionf & q) {
  Eigen::Vector3f euler;
  float q2q2 = q.y() * q.y();
  euler.x() = atan2(2 * (q.x() * q.w() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q2q2));
  float arg = std::max(-1.0f, std::min(1.0f, 2 * (q.y() * q.w() - q.x() * q.z())));
  euler.y() = asin(arg);
  euler.z() = atan2(2 * (q.x() * q.y() + q.z() * q.w()), 1 - 2 * (q2q2 + q.z() * q.z()));
  return euler;
}

EkfBagCsv::EkfBagCsv(const char* bagfile, const char* mapfile, const char* csvfile, bool run_ekf) :
          EkfBag(bagfile, mapfile, run_ekf),
          start_time_set_(false) {
  // virtual function has to be called in subclass since not initialized in superclass
  config_reader::ConfigReader config;
  ReadParams(&config);

  f_ = fopen(csvfile, "w");
  if (f_ == NULL) {
    fprintf(stderr, "Failed to open file %s.", csvfile);
    exit(0);
  }
}

EkfBagCsv::~EkfBagCsv(void) {
  fclose(f_);
}

void EkfBagCsv::ReadParams(config_reader::ConfigReader* config) {
  EkfBag::ReadParams(config);

  camera::CameraParameters cam_params(config, "nav_cam");
  tracked_of_.SetCameraParameters(cam_params);
}

void EkfBagCsv::UpdateEKF(const ff_msgs::EkfState & s) {
  EkfBag::UpdateEKF(s);

  if (!start_time_set_) {
    start_time_ = s.header.stamp;
    start_time_set_ = true;
  }
  fprintf(f_, "EKF %g ", (s.header.stamp - start_time_).toSec());
  fprintf(f_, "%g %g %g ", s.pose.position.x, s.pose.position.y, s.pose.position.z);
  Eigen::Vector3f euler = QuatToEuler(s.pose.orientation);
  fprintf(f_, "%g %g %g ", euler.x(), euler.y(), euler.z());
  fprintf(f_, "%g %g %g ", s.velocity.x, s.velocity.y, s.velocity.z);
  fprintf(f_, "%g %g %g ", s.omega.x, s.omega.y, s.omega.z);
  fprintf(f_, "%g %g %g ", s.accel.x, s.accel.y, s.accel.z);
  fprintf(f_, "%g %g %g ", s.accel_bias.x, s.accel_bias.y, s.accel_bias.z);
  fprintf(f_, "%g %g %g ", s.gyro_bias.x, s.gyro_bias.y, s.gyro_bias.z);
  fprintf(f_, "%d ", s.confidence);
  fprintf(f_, "%d ", s.status);
  fprintf(f_, "%d %d", s.ml_count, s.of_count);
  for (int i = 0; i < 15; i++)
    fprintf(f_, " %g", s.cov_diag[i]);
  for (int i = 0; i < 50; i++)
    fprintf(f_, " %g", s.ml_mahal_dists[i]);
  fprintf(f_, "\n");
}

void EkfBagCsv::UpdateOpticalFlow(const ff_msgs::Feature2dArray & of) {
  EkfBag::UpdateOpticalFlow(of);

  if (!start_time_set_)
    return;
  float t = (of.header.stamp - start_time_).toSec();
  tracked_of_.UpdateFeatures(of, t);
  fprintf(f_, "OF %g ", t);
  fprintf(f_, "%d ", static_cast<int>(of.feature_array.size()));
  for (auto it = tracked_of_.begin(); it != tracked_of_.end(); it++) {
    const auto & a = (*it).second;
    fprintf(f_, "%d %g %g %g ", a.id, a.time, a.x, a.y);
  }
  fprintf(f_, "\n");
}

void EkfBagCsv::UpdateSparseMap(const ff_msgs::VisualLandmarks & vl) {
  EkfBag::UpdateSparseMap(vl);

  if (!start_time_set_)
    return;
  const camera::CameraParameters & params = map_.GetCameraParameters();
  fprintf(f_, "VL %g ", (vl.header.stamp - start_time_).toSec());
  fprintf(f_, "%d ", static_cast<int>(vl.landmarks.size()));
  Eigen::Vector3f trans = ekf_.GetNavCamToBody().translation().cast<float>();
  Eigen::Quaternionf q1(0, trans.x(), trans.y(), trans.z());
  Eigen::Quaternionf q2(vl.pose.orientation.w, vl.pose.orientation.x, vl.pose.orientation.y, vl.pose.orientation.z);
  Eigen::Quaternionf cam_to_body_q(ekf_.GetNavCamToBody().rotation().cast<float>());
  cam_to_body_q.w() = -cam_to_body_q.w();
  q2 = q2 * cam_to_body_q;
  Eigen::Quaternionf t = (q2 * q1) * q2.conjugate();
  Eigen::Vector3f r(vl.pose.position.x, vl.pose.position.y, vl.pose.position.z);
  r -= Eigen::Vector3f(t.x(), t.y(), t.z());
  fprintf(f_, "%g %g %g ", r.x(), r.y(), r.z());
  Eigen::Vector3f euler = QuatToEuler(q2);
  fprintf(f_, "%g %g %g ", euler.x(), euler.y(), euler.z());
  for (unsigned int i = 0; i < vl.landmarks.size(); i++) {
    Eigen::Vector2d input(vl.landmarks[i].u, vl.landmarks[i].v);
    Eigen::Vector2d output;
    params.Convert<camera::UNDISTORTED_C, camera::DISTORTED_C>(input, &output);
    fprintf(f_, "%g %g %g %g %g ", output.x(), output.y(), vl.landmarks[i].x, vl.landmarks[i].y, vl.landmarks[i].z);
  }
  fprintf(f_, "\n");
}

void EkfBagCsv::UpdateGroundTruth(const geometry_msgs::PoseStamped & gt) {
  EkfBag::UpdateGroundTruth(gt);

  if (f_ == NULL || !start_time_set_)
    return;
  fprintf(f_, "GT %g ", (gt.header.stamp - start_time_).toSec());
  fprintf(f_, "%g %g %g ", gt.pose.position.x, gt.pose.position.y, gt.pose.position.z);
  Eigen::Vector3f euler = QuatToEuler(gt.pose.orientation);
  fprintf(f_, "%g %g %g\n", euler.x(), euler.y(), euler.z());
}

}  // namespace ekf_bag

