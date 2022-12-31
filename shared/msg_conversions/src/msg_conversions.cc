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

#include <msg_conversions/msg_conversions.h>

#include "rclcpp/rclcpp.hpp"
#define FF_FATAL_STREAM(...)   RCLCPP_FATAL_STREAM(LOGGER, __VA_ARGS__)

namespace msg_conversions {

// Setup logging for ROS2
static const rclcpp::Logger LOGGER = rclcpp::get_logger("msg_conversions");

Eigen::Vector3d ros_point_to_eigen_vector(const GeometryMsgs::Point& p) { return Eigen::Vector3d(p.x, p.y, p.z); }

Eigen::Vector3d ros_to_eigen_vector(const GeometryMsgs::Vector3& v) { return Eigen::Vector3d(v.x, v.y, v.z); }

GeometryMsgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d& v) {
  GeometryMsgs::Vector3 n;
  n.x = v[0];
  n.y = v[1];
  n.z = v[2];
  return n;
}

GeometryMsgs::Point eigen_to_ros_point(const Eigen::Vector3d& v) {
  GeometryMsgs::Point n;
  n.x = v[0];
  n.y = v[1];
  n.z = v[2];
  return n;
}

Eigen::Quaterniond ros_to_eigen_quat(const GeometryMsgs::Quaternion& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

GeometryMsgs::Quaternion eigen_to_ros_quat(const Eigen::Quaterniond& q) {
  GeometryMsgs::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

GeometryMsgs::Quaternion eigen_to_ros_quat(const Eigen::Vector4d& v) {
  GeometryMsgs::Quaternion out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
  out.w = v.w();
  return out;
}

GeometryMsgs::Vector3 array_to_ros_vector(float* array) {
  GeometryMsgs::Vector3 v;
  v.x = array[0];
  v.y = array[1];
  v.z = array[2];
  return v;
}

GeometryMsgs::Point array_to_ros_point(float* array) {
  GeometryMsgs::Point v;
  v.x = array[0];
  v.y = array[1];
  v.z = array[2];
  return v;
}

GeometryMsgs::Quaternion array_to_ros_quat(float* array) {
  GeometryMsgs::Quaternion q;
  q.x = array[0];
  q.y = array[1];
  q.z = array[2];
  q.w = array[3];
  return q;
}

void ros_to_array_vector(const GeometryMsgs::Vector3& v, float* array) {
  array[0] = v.x;
  array[1] = v.y;
  array[2] = v.z;
}

void ros_to_array_point(const GeometryMsgs::Point& p, float* array) {
  array[0] = p.x;
  array[1] = p.y;
  array[2] = p.z;
}

void ros_to_array_quat(const GeometryMsgs::Quaternion& q, float* array) {
  array[0] = q.x;
  array[1] = q.y;
  array[2] = q.z;
  array[3] = q.w;
}

void eigen_to_array_vector(const Eigen::Vector3d& v, float* array) {
  array[0] = v.x();
  array[1] = v.y();
  array[2] = v.z();
}

void eigen_to_array_quat(const Eigen::Quaterniond& q, float* array) {
  array[0] = q.x();
  array[1] = q.y();
  array[2] = q.z();
  array[3] = q.w();
}

bool config_read_quat(config_reader::ConfigReader* config, const char* name, Eigen::Quaterniond* quat) {
  config_reader::ConfigReader::Table t(config, name);
  return config_read_quat(&t, quat);
}

bool config_read_vector(config_reader::ConfigReader* config, const char* name, Eigen::Vector3d* vec) {
  config_reader::ConfigReader::Table t(config, name);
  return config_read_vector(&t, vec);
}

bool config_read_array(config_reader::ConfigReader* config, const char* name, int length, float* dest) {
  config_reader::ConfigReader::Table array(config, name);
  for (int i = 0; i < length; i++) {
    if (!array.GetReal(i + 1, &dest[i])) {
      return false;
    }
  }
  return true;
}

bool config_read_matrix(config_reader::ConfigReader* config, const char* name, int rows, int cols, float* dest) {
  config_reader::ConfigReader::Table matrix(config, name);
  int index = 0;
  for (int i = 1; i <= rows; i++) {
    config_reader::ConfigReader::Table row(&matrix, i);
    for (int j = 1; j <= cols; j++) {
      if (!row.GetReal(j, &dest[index])) {
        return false;
      }
      index++;
    }
  }
  return true;
}

bool config_read_transform(config_reader::ConfigReader* config, const char* name, Eigen::Vector3d* vec,
                           Eigen::Quaterniond* quat) {
  config_reader::ConfigReader::Table t(config, name);
  config_reader::ConfigReader::Table v(&t, "trans");
  config_reader::ConfigReader::Table r(&t, "rot");
  bool success = config_read_vector(&v, vec);
  success = success && config_read_quat(&r, quat);
  return success;
}

bool config_read_transform(config_reader::ConfigReader* config, const char* name, GeometryMsgs::Vector3* vec,
                           GeometryMsgs::Quaternion* quat) {
  config_reader::ConfigReader::Table t(config, name);
  config_reader::ConfigReader::Table v(&t, "trans");
  config_reader::ConfigReader::Table r(&t, "rot");
  bool success = config_read_vector(&v, vec);
  success = success && config_read_quat(&r, quat);
  return success;
}

bool config_read_transform(config_reader::ConfigReader* config, const char* name, Eigen::Affine3d* transform) {
  Eigen::Vector3d trans;
  Eigen::Quaterniond rot;
  if (!config_read_transform(config, name, &trans, &rot)) return false;
  *transform = Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) * Eigen::Affine3d(rot);
  return true;
}

bool config_read_transform(config_reader::ConfigReader* config, const char* name, GeometryMsgs::Transform* transform) {
  GeometryMsgs::Vector3 trans;
  GeometryMsgs::Quaternion rot;
  if (!config_read_transform(config, name, &trans, &rot)) return false;
  transform->translation = trans;
  transform->rotation = rot;
  return true;
}

bool config_read_quat(config_reader::ConfigReader::Table* t, Eigen::Quaterniond* quat) {
  bool success = t->GetReal("x", &quat->x());
  success = success && t->GetReal("y", &quat->y());
  success = success && t->GetReal("z", &quat->z());
  success = success && t->GetReal("w", &quat->w());
  return success;
}

bool config_read_vector(config_reader::ConfigReader::Table* t, Eigen::Vector3d* vec) {
  bool success = t->GetReal(1, &vec->x());
  success = success && t->GetReal(2, &vec->y());
  success = success && t->GetReal(3, &vec->z());
  return success;
}

bool config_read_quat(config_reader::ConfigReader::Table* t, GeometryMsgs::Quaternion* quat) {
  bool success = t->GetReal("x", &quat->x);
  success = success && t->GetReal("y", &quat->y);
  success = success && t->GetReal("z", &quat->z);
  success = success && t->GetReal("w", &quat->w);
  return success;
}

bool config_read_vector(config_reader::ConfigReader::Table* t, GeometryMsgs::Vector3* vec) {
  bool success = t->GetReal(1, &vec->x);
  success = success && t->GetReal(2, &vec->y);
  success = success && t->GetReal(3, &vec->z);
  return success;
}

bool config_read_vector(config_reader::ConfigReader::Table* t, GeometryMsgs::Point* point) {
  bool success = t->GetReal(1, &point->x);
  success = success && t->GetReal(2, &point->y);
  success = success && t->GetReal(3, &point->z);
  return success;
}

bool SingleBoolTrue(const std::initializer_list<bool>& bools) {
  int num_true_bools = 0;
  for (const auto bool_value : bools) {
    if (bool_value) ++num_true_bools;
    if (num_true_bools > 1) return false;
  }
  return num_true_bools == 1;
}

Eigen::Affine3d ros_pose_to_eigen_transform(const GeometryMsgs::Pose& p) {
  Eigen::Affine3d transform;
  transform.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  transform.linear() =
    Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix();
  return transform;
}

Eigen::Affine3d ros_to_eigen_transform(const GeometryMsgs::Transform& p) {
  Eigen::Affine3d transform;
  transform.translation() = Eigen::Vector3d(p.translation.x, p.translation.y, p.translation.z);
  transform.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).toRotationMatrix();
  return transform;
}

Eigen::Isometry3d LoadEigenTransform(config_reader::ConfigReader& config, const std::string& transform_config_name) {
  Eigen::Vector3d body_t_sensor;
  Eigen::Quaterniond body_Q_sensor;
  if (!msg_conversions::config_read_transform(&config, transform_config_name.c_str(), &body_t_sensor, &body_Q_sensor))
    FF_FATAL_STREAM("Unspecified transform config: " << transform_config_name);
  Eigen::Isometry3d body_T_sensor = Eigen::Isometry3d::Identity();
  body_T_sensor.translation() = body_t_sensor;
  body_T_sensor.linear() = body_Q_sensor.toRotationMatrix();
  return body_T_sensor;
}

float LoadFloat(config_reader::ConfigReader& config, const std::string& config_name) {
  float val;
  if (!config.GetReal(config_name.c_str(), &val)) FF_FATAL_STREAM("Failed to load " << config_name);
  return val;
}

double LoadDouble(config_reader::ConfigReader& config, const std::string& config_name) {
  double val;
  if (!config.GetReal(config_name.c_str(), &val)) FF_FATAL_STREAM("Failed to load " << config_name);
  return val;
}

int LoadInt(config_reader::ConfigReader& config, const std::string& config_name) {
  int val;
  if (!config.GetInt(config_name.c_str(), &val)) FF_FATAL_STREAM("Failed to load " << config_name);
  return val;
}

bool LoadBool(config_reader::ConfigReader& config, const std::string& config_name) {
  bool val;
  if (!config.GetBool(config_name.c_str(), &val)) FF_FATAL_STREAM("Failed to load " << config_name);
  return val;
}

std::string LoadString(config_reader::ConfigReader& config, const std::string& config_name) {
  std::string val;
  if (!config.GetStr(config_name.c_str(), &val)) FF_FATAL_STREAM("Failed to load " << config_name);
  return val;
}

void EigenPoseToMsg(const Eigen::Isometry3d& pose, GeometryMsgs::Pose& msg_pose) {
  RotationToMsg(Eigen::Quaterniond(pose.linear()), msg_pose.orientation);
  VectorToMsg(pose.translation(), msg_pose.position);
}

void EigenPoseToMsg(const Eigen::Isometry3d& pose, GeometryMsgs::Transform& msg_transform) {
  RotationToMsg(Eigen::Quaterniond(pose.linear()), msg_transform.rotation);
  VectorToMsg(pose.translation(), msg_transform.translation);
}

void VariancesToCovDiag(const Eigen::Vector3d& variances, float* const cov_diag) {
  cov_diag[0] = variances.x();
  cov_diag[1] = variances.y();
  cov_diag[2] = variances.z();
}

Eigen::Vector3d CovDiagToVariances(const float* const cov_diag) {
  return Eigen::Vector3d(cov_diag[0], cov_diag[1], cov_diag[2]);
}

void EigenPoseCovarianceToMsg(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& covariance,
                              GeometryMsgs::PoseWithCovarianceStamped& pose_cov_msg) {
  EigenPoseCovarianceToMsg(pose, covariance, pose_cov_msg.pose);
}

void EigenPoseCovarianceToMsg(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& covariance,
                              GeometryMsgs::PoseWithCovariance& pose_cov_msg) {
  EigenPoseToMsg(pose, pose_cov_msg.pose);
  EigenCovarianceToMsg(covariance, pose_cov_msg.covariance);
}
}  // end namespace msg_conversions
