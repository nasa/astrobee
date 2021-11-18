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

#ifndef MSG_CONVERSIONS_MSG_CONVERSIONS_H_
#define MSG_CONVERSIONS_MSG_CONVERSIONS_H_

#include <config_reader/config_reader.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

#include <string>

namespace msg_conversions {

Eigen::Vector3d ros_point_to_eigen_vector(const geometry_msgs::Point& p);
Eigen::Vector3d ros_to_eigen_vector(const geometry_msgs::Vector3& v);
geometry_msgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d& v);
void eigen_to_array_vector(const Eigen::Vector3d& v, float* array);
void ros_to_array_vector(const geometry_msgs::Vector3& v, float* array);
geometry_msgs::Vector3 array_to_ros_vector(float* array);

geometry_msgs::Point eigen_to_ros_point(const Eigen::Vector3d& v);
void ros_to_array_point(const geometry_msgs::Point& p, float* array);
geometry_msgs::Point array_to_ros_point(float* array);

Eigen::Quaterniond ros_to_eigen_quat(const geometry_msgs::Quaternion& q);
geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Quaterniond& q);
geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Vector4d& v);
void eigen_to_array_quat(const Eigen::Quaterniond& q, float* array);
void ros_to_array_quat(const geometry_msgs::Quaternion& q, float* array);
geometry_msgs::Quaternion array_to_ros_quat(float* array);
Eigen::Affine3d ros_pose_to_eigen_transform(const geometry_msgs::Pose& p);
Eigen::Affine3d ros_to_eigen_transform(const geometry_msgs::Transform& p);

// load from config file
bool config_read_quat(config_reader::ConfigReader* config, const char* name, Eigen::Quaterniond* quat);
bool config_read_vector(config_reader::ConfigReader* config, const char* name, Eigen::Vector3d* vec);
bool config_read_array(config_reader::ConfigReader* config, const char* name, int length, float* dest);
bool config_read_matrix(config_reader::ConfigReader* config, const char* name, int rows, int cols, float* dest);
bool config_read_transform(config_reader::ConfigReader* config, const char* name, Eigen::Vector3d* vec,
                           Eigen::Quaterniond* quat);
bool config_read_transform(config_reader::ConfigReader* config, const char* name, geometry_msgs::Vector3* vec,
                           geometry_msgs::Quaternion* quat);
bool config_read_transform(config_reader::ConfigReader* config, const char* name, Eigen::Affine3d* transform);
bool config_read_transform(config_reader::ConfigReader* config, const char* name, geometry_msgs::Transform* transform);

bool config_read_quat(config_reader::ConfigReader::Table* t, Eigen::Quaterniond* quat);
bool config_read_vector(config_reader::ConfigReader::Table* t, Eigen::Vector3d* vec);
bool config_read_quat(config_reader::ConfigReader::Table* t, geometry_msgs::Quaternion* quat);
bool config_read_vector(config_reader::ConfigReader::Table* t, geometry_msgs::Vector3* vec);
bool config_read_vector(config_reader::ConfigReader::Table* t, geometry_msgs::Point* point);

bool SingleBoolTrue(const std::initializer_list<bool>& bools);

// Alternative format for loading configs
Eigen::Isometry3d LoadEigenTransform(config_reader::ConfigReader& config, const std::string& transform_config_name);
double LoadDouble(config_reader::ConfigReader& config, const std::string& config_name);
float LoadFloat(config_reader::ConfigReader& config, const std::string& config_name);
int LoadInt(config_reader::ConfigReader& config, const std::string& config_name);
bool LoadBool(config_reader::ConfigReader& config, const std::string& config_name);
std::string LoadString(config_reader::ConfigReader& config, const std::string& config_name);
void EigenPoseToMsg(const Eigen::Isometry3d& pose, geometry_msgs::Pose& msg_pose);
void EigenPoseToMsg(const Eigen::Isometry3d& pose, geometry_msgs::Transform& msg_transform);
void VariancesToCovDiag(const Eigen::Vector3d& variances, float* const cov_diag);
Eigen::Vector3d CovDiagToVariances(const float* const cov_diag);
void EigenPoseCovarianceToMsg(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& covariance,
                              geometry_msgs::PoseWithCovarianceStamped& pose_cov_msg);
void EigenPoseCovarianceToMsg(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& covariance,
                              geometry_msgs::PoseWithCovariance& pose_cov_msg);

template <typename VectorType, typename MsgVectorType>
VectorType VectorFromMsg(const MsgVectorType& msg_vector) {
  return VectorType(msg_vector.x, msg_vector.y, msg_vector.z);
}

template <typename VectorType, typename MsgVectorType>
VectorType Vector2dFromMsg(const MsgVectorType& msg_vector) {
  return VectorType(msg_vector.x, msg_vector.y);
}

template <typename VectorType, typename MsgVectorType>
void VectorToMsg(const VectorType& vector, MsgVectorType& msg_vector) {
  msg_vector.x = vector.x();
  msg_vector.y = vector.y();
  msg_vector.z = vector.z();
}

template <typename VectorType, typename MsgVectorType>
void Vector2dToMsg(const VectorType& vector, MsgVectorType& msg_vector) {
  msg_vector.x = vector.x();
  msg_vector.y = vector.y();
}

template <typename RotationType, typename MsgRotationType>
RotationType RotationFromMsg(const MsgRotationType& msg_rotation) {
  return RotationType(msg_rotation.w, msg_rotation.x, msg_rotation.y, msg_rotation.z);
}

template <typename RotationType, typename MsgRotationType>
void RotationToMsg(const RotationType& rotation, MsgRotationType& msg_rotation) {
  msg_rotation.w = rotation.w();
  msg_rotation.x = rotation.x();
  msg_rotation.y = rotation.y();
  msg_rotation.z = rotation.z();
}

template <typename ArrayType>
void EigenCovarianceToMsg(const Eigen::Matrix<double, 6, 6>& covariance, ArrayType& covariance_array) {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      covariance_array[i*6 + j] = covariance(i, j);
    }
  }
}
}  // namespace msg_conversions

#endif  // MSG_CONVERSIONS_MSG_CONVERSIONS_H_
