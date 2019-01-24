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

// ROS and TF2
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// STL
#include <fstream>
#include <sstream>

// This include
#include "./vive.h"

/**
 * \ingroup localization
 */
namespace vive_localization {

// TRANSFORM ENGINE

void SendStaticTransform(geometry_msgs::TransformStamped const& tfs) {
  static tf2_ros::StaticTransformBroadcaster bc;
  bc.sendTransform(tfs);
}

void SendDynamicTransform(geometry_msgs::TransformStamped const& tfs) {
  static tf2_ros::TransformBroadcaster bc;
  bc.sendTransform(tfs);
}

void SendTransforms(
  std::string const& frame_world,   // World name
  std::string const& frame_vive,    // Vive frame name
  std::string const& frame_body,    // Body frame name
  double registration[6],
  LighthouseMap const& lighthouses, TrackerMap const& trackers) {
  // Publish world -> vive transform
  {
    Eigen::Vector3d v(registration[3], registration[4], registration[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = frame_world;
    tfs.child_frame_id = frame_vive;
    tfs.transform.translation.x = registration[0];
    tfs.transform.translation.y = registration[1];
    tfs.transform.translation.z = registration[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
  // Publish lighthouse positions
  LighthouseMap::const_iterator it;
  for (it = lighthouses.begin(); it != lighthouses.end(); it++)  {
    Eigen::Vector3d v(it->second.vTl[3], it->second.vTl[4], it->second.vTl[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = frame_vive;
    tfs.child_frame_id = it->second.serial;
    tfs.transform.translation.x = it->second.vTl[0];
    tfs.transform.translation.y = it->second.vTl[1];
    tfs.transform.translation.z = it->second.vTl[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
  // Publish tracker extrinsics
  TrackerMap::const_iterator jt;
  for (jt = trackers.begin(); jt != trackers.end(); jt++)  {
    Eigen::Vector3d v(jt->second.bTh[3], jt->second.bTh[4], jt->second.bTh[5]);
    Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
    if (v.norm() > 0) {
      aa.angle() = v.norm();
      aa.axis() = v.normalized();
    }
    Eigen::Quaterniond q(aa);
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = frame_body;
    tfs.child_frame_id = jt->first;
    tfs.transform.translation.x = jt->second.bTh[0];
    tfs.transform.translation.y = jt->second.bTh[1];
    tfs.transform.translation.z = jt->second.bTh[2];
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    SendStaticTransform(tfs);
  }
}

// Convert a ceres to an Eigen transform
Eigen::Affine3d CeresToEigen(double ceres[6], bool invert) {
  Eigen::Affine3d A;
  A.translation()[0] = ceres[0];
  A.translation()[1] = ceres[1];
  A.translation()[2] = ceres[2];
  Eigen::Vector3d v(ceres[3], ceres[4], ceres[5]);
  Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
  if (v.norm() > 0) {
    aa.angle() = v.norm();
    aa.axis() = v.normalized();
  }
  A.linear() = aa.toRotationMatrix();
  if (invert)
    return A.inverse();
  return A;
}

// CONFIG CALLS

// Read lighthouse data
bool ReadLighthouseConfig(config_reader::ConfigReader *config,
  std::map<std::string, Eigen::Affine3d> & lighthouses) {
  config_reader::ConfigReader::Table lh(config, "lighthouses");
  for (int i = 0; i < lh.GetSize(); i++) {
    config_reader::ConfigReader::Table lighthouse;
    if (!lh.GetTable(i + 1, &lighthouse))
      return false;
    std::string serial;
    if (!lighthouse.GetStr("serial", &serial))
      return false;
    // Get the pose
    Eigen::Vector3d tf_trans;
    Eigen::Quaterniond tf_rot;
    config_reader::ConfigReader::Table pose, trans, rot;
    if (!lighthouse.GetTable("pose", &pose) ||
        !pose.GetTable("trans", &trans) ||
        !pose.GetTable("rot", &rot) ||
        !msg_conversions::config_read_quat(&rot, &tf_rot) ||
        !msg_conversions::config_read_vector(&trans, &tf_trans))
      return false;
    // Convert to an affine transform
    lighthouses[serial].linear() = tf_rot.toRotationMatrix();
    lighthouses[serial].translation() = tf_trans;
  }
  return true;
}

// Read lighthouse data
bool ReadTrackerConfig(config_reader::ConfigReader *config,
  std::map<std::string, Tracker> & trackers) {
  config_reader::ConfigReader::Table tr(config, "trackers");
  for (int i = 0; i < tr.GetSize(); i++) {
    config_reader::ConfigReader::Table tracker;
    if (!tr.GetTable(i + 1, &tracker))
      return false;
    std::string serial;
    if (!tracker.GetStr("serial", &serial))
      return false;
    // Get the pose
    Eigen::Vector3d tf_trans;
    Eigen::Quaterniond tf_rot;
    config_reader::ConfigReader::Table pose, trans, rot;
    if (!tracker.GetTable("pose", &pose) ||
        !pose.GetTable("trans", &trans) ||
        !pose.GetTable("rot", &rot) ||
        !msg_conversions::config_read_quat(&rot, &tf_rot) ||
        !msg_conversions::config_read_vector(&trans, &tf_trans))
      return false;
    // Convert to an affine transform
    Eigen::AngleAxisd aa(tf_rot);
    trackers[serial].bTh[0] = tf_trans[0];
    trackers[serial].bTh[1] = tf_trans[1];
    trackers[serial].bTh[2] = tf_trans[2];
    trackers[serial].bTh[3] = aa.angle() * aa.axis()[0];
    trackers[serial].bTh[4] = aa.angle() * aa.axis()[1];
    trackers[serial].bTh[5] = aa.angle() * aa.axis()[2];
  }
  return true;
}

// Read lighthouse data
bool ReadRegistrationConfig(config_reader::ConfigReader *config, double T[6]) {
  Eigen::Vector3d tf_trans;
  Eigen::Quaterniond tf_rot;
  config_reader::ConfigReader::Table reg, trans, rot;
  if (!config->GetTable("registration", &reg) ||
      !reg.GetTable("trans", &trans) ||
      !reg.GetTable("rot", &rot) ||
      !msg_conversions::config_read_quat(&rot, &tf_rot) ||
      !msg_conversions::config_read_vector(&trans, &tf_trans))
    return false;
  // Convert to an affine transform
  Eigen::AngleAxisd aa(tf_rot);
  T[0] = tf_trans[0];
  T[1] = tf_trans[1];
  T[2] = tf_trans[2];
  T[3] = aa.angle() * aa.axis()[0];
  T[4] = aa.angle() * aa.axis()[1];
  T[5] = aa.angle() * aa.axis()[2];
  return true;
}


// REUSABLE CALLS

void LighthouseCallback(ff_hw_msgs::ViveLighthouses::ConstPtr const& msg,
  LighthouseMap & lighthouses) {
  for (auto const& lighthouse : msg->lighthouses) {
    lighthouses[lighthouse.id].serial = lighthouse.serial;
    for (size_t i = 0; i < lighthouse.motors.size(); i++) {
      lighthouses[lighthouse.id].params[i*NUM_PARAMS + PARAM_PHASE]
        = lighthouse.motors[i].phase;
      lighthouses[lighthouse.id].params[i*NUM_PARAMS + PARAM_TILT]
        = lighthouse.motors[i].tilt;
      lighthouses[lighthouse.id].params[i*NUM_PARAMS + PARAM_GIB_PHASE]
        = lighthouse.motors[i].gibphase;
      lighthouses[lighthouse.id].params[i*NUM_PARAMS + PARAM_GIB_MAG]
        = lighthouse.motors[i].gibmag;
      lighthouses[lighthouse.id].params[i*NUM_PARAMS + PARAM_CURVE]
        = lighthouse.motors[i].curve;
    }
  }
}

void TrackerCallback(ff_hw_msgs::ViveTrackers::ConstPtr const& msg,
  TrackerMap & trackers) {
  // Iterate over the trackers in this message
  for (auto const& tracker : msg->trackers) {
    // Copy over the initial IMU errors
    trackers[tracker.serial].errors[ERROR_GYR_BIAS][0] = tracker.gyr_bias.x;
    trackers[tracker.serial].errors[ERROR_GYR_BIAS][1] = tracker.gyr_bias.y;
    trackers[tracker.serial].errors[ERROR_GYR_BIAS][2] = tracker.gyr_bias.z;
    trackers[tracker.serial].errors[ERROR_GYR_SCALE][0] = tracker.gyr_scale.x;
    trackers[tracker.serial].errors[ERROR_GYR_SCALE][1] = tracker.gyr_scale.y;
    trackers[tracker.serial].errors[ERROR_GYR_SCALE][2] = tracker.gyr_scale.z;
    trackers[tracker.serial].errors[ERROR_ACC_BIAS][0] = tracker.acc_bias.x;
    trackers[tracker.serial].errors[ERROR_ACC_BIAS][1] = tracker.acc_bias.y;
    trackers[tracker.serial].errors[ERROR_ACC_BIAS][2] = tracker.acc_bias.z;
    trackers[tracker.serial].errors[ERROR_ACC_SCALE][0] = tracker.acc_scale.x;
    trackers[tracker.serial].errors[ERROR_ACC_SCALE][1] = tracker.acc_scale.y;
    trackers[tracker.serial].errors[ERROR_ACC_SCALE][2] = tracker.acc_scale.z;
    // Add the sensor locations and normals
    for (size_t i = 0; i < tracker.sensors.size(); i++) {
      trackers[tracker.serial].sensors[6*i+0] = tracker.sensors[i].position.x;
      trackers[tracker.serial].sensors[6*i+1] = tracker.sensors[i].position.y;
      trackers[tracker.serial].sensors[6*i+2] = tracker.sensors[i].position.z;
      trackers[tracker.serial].sensors[6*i+3] = tracker.sensors[i].normal.x;
      trackers[tracker.serial].sensors[6*i+4] = tracker.sensors[i].normal.y;
      trackers[tracker.serial].sensors[6*i+5] = tracker.sensors[i].normal.z;
    }
    // Add the head -> light transform
    {
      // Write to the global data structure
      Eigen::Quaterniond q(
        tracker.pTh.rotation.w,
        tracker.pTh.rotation.x,
        tracker.pTh.rotation.y,
        tracker.pTh.rotation.z);
      Eigen::AngleAxisd aa(q);
      Eigen::Affine3d tTh;
      tTh.linear() = q.toRotationMatrix();
      tTh.translation() = Eigen::Vector3d(
        tracker.pTh.translation.x,
        tracker.pTh.translation.y,
        tracker.pTh.translation.z);
      trackers[tracker.serial].tTh[0] = tTh.translation()[0];
      trackers[tracker.serial].tTh[1] = tTh.translation()[1];
      trackers[tracker.serial].tTh[2] = tTh.translation()[2];
      trackers[tracker.serial].tTh[3] = aa.angle() * aa.axis()[0];
      trackers[tracker.serial].tTh[4] = aa.angle() * aa.axis()[1];
      trackers[tracker.serial].tTh[5] = aa.angle() * aa.axis()[2];
      // Send off the transform
      Eigen::Affine3d hTt = tTh.inverse();
      q = Eigen::Quaterniond(hTt.linear());
      geometry_msgs::TransformStamped tfs;
      tfs.header.frame_id = tracker.serial;
      tfs.child_frame_id = tracker.serial + "/light";
      tfs.transform.translation.x = hTt.translation()[0];
      tfs.transform.translation.y = hTt.translation()[1];
      tfs.transform.translation.z = hTt.translation()[2];
      tfs.transform.rotation.w = q.w();
      tfs.transform.rotation.x = q.x();
      tfs.transform.rotation.y = q.y();
      tfs.transform.rotation.z = q.z();
      SendStaticTransform(tfs);
    }
    // Add the imu -> light transform
    {
      // Write to the global data structure
      Eigen::Quaterniond q(
        tracker.pTi.rotation.w,
        tracker.pTi.rotation.x,
        tracker.pTi.rotation.y,
        tracker.pTi.rotation.z);
      Eigen::AngleAxisd aa(q);
      Eigen::Affine3d tTi;
      tTi.linear() = q.toRotationMatrix();
      tTi.translation() = Eigen::Vector3d(
        tracker.pTi.translation.x,
        tracker.pTi.translation.y,
        tracker.pTi.translation.z);
      trackers[tracker.serial].tTi[0] = tTi.translation()[0];
      trackers[tracker.serial].tTi[1] = tTi.translation()[1];
      trackers[tracker.serial].tTi[2] = tTi.translation()[2];
      trackers[tracker.serial].tTi[3] = aa.angle() * aa.axis()[0];
      trackers[tracker.serial].tTi[4] = aa.angle() * aa.axis()[1];
      trackers[tracker.serial].tTi[5] = aa.angle() * aa.axis()[2];
      // Send off the transform
      geometry_msgs::TransformStamped tfs;
      tfs.header.frame_id = tracker.serial + "/light";
      tfs.child_frame_id = tracker.serial + "/imu";
      tfs.transform = tracker.pTi;
      SendStaticTransform(tfs);
    }
  }
}

bool SolvePnP(std::vector<cv::Point3f> const& obj,
              std::vector<cv::Point2f> const& img,
              Eigen::Affine3d & transform) {
  if (obj.size() > 3) {
    cv::Mat cam = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    cv::Mat R(3, 1, cv::DataType<double>::type);
    cv::Mat T(3, 1, cv::DataType<double>::type);
    cv::Mat C(3, 3, cv::DataType<double>::type);
    if (cv::solvePnPRansac(obj, img, cam, cv::noArray(), R, T,  false,
      100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_EPNP)) {
      cv::Rodrigues(R, C);
      Eigen::Matrix3d rot;
      for (size_t r = 0; r < 3; r++)
        for (size_t c = 0; c < 3; c++)
          rot(r, c) = C.at<double>(r, c);
      transform.translation()[0] = T.at<double>(0, 0);
      transform.translation()[1] = T.at<double>(1, 0);
      transform.translation()[2] = T.at<double>(2, 0);
      transform.linear() = rot;
      return true;
    }
  }
  return false;
}

}  // namespace vive_localization
