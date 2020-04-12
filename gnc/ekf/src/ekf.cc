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

#include <ekf/ekf.h>
#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <common/init.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/SetEkfInput.h>
#include <gflags/gflags.h>
#include <msg_conversions/msg_conversions.h>
#include <ros/package.h>

DEFINE_bool(save_inputs_file, false, "Save the inputs to a file.");

namespace ekf {

Ekf::Ekf(void) :
  reset_ekf_(true), reset_ready_(false), reset_callback_(nullptr),
  processing_of_reg_(false), of_inputs_delayed_(false), output_file_(NULL),
  vl_camera_id_(0), of_camera_id_(0), dl_camera_id_(0), reset_dock_pose_(true) {
  gnc_.cmc_.speed_gain_cmd = 1;  // prevent from being invalid when running bags
  of_history_size_ = ASE_OF_NUM_AUG;
  of_max_features_ = ASE_OF_NUM_FEATURES;
  ml_max_features_ = ASE_ML_NUM_FEATURES;
  dl_max_features_ = ASE_ML_NUM_FEATURES;
  memset(&vis_,  0, sizeof(cvs_landmark_msg));
  memset(&reg_,  0, sizeof(cvs_registration_pulse));
  memset(&of_,   0, sizeof(cvs_optical_flow_msg));
  memset(&hand_, 0, sizeof(cvs_handrail_msg));
  memset(&imu_,  0, sizeof(imu_msg));


  bool output = FLAGS_save_inputs_file;
  if (output) {
    output_file_ = fopen("ekf_inputs.csv", "w");
    assert(output_file_);
    ROS_WARN("Recording EKF inputs. EKF *NOT* running.");
  }
}

Ekf::~Ekf() {
  if (output_file_) {
    fclose(output_file_);
    output_file_ = NULL;
  }
}

void Ekf::ReadParams(config_reader::ConfigReader* config) {
  gnc_.ReadParams(config);

  if (!config->GetInt("min_of_observations", &min_of_observations_))
    ROS_FATAL("Unspecified min_of_observations.");

  // get camera transform
  Eigen::Vector3d trans;
  Eigen::Quaterniond rot;
  if (!msg_conversions::config_read_transform(config, "nav_cam_transform", &trans, &rot))
    ROS_FATAL("Unspecified nav_cam_transform.");
  nav_cam_to_body_ = Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) *
                     Eigen::Affine3d(rot);
  if (!msg_conversions::config_read_transform(config, "dock_cam_transform", &trans, &rot))
    ROS_FATAL("Unspecified dock_cam_transform.");
  dock_cam_to_body_ = Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) *
                      Eigen::Affine3d(rot);
  if (!msg_conversions::config_read_transform(config, "perch_cam_transform", &trans, &rot))
    ROS_FATAL("Unspecified perch_cam_transform.");
  perch_cam_to_body_ = Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) *
                       Eigen::Affine3d(rot);
  if (!msg_conversions::config_read_transform(config, "imu_transform", &trans, &rot))
    ROS_FATAL("Unspecified imu_transform.");
  imu_to_body_ = Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) *
                       Eigen::Affine3d(rot);

  std::string imu_filename;
  if (!config->GetStr("imu_bias_file", &imu_filename)) {
    ROS_FATAL("IMU bias file not specified.");
  }

  // Modifying here some initial bias estimates. Also setting gravity subtract
  // to happen in the body frame instead of the world frame. Removing in the
  // world frame is fragile because vision could possibly rotate the
  // body_q_world slightly, making subtraction way off.
  auto& p = gnc_.est_->defaultParam;
  Eigen::Map<Eigen::Vector3f> gyro_fixed_bias(p->ase_gyro_fixed_bias),
                             accel_fixed_bias(p->ase_accel_fixed_bias);
  std::vector<real32_T> new_bias(3);
  FILE* f = fopen(imu_filename.c_str(), "r");
  if (f) {
    int ret = fscanf(f, "%g %g %g\n", &new_bias[0], &new_bias[1], &new_bias[2]);
    if (ret == 3)
      gyro_fixed_bias = Eigen::Vector3f(new_bias[0], new_bias[1], new_bias[2]);
    ret = fscanf(f, "%g %g %g\n", &new_bias[0], &new_bias[1], &new_bias[2]);
    if (ret == 3) {
      accel_fixed_bias = Eigen::Vector3f(new_bias[0], new_bias[1], new_bias[2]);
    }
    fclose(f);
  } else {
    ROS_WARN("No bias file found at %s.", imu_filename.c_str());
  }
}

void Ekf::SetBias(Eigen::Vector3f gyro_bias, Eigen::Vector3f accel_bias) {
    Eigen::Map<Eigen::Vector3f> gyro_fixed_bias(
                gnc_.est_->defaultParam->ase_gyro_fixed_bias),
                accel_fixed_bias(gnc_.est_->defaultParam->ase_accel_fixed_bias);
    gyro_fixed_bias = gyro_bias;
    accel_fixed_bias = accel_bias;
}

void Ekf::OpticalFlowUpdate(const ff_msgs::Feature2dArray & of) {
  // check that the camera id matches our registration
  if (of_camera_id_ != of.camera_id) {
    // ROS_DEBUG_THROTTLE(1, "Registered optical flow camera not found.");
    return;
  }

  // put new features in tree to make it faster
  std::map<int, OFObservation> new_features;
  for (size_t i = 0; i < of.feature_array.size(); i++) {
    new_features.insert(std::pair<int, OFObservation>(of.feature_array[i].id,
    {of.feature_array[i].x, of.feature_array[i].y}));
  }

  // delete all observations that we didn't see
  std::map<int, OFFeature>::iterator of_iter = optical_flow_features_.begin();
  while (of_iter != optical_flow_features_.end()) {
    if (new_features.count((*of_iter).first) == 0) {
      OFFeature & f = (*of_iter).second;
      f.missing_frames++;
    }
    of_iter++;
  }

  // now, add new features to the list of existing features
  for (size_t i = 0; i < of.feature_array.size(); i++) {
    uint16_t feature_id = of.feature_array[i].id;
    float x = of.feature_array[i].x;
    float y = of.feature_array[i].y;
    // add new feature to list
    if (optical_flow_features_.count(feature_id) == 0) {
      optical_flow_features_.insert(std::pair<int, OFFeature>(
            feature_id, {feature_id, std::vector<OFObservation>(), 0}));
    }
    OFFeature & f = optical_flow_features_[feature_id];
    // add the new observations
    f.obs.insert(f.obs.begin(), {x, y});
    optical_flow_augs_feature_counts_[0]++;
  }
  of_camera_id_ = 0;

  if (deleting_augs_.size() > 0)
    return;
  if (optical_flow_augs_feature_counts_.size() >= of_history_size_ &&
      optical_flow_augs_feature_counts_[of_history_size_ - 1] > 3) {
    deleting_augs_.push_back(0);
    if (optical_flow_augs_feature_counts_[of_history_size_ - 3] < 10) {
      deleting_augs_.push_back(of_history_size_ - 3);
      deleting_augs_.push_back(of_history_size_ - 2);
      deleting_augs_.push_back(of_history_size_ - 1);
    } else {
      deleting_augs_.push_back(2);
      if (optical_flow_augs_feature_counts_[of_history_size_ - 2] < 10) {
        deleting_augs_.push_back(of_history_size_ - 2);
        deleting_augs_.push_back(of_history_size_ - 1);
      } else {
        deleting_augs_.push_back(6);
        if (optical_flow_augs_feature_counts_[of_history_size_ - 1] < 10) {
          deleting_augs_.push_back(of_history_size_ - 1);
        } else {
          ros::Time now = of.header.stamp;
          double age_1 = (now - optical_flow_augs_times_[of_history_size_ - 1]).toSec();
          double age_2 = (now - optical_flow_augs_times_[of_history_size_ - 2]).toSec();
          double age_3 = (now - optical_flow_augs_times_[of_history_size_ - 3]).toSec();
          double age_4 = (now - optical_flow_augs_times_[of_history_size_ - 4]).toSec();
          if (age_1 < 2 * age_2)
            deleting_augs_.push_back(of_history_size_ - 2);
          else if (age_2 < 2 * age_3)
            deleting_augs_.push_back(of_history_size_ - 3);
          else if (age_3 < 2 * age_4)
            deleting_augs_.push_back(of_history_size_ - 4);
          else
            deleting_augs_.push_back(of_history_size_ - 5);
        }
      }
    }
  } else {
    return;
  }

  // clear all valid flags
  unsigned int index = 0;
  memset(of_.cvs_valid_flag, 0, of_history_size_ * of_max_features_);
  // we have to wait until the next timestep
  if (!processing_of_reg_) {
    of_.cvs_timestamp_sec = of.header.stamp.sec;
    of_.cvs_timestamp_nsec = of.header.stamp.nsec;
  } else {
    of_inputs_delayed_ = true;
  }
  std::map<int, OFFeature>::iterator iter = optical_flow_features_.begin();
  while (iter != optical_flow_features_.end()) {
    OFFeature & f = (*iter).second;
    if (f.missing_frames > 0) {
      // We are no longer using these observations because we can greatly speed up
      // computation in the EKF with a sparse block H matrix
      for (unsigned int i = 0; i < f.obs.size(); i++) {
        unsigned int aug = i + f.missing_frames;
        if (aug >= of_history_size_)
          break;
        // of_.cvs_observations[aug * of_max_features_ * 2 + index] = f.obs[i].x;
        // of_.cvs_observations[aug * of_max_features_ * 2 + index + of_max_features_] = f.obs[i].y;
        // of_.cvs_valid_flag[aug * of_max_features_ + index] = 1;
        if (optical_flow_augs_feature_counts_.size() > aug)
          optical_flow_augs_feature_counts_[aug]--;
      }
      iter = optical_flow_features_.erase(iter);
    } else {
      // only use oldest features, and choose three oldest frames and newest
      if (f.obs.size() >= of_history_size_) {
        of_.cvs_observations[0 * of_max_features_ * 2 + index] = f.obs[0].x;
        of_.cvs_observations[0 * of_max_features_ * 2 + index + of_max_features_] = f.obs[0].y;
        of_.cvs_valid_flag[0 * of_max_features_ + index] = 1;
        for (unsigned int aug = of_history_size_ - 3; aug < of_history_size_; aug++) {
          of_.cvs_observations[aug * of_max_features_ * 2 + index] = f.obs[aug].x;
          of_.cvs_observations[aug * of_max_features_ * 2 + index + of_max_features_] = f.obs[aug].y;
          of_.cvs_valid_flag[aug * of_max_features_ + index] = 1;
        }
      }
      iter++;
      index++;
    }
    if (index >= of_max_features_)
      break;
  }
  // just delete any missing features we didn't have space for
  while (iter != optical_flow_features_.end()) {
    OFFeature & f = (*iter).second;
    if (f.missing_frames > 0) {
      for (unsigned int i = 0; i < f.obs.size(); i++) {
        unsigned int aug = i + f.missing_frames;
        if (aug >= of_history_size_)
          break;
      //   of_.cvs_observations[aug * of_max_features_ * 2 + index] = f.obs[i].x;
      //   of_.cvs_observations[aug * of_max_features_ * 2 + index + of_max_features_] = f.obs[i].y;
      //   of_.cvs_valid_flag[aug * of_max_features_ + index] = 1;
        if (optical_flow_augs_feature_counts_.size() > aug)
          optical_flow_augs_feature_counts_[aug]--;
      }
      iter = optical_flow_features_.erase(iter);
    }
    iter++;
  }
}

void Ekf::SparseMapUpdate(const ff_msgs::VisualLandmarks & vl) {
  VisualLandmarksUpdate(vl);
  if (!output_file_ && reset_ekf_ && vl.landmarks.size() >= 5)
    ResetPose(nav_cam_to_body_, vl.pose);
  cmc_mode_ = ff_msgs::SetEkfInputRequest::MODE_MAP_LANDMARKS;
}

void Ekf::ResetAR(void) {
  reset_dock_pose_ = true;
}

void Ekf::ResetHR(void) {
  reset_handrail_pose_ = true;
}

bool Ekf::ARTagUpdate(const ff_msgs::VisualLandmarks & vl) {
  bool updated = false;
  if (reset_dock_pose_) {
    if (vl.landmarks.size() < 4)
      return false;
    geometry_msgs::Pose p;
    p.position = msg_conversions::array_to_ros_point(gnc_.kfl_.P_B_ISS_ISS);
    p.orientation = msg_conversions::array_to_ros_quat(gnc_.kfl_.quat_ISS2B);
    Eigen::Affine3d wTb = msg_conversions::ros_pose_to_eigen_transform(p);
    Eigen::Affine3d dTc = msg_conversions::ros_pose_to_eigen_transform(vl.pose);
    Eigen::Affine3d bTc = dock_cam_to_body_;
    dock_to_world_ = wTb * bTc * dTc.inverse();
    reset_dock_pose_ = false;
    updated = true;
  }

  ff_msgs::VisualLandmarks vl_mod = vl;
  for (unsigned int i = 0; i < vl.landmarks.size(); i++) {
    Eigen::Vector3d l(vl.landmarks[i].x, vl.landmarks[i].y, vl.landmarks[i].z);
    l = dock_to_world_ * l;
    vl_mod.landmarks[i].x = l.x();
    vl_mod.landmarks[i].y = l.y();
    vl_mod.landmarks[i].z = l.z();
  }
  VisualLandmarksUpdate(vl_mod);
  vl_mod.pose.position = msg_conversions::eigen_to_ros_point(
      dock_to_world_ * msg_conversions::ros_point_to_eigen_vector(vl.pose.position));
  Eigen::Quaterniond rot(dock_to_world_.linear());
  vl_mod.pose.orientation = msg_conversions::eigen_to_ros_quat(
      rot * msg_conversions::ros_to_eigen_quat(vl.pose.orientation));

  if (!output_file_ && reset_ekf_ && vl_mod.landmarks.size() >= 4)
    ResetPose(dock_cam_to_body_, vl_mod.pose);
  cmc_mode_ = ff_msgs::SetEkfInputRequest::MODE_AR_TAGS;

  return updated;
}

void Ekf::VisualLandmarksUpdate(const ff_msgs::VisualLandmarks & vl) {
  // check that the camera frame matches the one we just registered
  if (vl_camera_id_ != vl.camera_id) {
    ROS_DEBUG_THROTTLE(1, "Registered visual landmark camera %d not found.", vl.camera_id);
    return;
  }
  if (vl.landmarks.size() < 5)
    return;
  last_estimate_pose_= vl.pose;
  // find better way to choose limited landmarks to send?
  for (int i = 0; i < std::min(ml_max_features_, static_cast<int>(vl.landmarks.size())); i++) {
    vis_.cvs_landmarks[i]                        = vl.landmarks[i].x;
    vis_.cvs_landmarks[ml_max_features_ + i]     = vl.landmarks[i].y;
    vis_.cvs_landmarks[2 * ml_max_features_ + i] = vl.landmarks[i].z;
    vis_.cvs_observations[i]                     = vl.landmarks[i].u;
    vis_.cvs_observations[ml_max_features_ + i]  = vl.landmarks[i].v;
    vis_.cvs_valid_flag[i] = true;
  }
  for (int i = vl.landmarks.size(); i < ml_max_features_; i++)
    vis_.cvs_valid_flag[i] = false;

  vis_.cvs_timestamp_sec = vl.header.stamp.sec;
  vis_.cvs_timestamp_nsec = vl.header.stamp.nsec;
}

bool Ekf::HRTagUpdate(const ff_msgs::DepthLandmarks & dl) {
  bool updated = false;
  if (reset_handrail_pose_) {
    if (dl.landmarks.size() < 1)
      return false;
    geometry_msgs::Pose p;
    p.position = msg_conversions::array_to_ros_point(gnc_.kfl_.P_B_ISS_ISS);
    p.orientation = msg_conversions::array_to_ros_quat(gnc_.kfl_.quat_ISS2B);

    Eigen::Affine3d wTb = msg_conversions::ros_pose_to_eigen_transform(p);
    Eigen::Affine3d hTc = msg_conversions::ros_pose_to_eigen_transform(dl.local_pose);
    Eigen::Affine3d bTc = perch_cam_to_body_;
    handrail_to_world_ = wTb * bTc * hTc;
    ROS_WARN(" [EKF] Recalculated handrail_to_world transform.");
    reset_handrail_pose_ = false;
    updated = true;
  }
  ff_msgs::DepthLandmarks dl_mod = dl;
  for (unsigned int i = 0; i < dl.landmarks.size(); i++) {
    Eigen::Vector3d l(dl.landmarks[i].u, dl.landmarks[i].v, dl.landmarks[i].w);
    l = handrail_to_world_ * l;
    dl_mod.landmarks[i].u = l.x();
    dl_mod.landmarks[i].v = l.y();
    dl_mod.landmarks[i].w = l.z();
  }
  HandrailUpdate(dl);
  dl_mod.local_pose.position = msg_conversions::eigen_to_ros_point(
      handrail_to_world_ * msg_conversions::ros_point_to_eigen_vector(dl.local_pose.position));
  Eigen::Quaterniond rot(handrail_to_world_.linear());
  dl_mod.local_pose.orientation = msg_conversions::eigen_to_ros_quat(
      rot * msg_conversions::ros_to_eigen_quat(dl.local_pose.orientation));

  if (!output_file_ && reset_ekf_ && dl_mod.landmarks.size() >= 1)
    ResetPose(perch_cam_to_body_, dl_mod.local_pose);
  cmc_mode_ = ff_msgs::SetEkfInputRequest::MODE_HANDRAIL;

  return updated;
}

void Ekf::HandrailUpdate(const ff_msgs::DepthLandmarks & dl) {
  // check that the camera frame matches the one we just registered
  if (dl_camera_id_ != dl.camera_id) {
    ROS_DEBUG_THROTTLE(1, "Registered depth landmark camera not found.");
    return;
  }

  for (int i = 0; i < std::min(dl_max_features_, static_cast<int>(dl.landmarks.size())); i++) {
    hand_.cvs_observations[i]                         = dl.landmarks[i].u;
    hand_.cvs_observations[dl_max_features_ + i]      = dl.landmarks[i].v;
    hand_.cvs_observations[2 * dl_max_features_ + i]  = dl.landmarks[i].w;
    hand_.cvs_valid_flag[i] = true;
  }

  for (int i = dl.landmarks.size(); i < dl_max_features_; i++)
    hand_.cvs_valid_flag[i] = false;

  hand_.cvs_3d_knowledge_flag = dl.end_seen;

  if (reset_ekf_) {
    if (hand_.cvs_handrail_update_global_pose_flag == 0)
      hand_.cvs_handrail_update_global_pose_flag = 1;
    else
      reset_ekf_ = false;
  } else {
    hand_.cvs_handrail_update_global_pose_flag = 0;
  }

  hand_.cvs_handrail_local_pos[0] = dl.local_pose.position.x;
  hand_.cvs_handrail_local_pos[1] = dl.local_pose.position.y;
  hand_.cvs_handrail_local_pos[2] = dl.local_pose.position.z;
  hand_.cvs_handrail_local_quat[0] = dl.local_pose.orientation.x;
  hand_.cvs_handrail_local_quat[1] = dl.local_pose.orientation.y;
  hand_.cvs_handrail_local_quat[2] = dl.local_pose.orientation.z;
  hand_.cvs_handrail_local_quat[3] = dl.local_pose.orientation.w;

  hand_.cvs_timestamp_sec = dl.header.stamp.sec;
  hand_.cvs_timestamp_nsec = dl.header.stamp.nsec;
}

void Ekf::OpticalFlowRegister(const ff_msgs::CameraRegistration & cr) {
  processing_of_reg_ = true;
  unsigned int erased_aug = 0;
  if (of_camera_id_ != 0) {
    // ROS_WARN("Failed to get observations on time. Tossing last frame.");
    erased_aug = 0;
  } else {
    if (deleting_augs_.size() > 0) {
      erased_aug = deleting_augs_[0];
      deleting_augs_.erase(deleting_augs_.begin());
    } else {
      // choose the augmentation to delete
      if (optical_flow_augs_feature_counts_.size() < of_history_size_) {
        erased_aug = of_history_size_ - 1;
      } else if (optical_flow_augs_feature_counts_[of_history_size_ - 1] < 10) {
        erased_aug = of_history_size_ - 1;
      } else {
        erased_aug = 0;
      }
    }
    // delete the features from the deleted augmented state
    std::map<int, OFFeature>::iterator of_iter = optical_flow_features_.begin();
    while (of_iter != optical_flow_features_.end()) {
      OFFeature & f = (*of_iter).second;
      if (f.obs.size() > erased_aug) {
        f.obs.erase(f.obs.begin() + erased_aug);
      }
      if (f.obs.size() == 0)
        of_iter = optical_flow_features_.erase(of_iter);
      else
        of_iter++;
    }

    // update arrays of times and counts
    if (optical_flow_augs_feature_counts_.size() > of_history_size_) {
      optical_flow_augs_feature_counts_.erase(optical_flow_augs_feature_counts_.begin() + erased_aug);
      optical_flow_augs_times_.erase(optical_flow_augs_times_.begin() + erased_aug);
    }
    optical_flow_augs_feature_counts_.insert(optical_flow_augs_feature_counts_.begin(), 0);
    optical_flow_augs_times_.insert(optical_flow_augs_times_.begin(), cr.header.stamp);
  }

  // output to GNC
  reg_.cvs_optical_flow_pulse = of_history_size_ - erased_aug;
  of_camera_id_ = cr.camera_id;
}

void Ekf::SparseMapRegister(const ff_msgs::CameraRegistration & reg) {
  VisualLandmarksRegister(reg);
}

void Ekf::ARTagRegister(const ff_msgs::CameraRegistration & reg) {
  VisualLandmarksRegister(reg);
}

void Ekf::VisualLandmarksRegister(const ff_msgs::CameraRegistration & reg) {
  vl_camera_id_ = reg.camera_id;
  reg_.cvs_landmark_pulse = true;
}

void Ekf::HandrailRegister(const ff_msgs::CameraRegistration & reg) {
  dl_camera_id_ = reg.camera_id;
  reg_.cvs_handrail_pulse = true;
}

void Ekf::SetSpeedGain(const uint8_t gain) {
  gnc_.cmc_.speed_gain_cmd = gain;
}

void Ekf::SetResetCallback(std::function<void(void)> callback) {
  reset_callback_ = callback;
}

// this saves all the inputs to a file if an output file is specified,
// can be debugged later in matlab
void Ekf::WriteToFile(void) {
  // first output timestamp
  fprintf(output_file_, "%d, %d, ", gnc_.imu_.imu_timestamp_sec, gnc_.imu_.imu_timestamp_nsec);  // 1, 2
  // next all the imu data
  fprintf(output_file_, "%g, %g, %g, %g, %g, %g, ",  // 3, 4, 5, 6, 7, 8
          gnc_.imu_.imu_omega_B_ECI_sensor[0], gnc_.imu_.imu_omega_B_ECI_sensor[1], gnc_.imu_.imu_omega_B_ECI_sensor[2],
          gnc_.imu_.imu_A_B_ECI_sensor[0], gnc_.imu_.imu_A_B_ECI_sensor[1], gnc_.imu_.imu_A_B_ECI_sensor[2]);

  // now the registration pulses
  fprintf(output_file_, "%d, %d, ", gnc_.reg_.cvs_landmark_pulse, gnc_.reg_.cvs_optical_flow_pulse);  // 9, 10
  // next visual landmarks
  fprintf(output_file_, "%d, %d, ", gnc_.vis_.cvs_timestamp_sec, gnc_.vis_.cvs_timestamp_nsec);  // 11, 12
  // now the estimated pose
  fprintf(output_file_, "%g, %g, %g, ", last_estimate_pose_.position.x,  // 13, 14, 15
          last_estimate_pose_.position.y, last_estimate_pose_.position.z);
  fprintf(output_file_, "%g, %g, %g, %g, ", last_estimate_pose_.orientation.x,  // 16, 17, 18, 19
          last_estimate_pose_.orientation.y, last_estimate_pose_.orientation.z,
          last_estimate_pose_.orientation.w);
  // now the contents of cvs_landmarks. Stored as x0 x1 x2 ... y0 y1 y2... z0 z1 z2...
  for (int i = 0; i < ml_max_features_ * 3; i++)  // 20 ~ 169 (50 * 3)
    fprintf(output_file_, "%g, ", gnc_.vis_.cvs_landmarks[i]);
  // then the observations, again stored as u0 u1 u2... v0 v1 v2...
  for (int i = 0; i < ml_max_features_ * 2; i++)  // 170 ~ 269 (50 * 2)
    fprintf(output_file_, "%g, ", gnc_.vis_.cvs_observations[i]);
  // finally the valid flags
  for (int i = 0; i < ml_max_features_; i++)  // 270 ~ 319 (50)
    fprintf(output_file_, "%d, ", gnc_.vis_.cvs_valid_flag[i]);

  // next is the optical flow
  fprintf(output_file_, "%d, %d, ", gnc_.of_.cvs_timestamp_sec, gnc_.of_.cvs_timestamp_nsec);  // 320, 321
  // now the optical flow observations, stored as (xij is jth observation of feature i)
  // x00 x10 x20 x30 ... xn0
  // y00 y10 y20 y30 ... yn0
  // x01 x11 x21 x31 ... xn1
  // ...
  for (unsigned int i = 0; i < of_max_features_ * of_history_size_ * 2; i++)  // 322 ~ 1921 (50 * 2 * 16)
    fprintf(output_file_, "%g, ", gnc_.of_.cvs_observations[i]);
  // then valid flags, stored as
  // v00 v10 v20 v30 ... vn0
  // v01 v11 v21 v31 ... vn1
  // ...
  for (unsigned int i = 0; i < of_max_features_ * of_history_size_; i++)  // 1922 ~ 2721 (50 * 16)
    fprintf(output_file_, "%d, ", gnc_.of_.cvs_valid_flag[i]);

  // handrail registration pulses
  fprintf(output_file_, "%d, ", gnc_.reg_.cvs_handrail_pulse);  // 2722
  fprintf(output_file_, "%d, %d, ", gnc_.hand_.cvs_timestamp_sec, gnc_.hand_.cvs_timestamp_nsec);  // 2723, 2724
  // now the contents of handrail_landmarks. Stored as u0 u1 u2... v0 v1 v2... w0 w1 w2...
  for (int i = 0; i < dl_max_features_ * 3; i++)  // 2725 ~ 2874 (50 * 3)
    fprintf(output_file_, "%g, ", gnc_.hand_.cvs_observations[i]);

  // finally the valid flags
  for (int i = 0; i < dl_max_features_; i++)  // 2875 ~ 2924 (50)
    fprintf(output_file_, "%d, ", gnc_.hand_.cvs_valid_flag[i]);

  // now the handrail local pose
  fprintf(output_file_, "%g, %g, %g, ", gnc_.hand_.cvs_handrail_local_pos[0],  // 2925 ~ 2927 (3)
          gnc_.hand_.cvs_handrail_local_pos[1], gnc_.hand_.cvs_handrail_local_pos[2]);
  fprintf(output_file_, "%g, %g, %g, %g, ", gnc_.hand_.cvs_handrail_local_quat[0],  // 2928 ~ 2931 (4)
          gnc_.hand_.cvs_handrail_local_quat[1], gnc_.hand_.cvs_handrail_local_quat[2],
          gnc_.hand_.cvs_handrail_local_quat[3]);

  // handrail end point detection flag
  fprintf(output_file_, "%d, ", gnc_.hand_.cvs_3d_knowledge_flag);  // 2932
  // handrail global pose update flag
  fprintf(output_file_, "%d, ", gnc_.hand_.cvs_handrail_update_global_pose_flag);  // 2933

  // localization mode cmd
  fprintf(output_file_, "%d", gnc_.cmc_.localization_mode_cmd);   // 2934
  fprintf(output_file_, "\n");
}

void Ekf::PrepareStep(const sensor_msgs::Imu & imu, const geometry_msgs::Quaternion & quat) {
  // set IMU values
  // set timestamp
  imu_.imu_timestamp_sec  = imu.header.stamp.sec;
  imu_.imu_timestamp_nsec = imu.header.stamp.nsec;

  // set angular vel, ros message is double cast into float
  imu_.imu_omega_B_ECI_sensor[0] = static_cast<float>(imu.angular_velocity.x);
  imu_.imu_omega_B_ECI_sensor[1] = static_cast<float>(imu.angular_velocity.y);
  imu_.imu_omega_B_ECI_sensor[2] = static_cast<float>(imu.angular_velocity.z);

  // set linear accel
  imu_.imu_A_B_ECI_sensor[0] = static_cast<float>(imu.linear_acceleration.x);
  imu_.imu_A_B_ECI_sensor[1] = static_cast<float>(imu.linear_acceleration.y);
  imu_.imu_A_B_ECI_sensor[2] = static_cast<float>(imu.linear_acceleration.z);

  // set validity
  imu_.imu_validity_flag = 1;

  // set saturation
  imu_.imu_sat_flag = 0;

  // set the ISS2BODY quaternion in preparation for step - this is effectively ignored
  // if tun_ase_gravity_removal = true in gnc.config
  gnc_.quat_[0] = quat.x;
  gnc_.quat_[1] = quat.y;
  gnc_.quat_[2] = quat.z;
  gnc_.quat_[3] = quat.w;

  // then copy all other values in preparation for step
  memcpy(&gnc_.vis_,  &vis_,  sizeof(cvs_landmark_msg));
  memcpy(&gnc_.reg_,  &reg_,  sizeof(cvs_registration_pulse));
  memcpy(&gnc_.of_,   &of_,   sizeof(cvs_optical_flow_msg));
  memcpy(&gnc_.hand_, &hand_, sizeof(cvs_handrail_msg));
  memcpy(&gnc_.imu_,  &imu_,  sizeof(imu_msg));
  gnc_.cmc_.localization_mode_cmd = cmc_mode_;

  // prevent double registrations
  reg_.cvs_landmark_pulse = false;
  reg_.cvs_optical_flow_pulse = false;
  // registration complete, now update next time
  if (of_inputs_delayed_) {
    of_.cvs_timestamp_sec = imu.header.stamp.sec;
    of_.cvs_timestamp_nsec = imu.header.stamp.nsec;
    of_inputs_delayed_ = false;
  }
  processing_of_reg_ = false;

  ApplyReset();
}

int Ekf::Step(ff_msgs::EkfState* state) {
  if (output_file_)
    WriteToFile();
  else
    gnc_.Step();
  if (gnc_.kfl_.confidence == 2)
    reset_ekf_ = true;
  UpdateState(state);
  return 1;
}

void Ekf::UpdateState(ff_msgs::EkfState* state) {
  // now copy everything to the output message
  state->header.stamp.sec  = imu_.imu_timestamp_sec;
  state->header.stamp.nsec = imu_.imu_timestamp_nsec;
  state->header.frame_id   = "world";
  state->child_frame_id   = "body";
  state->pose.position    = msg_conversions::array_to_ros_point(gnc_.kfl_.P_B_ISS_ISS);
  state->velocity         = msg_conversions::array_to_ros_vector(gnc_.kfl_.V_B_ISS_ISS);
  state->accel            = msg_conversions::array_to_ros_vector(gnc_.kfl_.A_B_ISS_ISS);
  state->pose.orientation = msg_conversions::array_to_ros_quat(gnc_.kfl_.quat_ISS2B);
  state->omega            = msg_conversions::array_to_ros_vector(gnc_.kfl_.omega_B_ISS_B);
  state->accel_bias       = msg_conversions::array_to_ros_vector(gnc_.kfl_.accel_bias);
  state->gyro_bias        = msg_conversions::array_to_ros_vector(gnc_.kfl_.gyro_bias);
  state->confidence       = gnc_.kfl_.confidence;
  state->aug_state_enum   = gnc_.kfl_.aug_state_enum;
  state->status           = gnc_.kfl_.kfl_status;
  state->of_count         = gnc_.kfl_.update_OF_tracks_cnt;
  state->ml_count         = gnc_.kfl_.update_ML_features_cnt;
  std::copy(gnc_.kfl_.cov_diag, gnc_.kfl_.cov_diag + 15, state->cov_diag.c_array());
  state->hr_global_pose.position = msg_conversions::array_to_ros_point(gnc_.kfl_.hr_P_hr_ISS_ISS);
  state->hr_global_pose.orientation = msg_conversions::array_to_ros_quat(gnc_.kfl_.hr_quat_ISS2hr);
  if (state->ml_count > 0)
    std::copy(gnc_.kfl_.ml_mahal_distance, gnc_.kfl_.ml_mahal_distance + ml_max_features_,
            state->ml_mahal_dists.c_array());
  else
    memset(state->ml_mahal_dists.c_array(), 0, ml_max_features_ * sizeof(float));
}

void Ekf::Reset(void) {
  reset_ekf_ = true;
}

void Ekf::ResetPose(const Eigen::Affine3d & camera_to_body, geometry_msgs::Pose const& pose) {
  reset_camera_to_body_ = camera_to_body;
  reset_pose_ = pose;
  reset_ready_ = true;
}

// reset ekf, during step function to prevent race conditions
void Ekf::ApplyReset(void) {
  if (!reset_ready_)
    return;

  // set the robot's position based on the pose
  Eigen::Quaterniond world_q_body(reset_pose_.orientation.w, reset_pose_.orientation.x,
                                  reset_pose_.orientation.y, reset_pose_.orientation.z);
  Eigen::Quaterniond camera_to_body_rotation(reset_camera_to_body_.linear());
  Eigen::Vector3d world_r_body(reset_pose_.position.x, reset_pose_.position.y, reset_pose_.position.z);
  world_q_body = world_q_body * camera_to_body_rotation.conjugate();
  Eigen::Quaterniond q1(0, reset_camera_to_body_.translation().x(), reset_camera_to_body_.translation().y(),
                        reset_camera_to_body_.translation().z());
  Eigen::Quaterniond temp = world_q_body * q1 * world_q_body.conjugate();
  world_r_body = world_r_body - Eigen::Vector3d(temp.x(), temp.y(), temp.z());
  Eigen::Vector3d world_r_imu = world_r_body + world_q_body * imu_to_body_.translation();
  auto& quat_ISS2B    = gnc_.est_->defaultParam->tun_ase_state_ic_quat_ISS2B;
  auto& P_B_ISS_ISS   = gnc_.est_->defaultParam->tun_ase_state_ic_P_B_ISS_ISS;
  auto& P_EST_ISS_ISS = gnc_.est_->defaultParam->tun_ase_state_ic_P_EST_ISS_ISS;
  auto& V_B_ISS_ISS   = gnc_.est_->defaultParam->tun_ase_state_ic_V_B_ISS_ISS;
  quat_ISS2B[0] = world_q_body.x();
  quat_ISS2B[1] = world_q_body.y();
  quat_ISS2B[2] = world_q_body.z();
  quat_ISS2B[3] = world_q_body.w();
  P_B_ISS_ISS[0] = world_r_body[0];
  P_B_ISS_ISS[1] = world_r_body[1];
  P_B_ISS_ISS[2] = world_r_body[2];
  P_EST_ISS_ISS[0] = world_r_imu[0];
  P_EST_ISS_ISS[1] = world_r_imu[1];
  P_EST_ISS_ISS[2] = world_r_imu[2];
  V_B_ISS_ISS[0] = 0.0;
  V_B_ISS_ISS[1] = 0.0;
  V_B_ISS_ISS[2] = 0.0;

  ROS_INFO("Reset EKF.");

  // reset the EKF (especially for the covariance)
  gnc_.Initialize();

  // reset optical flow too
  optical_flow_features_.clear();
  optical_flow_augs_feature_counts_.clear();
  optical_flow_augs_times_.clear();
  of_camera_id_ = 0;
  processing_of_reg_ = false;
  of_inputs_delayed_ = false;

  reset_ready_ = false;
  reset_ekf_ = false;

  // If we have set the reset callback, call it now.
  if (reset_callback_)
    reset_callback_();
}

}  // end namespace ekf
