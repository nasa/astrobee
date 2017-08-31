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

#include <sim_wrapper/sim.h>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <msg_conversions/msg_conversions.h>
#include <rosgraph_msgs/Clock.h>

#include <ff_msgs/SetBool.h>
#include <ff_util/ff_names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <camera/camera_model.h>
#include <sparse_mapping/reprojection.h>

#include <common/init.h>

// parameters sim_model_lib0_P are set in
//  matlab/code_generation/sim_model_lib0_ert_rtw/sim_model_lib0_data.c

namespace sim_wrapper {
Sim::Sim(ros::NodeHandle* nh) :
  cam_params_(Eigen::Vector2i(0, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0)),
  cur_time_(0, 0), ml_camera_count_(0), ar_camera_count_(0), of_camera_count_(0), dl_camera_count_(0),
  en_landmark_(false), en_tag_(false), en_optical_(false), en_depth_(false),
  pmc_command_id_(0) {
  config_.AddFile("cameras.config");
  config_.AddFile("geometry.config");
  config_.AddFile("gnc.config");

  ReadParams();
  gnc_.Initialize();

  // TODO(bcoltin): load these from gnc autocode
  of_history_size_ = 4;
  of_max_features_ = 50;
  ml_max_features_ = 50;
  hr_max_features_ = 50;
  ar_max_features_ = 50;

  // initialize ROS
  config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
    config_.CheckFilesUpdated(std::bind(&Sim::ReadParams, this));
  }, false, true);

  pub_imu_ = nh->advertise<sensor_msgs::Imu>(TOPIC_HARDWARE_IMU, 5);
  pub_pmc_ = nh->advertise<ff_hw_msgs::PmcTelemetry>(TOPIC_HARDWARE_PMC_TELEMETRY, 5);
  pub_clock_ = nh->advertise<rosgraph_msgs::Clock>(TOPIC_CLOCK, 1);
  // Mapped landmarks
  pub_landmark_camera_ = nh->advertise<sensor_msgs::Image>(TOPIC_HARDWARE_NAV_CAM, 1);
  srv_landmark_enable_ = nh->advertiseService(SERVICE_LOCALIZATION_ML_ENABLE, &Sim::LandmarkEnable, this);
  pub_landmark_pulse_ = nh->advertise<ff_msgs::CameraRegistration>(TOPIC_LOCALIZATION_ML_REGISTRATION, 1);
  pub_landmarks_ = nh->advertise<ff_msgs::VisualLandmarks>(TOPIC_LOCALIZATION_ML_FEATURES, 1);
  // Optical flow
  pub_optical_ = nh->advertise<ff_msgs::Feature2dArray>(TOPIC_LOCALIZATION_OF_FEATURES, 1);
  srv_optical_enable_ = nh->advertiseService(SERVICE_LOCALIZATION_OF_ENABLE, &Sim::OpticalEnable, this);
  pub_optical_pulse_ = nh->advertise<ff_msgs::CameraRegistration>(TOPIC_LOCALIZATION_OF_REGISTRATION, 1);
  // AR tags
  pub_ar_tags_camera_ = nh->advertise<sensor_msgs::Image>(TOPIC_HARDWARE_DOCK_CAM, 1);
  srv_ar_tags_enable_ = nh->advertiseService(SERVICE_LOCALIZATION_AR_ENABLE, &Sim::TagEnable, this);
  pub_ar_tags_pulse_ = nh->advertise<ff_msgs::CameraRegistration>(TOPIC_LOCALIZATION_AR_REGISTRATION, 1);
  pub_ar_tags_ = nh->advertise<ff_msgs::VisualLandmarks>(TOPIC_LOCALIZATION_AR_FEATURES, 1);
  // Handrail localization
  pub_depth_camera_ = nh->advertise<sensor_msgs::PointCloud2>((std::string) TOPIC_HARDWARE_PICOFLEXX_PREFIX
      + (std::string) TOPIC_HARDWARE_NAME_PERCH_CAM + (std::string) TOPIC_HARDWARE_PICOFLEXX_SUFFIX, 1);
  srv_depth_enable_ = nh->advertiseService(SERVICE_LOCALIZATION_HR_ENABLE, &Sim::DepthEnable, this);
  pub_depth_ = nh->advertise<ff_msgs::DepthLandmarks>(TOPIC_LOCALIZATION_HR_FEATURES, 1);
  pub_depth_pulse_ = nh->advertise<ff_msgs::CameraRegistration>(TOPIC_LOCALIZATION_HR_REGISTRATION, 1);
  // Ground truth
  pub_truth_pose_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_TRUTH, 1);
  // FAM callback
  sub_fam_pmc_ = nh->subscribe(TOPIC_HARDWARE_PMC_COMMAND, 5, &Sim::PmcFamCallBack, this,
                               ros::TransportHints().tcpNoDelay());
}

Sim::~Sim() {}

void Sim::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }

  gnc_.ReadParams(&config_);

  cam_params_ = camera::CameraParameters(&config_, "nav_cam");

  auto& p = gnc_.sim_->defaultParam;
  std::string imu_filename;
  if (!config_.GetStr("imu_bias_file", &imu_filename)) {
    ROS_FATAL("IMU bias file not specified.");
  }
  std::string bias_file = std::string(common::GetConfigDir()) + std::string("/") + imu_filename;

  // set the biases to what the EKF expects
  Eigen::Map<Eigen::Vector3f> accel_bias(p->epson_accel_bias_ic);
  Eigen::Map<Eigen::Vector3d> gyro_bias(p->epson_gyro_bias_ic);
  std::vector<real32_T> new_bias(3);
  FILE* f = fopen(bias_file.c_str(), "r");
  if (f) {
    int ret = fscanf(f, "%g %g %g\n", &new_bias[0], &new_bias[1], &new_bias[2]);
    if (ret == 3)
      gyro_bias = Eigen::Vector3d(new_bias[0], new_bias[1], new_bias[2]);
    ret = fscanf(f, "%g %g %g\n", &new_bias[0], &new_bias[1], &new_bias[2]);
    if (ret == 3)
      accel_bias = Eigen::Vector3f(new_bias[0], new_bias[1], new_bias[2]);
    fclose(f);
  } else {
    ROS_WARN("No bias file found at %s.", bias_file.c_str());
  }
}

void Sim::PmcFamCallBack(ff_hw_msgs::PmcCommand::ConstPtr const& pmc) {
  gnc_.act_msg_.act_timestamp_sec  = cur_time_.sec;
  gnc_.act_msg_.act_timestamp_nsec = cur_time_.nsec;

  gnc_.act_msg_.act_impeller_speed_cmd[0] = pmc->states[0].motor_speed;
  gnc_.act_msg_.act_impeller_speed_cmd[1] = pmc->states[1].motor_speed;
  for (int i = 0; i < 6; i++) {
    gnc_.act_msg_.act_servo_pwm_cmd[i] = pmc->states[0].nozzle_positions[i];
    gnc_.act_msg_.act_servo_pwm_cmd[6 + i] = pmc->states[1].nozzle_positions[i];
  }
}

void Sim::Step() {
  gnc_.Step();

  // Publish the clock msg from the simulator
  rosgraph_msgs::Clock clock_msg;
  clock_msg.clock.sec = cur_time_.sec;
  clock_msg.clock.nsec = cur_time_.nsec;
  pub_clock_.publish(clock_msg);

  cur_time_.nsec += 16000000;
  if (cur_time_.nsec >= 1000000000) {
    cur_time_.nsec -= 1000000000;
    cur_time_.sec++;
  }

  ////////////////////////////////////////
  // Publish all the simulated messages //
  ////////////////////////////////////////
  // Ground truth
  PublishTruth();
  // IMU
  if (PullImuMsg())
    pub_imu_.publish(ros_imu_);
  // PMC
  if (PullPmcMsg())
    pub_pmc_.publish(ros_pmc_);
  // 1 - Mapped landmarks
  if (PullLandmarkMsg())
    if (en_landmark_) pub_landmarks_.publish(ros_landmark_);
  if (PullLandmarkPulseMsg()) {
    if (en_landmark_) pub_landmark_pulse_.publish(ros_reg_pulse_);
    if (en_landmark_) pub_landmark_camera_.publish(sensor_msgs::Image());
  }
  // 2 - AR tags
  if (PullTagMsg())
    if (en_tag_) pub_ar_tags_.publish(ros_tag_);
  if (PullTagPulseMsg()) {
    if (en_tag_) pub_ar_tags_pulse_.publish(ros_reg_pulse_);
    if (en_tag_) pub_ar_tags_camera_.publish(sensor_msgs::Image());
  }
  // 3 - Optical flow
  if (PullOpticalMsg())
    if (en_optical_) pub_optical_.publish(ros_optical_);
  if (PullOpticalPulseMsg())
    if (en_optical_) pub_optical_pulse_.publish(ros_reg_pulse_);
  // 4 - Handrail
  if (PullDepthMsg())
    if (en_depth_) pub_depth_.publish(ros_depth_);
  if (PullDepthPulseMsg()) {
    if (en_depth_) pub_depth_camera_.publish(sensor_msgs::PointCloud2());
    if (en_depth_) pub_depth_pulse_.publish(ros_reg_pulse_);
  }
}

void Sim::PublishTruth() {
  ros_truth_pose_.header.seq++;
  ros_truth_pose_.header.stamp.sec = gnc_.ex_time_msg_.timestamp_sec;
  ros_truth_pose_.header.stamp.nsec = gnc_.ex_time_msg_.timestamp_nsec;
  ros_truth_pose_.header.frame_id = "world";
  ros_truth_pose_.pose.position = msg_conversions::array_to_ros_point(gnc_.env_msg_.P_B_ISS_ISS);
  ros_truth_pose_.pose.orientation = msg_conversions::array_to_ros_quat(gnc_.env_msg_.Q_ISS2B);
  pub_truth_pose_.publish(ros_truth_pose_);
}

bool Sim::PullImuMsg() {
  // check if value is updated
  // publish on every tick

  ros_imu_.header.stamp = ros::Time::now();
  ros_imu_.header.frame_id = "body";

  // set angular vel, ros message is double cast from float
  ros_imu_.angular_velocity.x = static_cast<double>(gnc_.imu_msg_.imu_omega_B_ECI_sensor[0]);
  ros_imu_.angular_velocity.y = static_cast<double>(gnc_.imu_msg_.imu_omega_B_ECI_sensor[1]);
  ros_imu_.angular_velocity.z = static_cast<double>(gnc_.imu_msg_.imu_omega_B_ECI_sensor[2]);

  // set linear accel
  ros_imu_.linear_acceleration.x = static_cast<double>(gnc_.imu_msg_.imu_A_B_ECI_sensor[0]);
  ros_imu_.linear_acceleration.y = static_cast<double>(gnc_.imu_msg_.imu_A_B_ECI_sensor[1]);
  ros_imu_.linear_acceleration.z = static_cast<double>(gnc_.imu_msg_.imu_A_B_ECI_sensor[2]);

  return true;
}

bool Sim::PullPmcMsg() {
  ros_pmc_.header.stamp.sec  = gnc_.act_msg_.act_timestamp_sec;
  ros_pmc_.header.stamp.nsec = gnc_.act_msg_.act_timestamp_nsec;
  ros_pmc_.header.frame_id = "body";
  ros_pmc_.statuses.clear();

  for (int i = 0; i < 2; i++) {
    ff_hw_msgs::PmcStatus s;
    s.motor_speed = gnc_.bpm_msg_.bpm_motor_speed[i];
    s.motor_current = gnc_.bpm_msg_.bpm_motor_curr[i];
    s.pressure = -1;
    s.status_1 = 0;
    s.status_2 = 0;
    s.command_id = pmc_command_id_;
    ros_pmc_.statuses.push_back(s);
  }
  pmc_command_id_++;

  return true;
}

bool Sim::PullLandmarkPulseMsg() {
  if (!(!last_landmark_pulse_ && gnc_.reg_pulse_.cvs_landmark_pulse > 0)) {
    // early exit condition, not a rising edge
    last_landmark_pulse_ = gnc_.reg_pulse_.cvs_landmark_pulse > 0;
    return false;
  }

  ros_reg_pulse_.camera_id = ++ml_camera_count_;

  last_landmark_pulse_ = gnc_.reg_pulse_.cvs_landmark_pulse > 0;
  return true;
}

bool Sim::PullLandmarkMsg() {
  if (last_landmark_time_.sec == gnc_.landmark_msg_.cvs_timestamp_sec &&
      last_landmark_time_.nsec == gnc_.landmark_msg_.cvs_timestamp_nsec) {
    return false;
  }

  // Count how many landmarks there are
  size_t landmark_count = 0;
  for (auto flag : gnc_.landmark_msg_.cvs_valid_flag) {
    if (flag > 0) {
      landmark_count++;
    }
  }
  std::vector<Eigen::Vector3d> landmarks, inlier_landmarks;
  std::vector<Eigen::Vector2d> observations, inlier_observations;
  landmarks.resize(landmark_count);
  observations.resize(landmark_count);
  size_t output_i = 0;
  for (size_t i = 0; i < ml_max_features_; i++) {
    if (gnc_.landmark_msg_.cvs_valid_flag[i] > 0) {
      landmarks[output_i].x() = gnc_.landmark_msg_.cvs_landmarks[i];
      landmarks[output_i].y() = gnc_.landmark_msg_.cvs_landmarks[ml_max_features_ + i];
      landmarks[output_i].z() = gnc_.landmark_msg_.cvs_landmarks[2 * ml_max_features_ + i];
      observations[output_i].x() = gnc_.landmark_msg_.cvs_observations[i];
      observations[output_i].y() = gnc_.landmark_msg_.cvs_observations[ml_max_features_ + i];
      output_i++;
    }
  }
  camera::CameraModel camera(Eigen::Vector3d(), Eigen::Matrix3d::Identity(), cam_params_);
  if (sparse_mapping::RansacEstimateCamera(landmarks, observations, 1000, 3,
      &camera, &inlier_landmarks, &inlier_observations))
    return false;
  Eigen::Affine3d global_pose = camera.GetTransform().inverse();
  Eigen::Quaterniond quat(global_pose.rotation());

  ros_landmark_.header.seq++;
  ros_landmark_.header.stamp.sec = gnc_.landmark_msg_.cvs_timestamp_sec;
  ros_landmark_.header.stamp.nsec = gnc_.landmark_msg_.cvs_timestamp_nsec;
  ros_landmark_.camera_id = ml_camera_count_;

  ros_landmark_.pose.position = msg_conversions::eigen_to_ros_point(global_pose.translation());
  ros_landmark_.pose.orientation = msg_conversions::eigen_to_ros_quat(quat);
  assert(inlier_landmarks.size() == inlier_observations.size());
  ros_landmark_.landmarks.resize(inlier_landmarks.size());
  for (size_t i = 0; i < inlier_landmarks.size(); i++) {
    ff_msgs::VisualLandmark l;
    l.x = inlier_landmarks[i].x();
    l.y = inlier_landmarks[i].y();
    l.z = inlier_landmarks[i].z();
    l.u = inlier_observations[i].x();
    l.v = inlier_observations[i].y();
    ros_landmark_.landmarks[i] = l;
  }

  last_landmark_time_.sec = gnc_.landmark_msg_.cvs_timestamp_sec;
  last_landmark_time_.nsec = gnc_.landmark_msg_.cvs_timestamp_nsec;
  return true;
}

bool Sim::LandmarkEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res) {
  en_landmark_ = req.enable;
  res.success = true;
  return true;
}

/* TODO(Jesse, Brian) Implement this for AR tag? */
bool Sim::PullTagPulseMsg() {
  if (!(!last_tag_pulse_ && gnc_.reg_pulse_.cvs_ar_tag_pulse > 0)) {
    // early exit condition, not a rising edge
    last_tag_pulse_ = gnc_.reg_pulse_.cvs_ar_tag_pulse > 0;
    return false;
  }
  ros_reg_pulse_.camera_id = ++ar_camera_count_;
  last_tag_pulse_ = gnc_.reg_pulse_.cvs_ar_tag_pulse > 0;
  return true;
}

/* TODO(Jesse, Brian) Implement this for AR tag? */
bool Sim::PullTagMsg() {
  if (last_tag_time_.sec == gnc_.ar_tag_msg_.cvs_timestamp_sec &&
      last_tag_time_.nsec == gnc_.ar_tag_msg_.cvs_timestamp_nsec) {
    return false;
  }

  // Count how many landmarks there are
  size_t landmark_count = 0;
  for (auto flag : gnc_.ar_tag_msg_.cvs_valid_flag) {
    if (flag > 0) {
      landmark_count++;
    }
  }

  std::vector<Eigen::Vector3d> landmarks, inlier_landmarks;
  std::vector<Eigen::Vector2d> observations, inlier_observations;
  landmarks.resize(landmark_count);
  observations.resize(landmark_count);
  size_t output_i = 0;
  for (size_t i = 0; i < ar_max_features_; i++) {
    if (gnc_.ar_tag_msg_.cvs_valid_flag[i] > 0) {
      landmarks[output_i].x() = gnc_.ar_tag_msg_.cvs_landmarks[i];
      landmarks[output_i].y() = gnc_.ar_tag_msg_.cvs_landmarks[ar_max_features_ + i];
      landmarks[output_i].z() = gnc_.ar_tag_msg_.cvs_landmarks[2 * ar_max_features_ + i];
      observations[output_i].x() = gnc_.ar_tag_msg_.cvs_observations[i];
      observations[output_i].y() = gnc_.ar_tag_msg_.cvs_observations[ar_max_features_ + i];
      output_i++;
    }
  }
  camera::CameraModel camera(Eigen::Vector3d(), Eigen::Matrix3d::Identity(), cam_params_);
  if (sparse_mapping::RansacEstimateCamera(landmarks, observations, 1000, 3,
      &camera, &inlier_landmarks, &inlier_observations))
    return false;
  Eigen::Affine3d global_pose = camera.GetTransform().inverse();
  Eigen::Quaterniond quat(global_pose.rotation());

  ros_tag_.header.seq++;
  ros_tag_.header.stamp.sec = gnc_.ar_tag_msg_.cvs_timestamp_sec;
  ros_tag_.header.stamp.nsec = gnc_.ar_tag_msg_.cvs_timestamp_nsec;
  ros_tag_.camera_id = ar_camera_count_;

  ros_tag_.pose.position = msg_conversions::eigen_to_ros_point(global_pose.translation());
  ros_tag_.pose.orientation = msg_conversions::eigen_to_ros_quat(quat);
  assert(inlier_landmarks.size() == inlier_observations.size());
  ros_tag_.landmarks.resize(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); i++) {
    ff_msgs::VisualLandmark l;
    l.x = landmarks[i].x();
    l.y = landmarks[i].y();
    l.z = landmarks[i].z();
    l.u = observations[i].x();
    l.v = observations[i].y();
    ros_tag_.landmarks[i] = l;
  }

  last_tag_time_.sec = gnc_.ar_tag_msg_.cvs_timestamp_sec;
  last_tag_time_.nsec = gnc_.ar_tag_msg_.cvs_timestamp_nsec;
  return true;
}

bool Sim::TagEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res) {
  en_tag_ = req.enable;
  res.success = true;
  return true;
}

bool Sim::PullOpticalPulseMsg() {
  if (!(!last_optical_pulse_ && gnc_.reg_pulse_.cvs_optical_flow_pulse > 0)) {
    // early exit condition, not a rising edge
    last_optical_pulse_ = gnc_.reg_pulse_.cvs_optical_flow_pulse > 0;
    return false;
  }

  ros_reg_pulse_.camera_id = ++of_camera_count_;

  last_optical_pulse_ = gnc_.reg_pulse_.cvs_optical_flow_pulse > 0;
  return true;
}

bool Sim::PullOpticalMsg() {
  if (last_optical_time_.sec == gnc_.optical_msg_.cvs_timestamp_sec &&
      last_optical_time_.nsec == gnc_.optical_msg_.cvs_timestamp_nsec) {
    return false;
  }

  ros_optical_.header.seq++;
  ros_optical_.header.stamp.sec = gnc_.optical_msg_.cvs_timestamp_sec;
  ros_optical_.header.stamp.nsec = gnc_.optical_msg_.cvs_timestamp_nsec;
  ros_optical_.camera_id = of_camera_count_;

  ros_optical_.feature_array.clear();

  // send message with all new features from this frame
  for (size_t i = 0; i < of_max_features_; i++) {
    int index = (of_history_size_ - 1) * of_max_features_;
    int valid = gnc_.optical_msg_.cvs_valid_flag[index + i];
    if (!valid)
      continue;
    float x = gnc_.optical_msg_.cvs_observations[index * 2 + i];
    float y = gnc_.optical_msg_.cvs_observations[index * 2 + i + of_max_features_];
    int id = gnc_.optical_msg_.cvs_id_tag[i];
    ff_msgs::Feature2d feature;
    feature.id = id;
    feature.x = x;
    feature.y = y;
    ros_optical_.feature_array.push_back(feature);
  }

  last_optical_time_.sec = gnc_.optical_msg_.cvs_timestamp_sec;
  last_optical_time_.nsec = gnc_.optical_msg_.cvs_timestamp_nsec;
  return true;
}

bool Sim::OpticalEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res) {
  en_optical_ = req.enable;
  res.success = true;
  return true;
}

bool Sim::PullDepthPulseMsg() {
  if (!(!last_depth_pulse_ && gnc_.reg_pulse_.cvs_handrail_pulse > 0)) {
    // early exit condition, not a rising edge
    last_depth_pulse_ = gnc_.reg_pulse_.cvs_handrail_pulse > 0;
    return false;
  }

  ros_reg_pulse_.camera_id = ++dl_camera_count_;

  last_depth_pulse_ = gnc_.reg_pulse_.cvs_handrail_pulse > 0;
  return true;
}

bool Sim::PullDepthMsg() {
  if (last_depth_time_.sec == gnc_.hand_msg_.cvs_timestamp_sec &&
      last_depth_time_.nsec == gnc_.hand_msg_.cvs_timestamp_nsec) {
    return false;
  }

  ros_depth_.header.seq++;
  ros_depth_.header.stamp.sec = gnc_.hand_msg_.cvs_timestamp_sec;
  ros_depth_.header.stamp.nsec = gnc_.hand_msg_.cvs_timestamp_nsec;
  ros_depth_.camera_id = dl_camera_count_;

  // Count how many landmarks there are
  size_t landmark_count = 0;
  for (auto flag : gnc_.hand_msg_.cvs_valid_flag) {
    if (flag > 0) {
      landmark_count++;
    }
  }
  ros_depth_.landmarks.resize(landmark_count);
  size_t output_i = 0;
  for (size_t i = 0; i < hr_max_features_; i++) {
    if (gnc_.hand_msg_.cvs_valid_flag[i] > 0) {
      /*
      ros_depth_.landmarks[output_i].x = gnc_.hand_msg_.cvs_landmarks[i];
      ros_depth_.landmarks[output_i].y = gnc_.hand_msg_.cvs_landmarks[hr_max_features_ + i];
      ros_depth_.landmarks[output_i].z = gnc_.hand_msg_.cvs_landmarks[2 * hr_max_features_ + i];
      */
      ros_depth_.landmarks[output_i].u = gnc_.hand_msg_.cvs_observations[i];
      ros_depth_.landmarks[output_i].v = gnc_.hand_msg_.cvs_observations[hr_max_features_ + i];
      ros_depth_.landmarks[output_i].w = gnc_.hand_msg_.cvs_observations[2 * hr_max_features_ + i];
      output_i++;
    }
  }
  ros_depth_.end_seen = gnc_.hand_msg_.cvs_3d_knowledge_flag;

  last_depth_time_.sec = gnc_.hand_msg_.cvs_timestamp_sec;
  last_depth_time_.nsec = gnc_.hand_msg_.cvs_timestamp_nsec;
  return true;
}

bool Sim::DepthEnable(ff_msgs::SetBool::Request &req, ff_msgs::SetBool::Response &res) {
  en_depth_ = req.enable;
  res.success = true;
  return true;
}

}  // end namespace sim_wrapper
