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

#include <ekf/ekf_wrapper.h>
#include <camera/camera_params.h>
#include <camera/camera_model.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <msg_conversions/msg_conversions.h>
#include <ros/package.h>

#include <ff_util/ff_names.h>

namespace ekf {

EkfWrapper::EkfWrapper(ros::NodeHandle* nh, std::string const& platform_name) :
          ekf_initialized_(false), imus_dropped_(0), have_imu_(false),
          input_mode_(ff_msgs::SetEkfInputRequest::MODE_NONE), nh_(nh),
          estimating_bias_(false), disp_features_(false), killed_(false) {
  platform_name_ = (platform_name.empty() ? "" : platform_name + "/");

  config_.AddFile("gnc.config");
  config_.AddFile("cameras.config");
  config_.AddFile("geometry.config");
  ReadParams();
  config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&EkfWrapper::ReadParams, this));}, false, true);
  pt_ekf_.Initialize("ekf");

  // Register to receive a callback when the EKF resets
  ekf_.SetResetCallback(std::bind(&EkfWrapper::ResetCallback, this));
  // subscribe to IMU first, then rest once IMU is ready
  // this is so localization manager doesn't timeout
  imu_sub_    = nh_->subscribe(TOPIC_HARDWARE_IMU, 5, &EkfWrapper::ImuCallBack, this,
                              ros::TransportHints().tcpNoDelay());
}

EkfWrapper::~EkfWrapper() {
  killed_ = true;
}

// wait to start up until the IMU is ready
void EkfWrapper::InitializeEkf(void) {
  state_pub_   = nh_->advertise<ff_msgs::EkfState>(TOPIC_GNC_EKF, 1);
  pose_pub_    = nh_->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_POSE, 1);
  twist_pub_   = nh_->advertise<geometry_msgs::TwistStamped>(TOPIC_LOCALIZATION_TWIST, 1);
  feature_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(TOPIC_GNC_EKF_FEATURES, 1,
      boost::bind(&EkfWrapper::SubscriberCallback, this),
      boost::bind(&EkfWrapper::SubscriberCallback, this));
  reset_pub_   = nh_->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 1);

  vl_sub_     = nh_->subscribe(TOPIC_LOCALIZATION_ML_FEATURES, 1, &EkfWrapper::VLVisualLandmarksCallBack, this,
                              ros::TransportHints().tcpNoDelay());
  vl_reg_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_ML_REGISTRATION, 1, &EkfWrapper::VLRegisterCamera, this,
                              ros::TransportHints().tcpNoDelay());
  ar_sub_     = nh_->subscribe(TOPIC_LOCALIZATION_AR_FEATURES, 1, &EkfWrapper::ARVisualLandmarksCallBack, this,
                              ros::TransportHints().tcpNoDelay());
  ar_reg_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_AR_REGISTRATION, 1, &EkfWrapper::ARRegisterCamera, this,
                              ros::TransportHints().tcpNoDelay());
  of_sub_     = nh_->subscribe(TOPIC_LOCALIZATION_OF_FEATURES, 1, &EkfWrapper::OpticalFlowCallBack, this,
                              ros::TransportHints().tcpNoDelay());
  of_reg_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_OF_REGISTRATION, 1, &EkfWrapper::RegisterOpticalFlowCamera, this,
                              ros::TransportHints().tcpNoDelay());
  dl_sub_     = nh_->subscribe(TOPIC_LOCALIZATION_HR_FEATURES, 1, &EkfWrapper::DepthLandmarksCallBack, this,
                              ros::TransportHints().tcpNoDelay());
  dl_reg_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_HR_REGISTRATION, 1, &EkfWrapper::RegisterDepthCamera, this,
                              ros::TransportHints().tcpNoDelay());
  truth_sub_  = nh_->subscribe(TOPIC_LOCALIZATION_TRUTH, 1, &EkfWrapper::GroundTruthCallback, this,
                              ros::TransportHints().tcpNoDelay());
  truth_twist_sub_  = nh_->subscribe(TOPIC_LOCALIZATION_TRUTH_TWIST, 1, &EkfWrapper::GroundTruthTwistCallback, this,
                              ros::TransportHints().tcpNoDelay());
  flight_mode_sub_ = nh_->subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 1, &EkfWrapper::FlightModeCallback, this);

  reset_srv_      = nh_->advertiseService(SERVICE_GNC_EKF_RESET, &EkfWrapper::ResetService, this);
  bias_srv_       = nh_->advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &EkfWrapper::InitializeBiasService, this);
  input_mode_srv_ = nh_->advertiseService(SERVICE_GNC_EKF_SET_INPUT, &EkfWrapper::SetInputService, this);

  reset_hr_srv_ = nh_->advertiseService(SERVICE_GNC_EKF_RESET_HR, &EkfWrapper::ResetHandrailService, this);

  ekf_initialized_ = true;
}

void EkfWrapper::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }

  if (!config_.GetInt("bias_required_observations", &bias_required_observations_))
    ROS_FATAL("Unspecified bias_required_observations.");

  if (!config_.GetStr("imu_bias_file", &bias_file_)) {
    ROS_FATAL("IMU bias file not specified.");
  }

  ekf_.ReadParams(&config_);
}

void EkfWrapper::SubscriberCallback() {
  disp_features_ = (feature_pub_.getNumSubscribers() > 0);
}

void EkfWrapper::PublishFeatures(ff_msgs::VisualLandmarks::ConstPtr const& l) {
  if (!disp_features_) return;
  features_.header = l->header;
  features_.height = 1;
  features_.width = l->landmarks.size();
  features_.fields.resize(3);
  features_.fields[0].name = "x";
  features_.fields[0].offset = 0;
  features_.fields[0].datatype = 7;
  features_.fields[0].count = 1;
  features_.fields[1].name = "y";
  features_.fields[1].offset = 4;
  features_.fields[1].datatype = 7;
  features_.fields[1].count = 1;
  features_.fields[2].name = "z";
  features_.fields[2].offset = 8;
  features_.fields[2].datatype = 7;
  features_.fields[2].count = 1;
  features_.is_bigendian = false;
  features_.point_step = 12;
  features_.row_step = features_.point_step * features_.width;
  features_.is_dense = true;
  features_.data.resize(features_.row_step);
  for (unsigned int i = 0; i < l->landmarks.size(); i++) {
    memcpy(&features_.data[features_.point_step * i + 0], &l->landmarks[i].x, 4);
    memcpy(&features_.data[features_.point_step * i + 4], &l->landmarks[i].y, 4);
    memcpy(&features_.data[features_.point_step * i + 8], &l->landmarks[i].z, 4);
  }
  feature_pub_.publish(features_);
}

void EkfWrapper::PublishFeatures(ff_msgs::DepthLandmarks::ConstPtr const& l) {
  if (!disp_features_) return;
  features_.header = std_msgs::Header();
  features_.header.stamp = ros::Time::now();
  features_.header.frame_id = platform_name_ + "perch_cam";
  features_.height = 1;
  features_.width = l->landmarks.size();
  features_.fields.resize(3);
  features_.fields[0].name = "x";
  features_.fields[0].offset = 0;
  features_.fields[0].datatype = 7;
  features_.fields[0].count = 1;
  features_.fields[1].name = "y";
  features_.fields[1].offset = 4;
  features_.fields[1].datatype = 7;
  features_.fields[1].count = 1;
  features_.fields[2].name = "z";
  features_.fields[2].offset = 8;
  features_.fields[2].datatype = 7;
  features_.fields[2].count = 1;
  features_.is_bigendian = false;
  features_.point_step = 12;
  features_.row_step = features_.point_step * features_.width;
  features_.is_dense = true;
  features_.data.resize(features_.row_step);
  // The features are expressed as image plane coordinated (u,v) and a depth. We
  // must use the camera model to convert this to an (x,y,z) coordinate.
  static camera::CameraParameters cam_params(&config_, "perch_cam");
  static camera::CameraModel camera_model(Eigen::Vector3d(0, 0, 0),
    Eigen::Matrix3d::Identity(), cam_params);
  float x, y, z;
  for (unsigned int i = 0; i < l->landmarks.size(); i++) {
    Eigen::Vector3d p_c = l->landmarks[i].w * camera_model.Ray(
      static_cast<int>(l->landmarks[i].u), static_cast<int>(l->landmarks[i].v));
    x = static_cast<float>(p_c[0]);
    y = static_cast<float>(p_c[1]);
    z = static_cast<float>(p_c[2]);
    memcpy(&features_.data[features_.point_step * i + 0], &x, 4);
    memcpy(&features_.data[features_.point_step * i + 4], &y, 4);
    memcpy(&features_.data[features_.point_step * i + 8], &z, 4);
  }
  feature_pub_.publish(features_);
}

void EkfWrapper::ResetCallback() {
  static std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void EkfWrapper::ImuCallBack(sensor_msgs::Imu::ConstPtr const& imu) {
  // concurrency protection
  std::unique_lock<std::mutex> lock(mutex_imu_msg_);
  while (have_imu_ && !killed_)
    cv_imu_.wait_for(lock, std::chrono::milliseconds(8));

  // copy IMU data
  imu_ = *imu;

  have_imu_ = true;
  // now notify the condition variable waiting for IMU that we have it
  lock.unlock();
  cv_imu_.notify_all();

  if (estimating_bias_)
    EstimateBias(imu);
}

void EkfWrapper::EstimateBias(sensor_msgs::Imu::ConstPtr const& imu) {
  bias_reset_count_++;
  bias_reset_sums_[0] += imu->angular_velocity.x;
  bias_reset_sums_[1] += imu->angular_velocity.y;
  bias_reset_sums_[2] += imu->angular_velocity.z;
  bias_reset_sums_[3] += imu->linear_acceleration.x;
  bias_reset_sums_[4] += imu->linear_acceleration.y;
  bias_reset_sums_[5] += imu->linear_acceleration.z;
  if (bias_reset_count_ >= bias_required_observations_) {
    for (int i = 0; i < 6; i++)
      bias_reset_sums_[i] = bias_reset_sums_[i] / bias_reset_count_;

    ROS_INFO("Esimated biases: gyro: %g %g %g accel: %g %g %g",
             bias_reset_sums_[0], bias_reset_sums_[1], bias_reset_sums_[2],
             bias_reset_sums_[3], bias_reset_sums_[4], bias_reset_sums_[5]);
    FILE* f = fopen(bias_file_.c_str(), "w");
    if (f) {
      fprintf(f, "%g %g %g\n", bias_reset_sums_[0], bias_reset_sums_[1], bias_reset_sums_[2]);
      fprintf(f, "%g %g %g\n", bias_reset_sums_[3], bias_reset_sums_[4], bias_reset_sums_[5]);
      fclose(f);
    } else {
      ROS_ERROR("Bias file %s could not be opened.", bias_file_.c_str());
    }
    ekf_.SetBias(Eigen::Vector3f(bias_reset_sums_[0], bias_reset_sums_[1], bias_reset_sums_[2]),
                 Eigen::Vector3f(bias_reset_sums_[3], bias_reset_sums_[4], bias_reset_sums_[5]));

    estimating_bias_ = false;
    ekf_.Reset();
  }
}

void EkfWrapper::OpticalFlowCallBack(ff_msgs::Feature2dArray::ConstPtr const& of) {
  std::lock_guard<std::mutex> lock(mutex_of_msg_);
  ekf_.OpticalFlowUpdate(*of.get());
}

void EkfWrapper::VLVisualLandmarksCallBack(ff_msgs::VisualLandmarks::ConstPtr const& vl) {
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_MAP_LANDMARKS) {
    std::lock_guard<std::mutex> lock(mutex_vl_msg_);
    ekf_.SparseMapUpdate(*vl.get());
    PublishFeatures(vl);
  }
}

void EkfWrapper::ARVisualLandmarksCallBack(ff_msgs::VisualLandmarks::ConstPtr const& vl) {
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_AR_TAGS) {
    std::lock_guard<std::mutex> lock(mutex_vl_msg_);
    bool updated = ekf_.ARTagUpdate(*vl.get());
    if (updated) {
      Eigen::Affine3d t = ekf_.GetDockToWorldTransform();
      geometry_msgs::TransformStamped transform;
      transform.header = std_msgs::Header();
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = "world";
      transform.child_frame_id = "dock/body";
      transform.transform.translation = msg_conversions::eigen_to_ros_vector(t.translation());
      transform.transform.rotation = msg_conversions::eigen_to_ros_quat(Eigen::Quaterniond(t.linear()));
      transform_pub_.sendTransform(transform);
    }
    PublishFeatures(vl);
  }
}

void EkfWrapper::DepthLandmarksCallBack(ff_msgs::DepthLandmarks::ConstPtr const& dl) {
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_HANDRAIL) {
    std::lock_guard<std::mutex> lock(mutex_dl_msg_);
    bool updated = ekf_.HRTagUpdate(*dl.get());
    if (updated) {
      Eigen::Affine3d t = ekf_.GetHandrailToWorldTransform();
      geometry_msgs::TransformStamped transform;
      transform.header = std_msgs::Header();
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = "world";
      transform.child_frame_id = "handrail/body";
      transform.transform.translation = msg_conversions::eigen_to_ros_vector(t.translation());
      transform.transform.rotation = msg_conversions::eigen_to_ros_quat(Eigen::Quaterniond(t.linear()));
      transform_pub_.sendTransform(transform);
    }
    PublishFeatures(dl);
  }
}

void EkfWrapper::RegisterOpticalFlowCamera(ff_msgs::CameraRegistration::ConstPtr const& cr) {
  std::lock_guard<std::mutex> lock(mutex_of_msg_);
  ekf_.OpticalFlowRegister(*cr.get());
}

void EkfWrapper::VLRegisterCamera(ff_msgs::CameraRegistration::ConstPtr const& reg) {
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_MAP_LANDMARKS) {
    std::lock_guard<std::mutex> lock(mutex_vl_msg_);
    ekf_.SparseMapRegister(*reg.get());
  }
}

void EkfWrapper::ARRegisterCamera(ff_msgs::CameraRegistration::ConstPtr const& reg) {
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_AR_TAGS) {
    std::lock_guard<std::mutex> lock(mutex_vl_msg_);
    ekf_.ARTagRegister(*reg.get());
  }
}

void EkfWrapper::RegisterDepthCamera(ff_msgs::CameraRegistration::ConstPtr const& reg) {
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_HANDRAIL) {
    std::lock_guard<std::mutex> lock(mutex_vl_msg_);
    ekf_.HandrailRegister(*reg.get());
  }
}

void EkfWrapper::GroundTruthCallback(geometry_msgs::PoseStamped::ConstPtr const& pose) {
  // For certain contexts (like MGTF) we want to extract the correct orientation, and pass it to
  // GNC, so that Earth's gravity can be extracted out of the linear acceleration.
  std::lock_guard<std::mutex> lk(mutex_truth_msg_);
  assert(pose->header.frame_id == "world");
  quat_ = pose->pose.orientation;
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_TRUTH) {
    truth_pose_= *pose;
    pose_pub_.publish(pose);
  }
}

void EkfWrapper::GroundTruthTwistCallback(geometry_msgs::TwistStamped::ConstPtr const& twist) {
  std::lock_guard<std::mutex> lk(mutex_truth_msg_);
  assert(twist->header.frame_id == "world");
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_TRUTH) {
    truth_twist_= *twist;
    twist_pub_.publish(twist);
  }
}

void EkfWrapper::FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode) {
  ekf_.SetSpeedGain(mode->speed);
}

void EkfWrapper::Run(std::atomic<bool> const& killed) {
  // Kill the step thread
  while (!killed) {
    ros::spinOnce();
    Step();
  }
  // Make sure the IMU thread also gets killed
  killed_ = true;
}

int EkfWrapper::Step() {
  // don't modify anything while we're copying the data
  {
    // wait until we get an imu reading with the condition variable
    std::unique_lock<std::mutex> lk(mutex_imu_msg_);
    if (!have_imu_)
      cv_imu_.wait_for(lk, std::chrono::milliseconds(8));
    if (!have_imu_) {
      imus_dropped_++;
      // publish a failure if we stop getting imu messages
      if (imus_dropped_ > 10 && ekf_initialized_) {
        state_.header.stamp = ros::Time::now();
        state_.confidence = 2;  // lost
        state_pub_.publish<ff_msgs::EkfState>(state_);
        ekf_.Reset();
      }
      return 0;   // Changed by Andrew due to 250Hz ctl messages when sim blocks (!)
    }
    imus_dropped_ = 0;
    if (!ekf_initialized_)
      InitializeEkf();

    std::lock(mutex_act_msg_, mutex_vl_msg_, mutex_dl_msg_, mutex_of_msg_, mutex_truth_msg_);
    std::lock_guard<std::mutex> lock_actMsg(mutex_act_msg_, std::adopt_lock);
    std::lock_guard<std::mutex> lock_vlMsg(mutex_vl_msg_, std::adopt_lock);
    std::lock_guard<std::mutex> lock_dlMsg(mutex_dl_msg_, std::adopt_lock);
    std::lock_guard<std::mutex> lock_ofMsg(mutex_of_msg_, std::adopt_lock);
    std::lock_guard<std::mutex> lock_truthMsg(mutex_truth_msg_, std::adopt_lock);
    // copy everything in EKF, so data structures can be modified for next
    // step while current step processes. We pass the ground truth quaternion
    // representing the latest ISS2BODY rotation, which is used in certain
    // testing contexts to remove the effect of Earth's gravity.
    ekf_.PrepareStep(imu_, quat_);
    // don't reuse imu reading
    have_imu_ = false;
  }
  cv_imu_.notify_all();

  int ret = 1;
  switch (input_mode_) {
  // Don't do anything when localization in turned off
  case ff_msgs::SetEkfInputRequest::MODE_NONE:
    state_.ml_count = 0;  // these could be set from before
    state_.of_count = 0;
    break;
  // In truth mode we don't step the filter forward, but we do copy the pose
  // and twist into the EKF message.
  case ff_msgs::SetEkfInputRequest::MODE_TRUTH: {
      std::lock_guard<std::mutex> lk(mutex_truth_msg_);
      ros::Time t = ros::Time::now();
      if (fabs((truth_pose_.header.stamp - t).toSec()) < 1 &&
          fabs((truth_twist_.header.stamp - t).toSec()) < 1) {
        state_.header.stamp = t;
        state_.pose = truth_pose_.pose;
        state_.velocity = truth_twist_.twist.linear;
        state_.omega = truth_twist_.twist.angular;
        break;
      }
      ret = 0;
      break;
    }
  // All other pipelines get stepped forward normally
  default:
    pt_ekf_.Tick();
    ret = ekf_.Step(&state_);
    pt_ekf_.Tock();
    break;
  }
  // Let other elements in the system know that the bias is being estimated
  state_.estimating_bias = estimating_bias_;
  // Only send the state if the Step() function was successful
  if (ret)
    PublishState(state_);
  return ret;
}

void EkfWrapper::PublishState(const ff_msgs::EkfState & state) {
  // Publish the full EKF state
  state_pub_.publish<ff_msgs::EkfState>(state);
  // Only publish a transform if the confidence is good enough and we have
  // actually populated the state (examine the header to check)
  if (state.confidence == 0 && !state.header.frame_id.empty()) {
    geometry_msgs::TransformStamped transform;
    transform.header = state.header;
    transform.child_frame_id = platform_name_ + "body";
    transform.transform.translation.x = state.pose.position.x;
    transform.transform.translation.y = state.pose.position.y;
    transform.transform.translation.z = state.pose.position.z;
    transform.transform.rotation = state.pose.orientation;
    transform_pub_.sendTransform(transform);
  }
  // Publish a truthful
  if (input_mode_ != ff_msgs::SetEkfInputRequest::MODE_TRUTH) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = state.header.stamp;
    pose.pose = state.pose;
    pose_pub_.publish(pose);
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = state.header.stamp;
    twist.twist.linear = state.velocity;
    twist.twist.angular = state.omega;
    twist_pub_.publish(twist);
  }
}

bool EkfWrapper::ResetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {  // NOLINT
  ekf_.Reset();
  return true;
}

// Service to reset the handrail position which is only calculated once. Unused, but
// should be used in the future.
bool EkfWrapper::ResetHandrailService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ekf_.ResetHR();
  return true;
}

bool EkfWrapper::InitializeBiasService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {  // NOLINT
  bias_reset_count_ = 0;
  for (int i = 0; i < 6; i++)
    bias_reset_sums_[i] = 0.0;
  estimating_bias_ = true;
  ROS_INFO("Beginning bias estimation.");
  return true;
}

bool EkfWrapper::SetInputService(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {  //NOLINT
  input_mode_ = req.mode;
  switch (input_mode_) {
  case ff_msgs::SetEkfInputRequest::MODE_AR_TAGS:
    ekf_.ResetAR();
    ROS_INFO("EKF input switched to AR tags.");
    break;
  case ff_msgs::SetEkfInputRequest::MODE_HANDRAIL:
    ekf_.ResetHR();
    ROS_INFO("EKF input switched to handrail.");
    break;
  case ff_msgs::SetEkfInputRequest::MODE_MAP_LANDMARKS:
    ROS_INFO("EKF input switched to mapped landmarks.");
    break;
  case ff_msgs::SetEkfInputRequest::MODE_NONE:
    ROS_INFO("EKF input switched to none.");
    break;
  case ff_msgs::SetEkfInputRequest::MODE_TRUTH:
    ROS_INFO("EKF input switched to ground truth.");
    break;
  default:
    break;
  }
  return true;
}

}  // end namespace ekf
