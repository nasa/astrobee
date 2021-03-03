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

#include <ekf_bag/ekf_bag.h>

#include <ff_util/ff_names.h>
#include <image_transport/image_transport.h>

#include <camera/camera_params.h>
#include <ff_common/utils.h>
#include <rosbag/view.h>
#include <Eigen/Core>

namespace ekf_bag {

EkfBag::EkfBag(const char* bagfile, const char* mapfile, bool run_ekf, bool gen_features, const char* biasfile,
               std::string image_topic, const std::string& gnc_config)
    : map_(mapfile, true),
      loc_(&map_),
      run_ekf_(run_ekf),
      gen_features_(gen_features),
      bias_file_(biasfile),
      image_topic_(image_topic),
      gnc_config_(gnc_config) {
  bag_.open(bagfile, rosbag::bagmode::Read);
  // Don't gen features if they are already in the bag
  rosbag::View view(bag_);
  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
  gen_features_ = true;
  for (const auto info : connection_infos) {
    std::string of_features = "/" + std::string(TOPIC_LOCALIZATION_OF_FEATURES);
    if (info->topic == of_features) {
      gen_features_ = false;
      break;
    }
  }
}

EkfBag::~EkfBag(void) { bag_.close(); }

void EkfBag::ReadParams(config_reader::ConfigReader* config) {
  config->AddFile("tools/ekf_bag.config");
  config->AddFile(gnc_config_.c_str());
  config->AddFile("cameras.config");
  config->AddFile("geometry.config");
  config->AddFile("localization.config");
  config->AddFile("optical_flow.config");

  if (!config->ReadFiles()) {
    ROS_FATAL("Failed to read config files.");
    return;
  }

  if (!config->GetReal("sparse_map_delay", &sparse_map_delay_)) ROS_FATAL("sparse_map_delay not specified.");
  if (!config->GetReal("of_delay", &of_delay_)) ROS_FATAL("of_delay not specified.");

  ekf_.ReadParams(config);
  of_.ReadParams(config);
  loc_.ReadParams(config);
}

void EkfBag::EstimateBias(void) {
  Eigen::Vector3f gyro(0, 0, 0), accel(0, 0, 0);

  if (bias_file_ != NULL) {
    printf("Using bias file: %s\n", bias_file_);
    FILE* f = fopen(bias_file_, "r");
    if (f == NULL) {
      fprintf(stderr, "Failed to open bias file %s.\n", bias_file_);
      exit(1);
    }
    int ret = fscanf(f, "%g %g %g\n", &gyro[0], &gyro[1], &gyro[2]);
    if (ret != 3) {
      fprintf(stderr, "Failed to read bias.\n");
      exit(1);
    }
    ret = fscanf(f, "%g %g %g\n", &accel[0], &accel[1], &accel[2]);
    if (ret != 3) {
      fprintf(stderr, "Failed to read bias.\n");
      exit(1);
    }
    fclose(f);
  } else {
    std::vector<std::string> topics;
    topics.push_back(std::string("/") + TOPIC_HARDWARE_IMU);
    topics.push_back(TOPIC_HARDWARE_IMU);

    // get the start time
    rosbag::View temp(bag_, rosbag::TopicQuery(topics));
    ros::Time start = temp.getBeginTime();

    // look in only the first few seconds
    rosbag::View early_imu(bag_, rosbag::TopicQuery(topics), start, start + ros::Duration(5.0));

    int count = 0;
    for (rosbag::MessageInstance const m : early_imu) {
      if (m.isType<sensor_msgs::Imu>()) {
        sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        accel.x() += imu->linear_acceleration.x;
        accel.y() += imu->linear_acceleration.y;
        accel.z() += imu->linear_acceleration.z;
        gyro.x() += imu->angular_velocity.x;
        gyro.y() += imu->angular_velocity.y;
        gyro.z() += imu->angular_velocity.z;
        count++;
      }
    }
    accel = accel / count;
    gyro = gyro / count;
    printf("Using Bias:\n");
    printf("%g %g %g\n", gyro[0], gyro[1], gyro[2]);
    printf("%g %g %g\n", accel[0], accel[1], accel[2]);
  }
  ekf_.SetBias(gyro, accel);
  ekf_.Reset();
}

void EkfBag::UpdateImu(const ros::Time& time, const sensor_msgs::Imu& imu) {
  if (!run_ekf_) return;
  // send the visual messages when it's time
  if (processing_of_ && time >= of_send_time_) {
    processing_of_ = false;
    UpdateOpticalFlow(of_features_);
  }
  if (processing_sparse_map_ && time >= vl_send_time_) {
    processing_sparse_map_ = false;
    UpdateSparseMap(vl_features_);
  }

  // Pass a quaternion in to do MGTF gravity correction if needed
  ekf_.PrepareStep(imu, ground_truth_.orientation);
  ff_msgs::EkfState state;
  int ret = ekf_.Step(&state);
  if (ret) UpdateEKF(state);
}

void EkfBag::UpdateImage(const ros::Time& time, const sensor_msgs::ImageConstPtr& image_msg) {
  if (!run_ekf_) return;
  if (!gen_features_) return;
  if (!processing_of_) {
    // send of registration
    ff_msgs::CameraRegistration r;
    r.header = std_msgs::Header();
    r.header.stamp = time;
    r.camera_id = ++of_id_;
    UpdateOpticalFlowReg(r);
    // do the processing now, but send it later
    of_features_.feature_array.clear();
    of_.OpticalFlow(image_msg, &of_features_);
    of_features_.camera_id = of_id_;
    processing_of_ = true;
    of_send_time_ = time + ros::Duration(of_delay_);
  }
  // now do sparse map
  if (!processing_sparse_map_) {
    // send registration
    ff_msgs::CameraRegistration r;
    r.header = std_msgs::Header();
    r.header.stamp = time;
    r.camera_id = ++vl_id_;
    UpdateSparseMapReg(r);
    // do processing now, but send it later
    cv_bridge::CvImageConstPtr image;
    try {
      image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    vl_features_.landmarks.clear();
    loc_.Localize(image, &vl_features_);
    vl_features_.camera_id = vl_id_;
    processing_sparse_map_ = true;
    vl_send_time_ = time + ros::Duration(sparse_map_delay_);
  }
}

void EkfBag::UpdateGroundTruth(const geometry_msgs::PoseStamped& pose) {
  ground_truth_ = pose.pose;  // Cache the pose for MGTF gravity correction
}

void EkfBag::UpdateOpticalFlow(const ff_msgs::Feature2dArray& of) {
  if (run_ekf_) ekf_.OpticalFlowUpdate(of);
}

void EkfBag::UpdateSparseMap(const ff_msgs::VisualLandmarks& vl) {
  if (run_ekf_) ekf_.SparseMapUpdate(vl);
}

void EkfBag::UpdateOpticalFlowReg(const ff_msgs::CameraRegistration& reg) {
  if (run_ekf_) ekf_.OpticalFlowRegister(reg);
}

void EkfBag::UpdateSparseMapReg(const ff_msgs::CameraRegistration& reg) {
  if (run_ekf_) ekf_.SparseMapRegister(reg);
}

bool string_ends_with(const std::string& str, const std::string& ending) {
  if (str.length() >= ending.length()) {
    return (0 == str.compare(str.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

void EkfBag::Run(void) {
  if (run_ekf_) EstimateBias();

  std::vector<std::string> topics;
  topics.push_back(std::string("/") + TOPIC_HARDWARE_IMU);
  topics.push_back(TOPIC_HARDWARE_IMU);
  topics.push_back(std::string("/") + TOPIC_LOCALIZATION_TRUTH);
  topics.push_back(TOPIC_LOCALIZATION_TRUTH);
  topics.push_back(std::string("/") + image_topic_);
  topics.push_back(image_topic_);
  if (!gen_features_) {
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_ML_REGISTRATION);
    topics.push_back(TOPIC_LOCALIZATION_ML_REGISTRATION);
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_ML_FEATURES);
    topics.push_back(TOPIC_LOCALIZATION_ML_FEATURES);
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_AR_REGISTRATION);
    topics.push_back(TOPIC_LOCALIZATION_AR_REGISTRATION);
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_AR_FEATURES);
    topics.push_back(TOPIC_LOCALIZATION_AR_FEATURES);
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_OF_REGISTRATION);
    topics.push_back(TOPIC_LOCALIZATION_OF_REGISTRATION);
    topics.push_back(std::string("/") + TOPIC_LOCALIZATION_OF_FEATURES);
    topics.push_back(TOPIC_LOCALIZATION_OF_FEATURES);
  }
  if (!run_ekf_) {
    topics.push_back(std::string("/") + TOPIC_GNC_EKF);
    topics.push_back(TOPIC_GNC_EKF);
  }

  rosbag::View view(bag_, rosbag::TopicQuery(topics));
  bag_start_time_ = view.getBeginTime();

  processing_of_ = processing_sparse_map_ = false;
  of_id_ = vl_id_ = 0;

  int progress = 0;
  for (rosbag::MessageInstance const m : view) {
    progress++;

    if (string_ends_with(m.getTopic(), image_topic_)) {
      sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      UpdateImage(m.getTime(), image_msg);
      // TODO(rsoussan): making print this optional
      /*ff_common::PrintProgressBar(stdout,
                               static_cast<float>(progress) / view.size());*/
    } else if (string_ends_with(m.getTopic(), TOPIC_HARDWARE_IMU)) {
      sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      UpdateImu(m.getTime(), *imu_msg.get());
    } else if (string_ends_with(m.getTopic(), TOPIC_LOCALIZATION_TRUTH)) {
      geometry_msgs::PoseStampedConstPtr gt_msg = m.instantiate<geometry_msgs::PoseStamped>();
      UpdateGroundTruth(*gt_msg.get());
    } else if (string_ends_with(m.getTopic(), TOPIC_LOCALIZATION_ML_FEATURES)) {
      ff_msgs::VisualLandmarksConstPtr vl_msg = m.instantiate<ff_msgs::VisualLandmarks>();
      UpdateSparseMap(*vl_msg.get());
    } else if (string_ends_with(m.getTopic(), TOPIC_LOCALIZATION_ML_REGISTRATION)) {
      ff_msgs::CameraRegistrationConstPtr reg_msg = m.instantiate<ff_msgs::CameraRegistration>();
      UpdateSparseMapReg(*reg_msg.get());
    } else if (string_ends_with(m.getTopic(), TOPIC_LOCALIZATION_OF_FEATURES)) {
      ff_msgs::Feature2dArrayConstPtr of_msg = m.instantiate<ff_msgs::Feature2dArray>();
      UpdateOpticalFlow(*of_msg.get());
    } else if (string_ends_with(m.getTopic(), TOPIC_LOCALIZATION_OF_REGISTRATION)) {
      ff_msgs::CameraRegistrationConstPtr reg_msg = m.instantiate<ff_msgs::CameraRegistration>();
      UpdateOpticalFlowReg(*reg_msg.get());
    } else if (string_ends_with(m.getTopic(), TOPIC_GNC_EKF)) {
      ff_msgs::EkfStateConstPtr ekf_msg = m.instantiate<ff_msgs::EkfState>();
      UpdateEKF(*ekf_msg.get());
    }
  }
  printf("\n");
}

}  // namespace ekf_bag
