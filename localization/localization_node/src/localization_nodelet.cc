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

#include <localization_node/localization_nodelet.h>

#include <ff_common/init.h>
#include <sparse_mapping/sparse_map.h>

#include <ros/ros.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/ResetMap.h>
#include <ff_msgs/VisualLandmarks.h>
#include <geometry_msgs/TransformStamped.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <msg_conversions/msg_conversions.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/filesystem.hpp>

namespace localization_node {

LocalizationNodelet::LocalizationNodelet() : ff_util::FreeFlyerNodelet(NODE_MAPPED_LANDMARKS),
        enabled_(false), count_(0), processing_image_(true) {
  pthread_mutex_init(&mutex_features_, NULL);
  pthread_cond_init(&cond_features_, NULL);
}

LocalizationNodelet::~LocalizationNodelet(void) {
  thread_->join();
  pthread_mutex_destroy(&mutex_features_);
  pthread_cond_destroy(&cond_features_);
}

bool LocalizationNodelet::ResetMap(const std::string& map_file) {
  if (!boost::filesystem::exists(map_file)) {
    LOG(ERROR) << "Map file " << map_file << " does not exist, failed to reset map.";
    return false;
  }
  // Disable and wait for localization to finish running if it is running
  // before resetting the localizer
  enabled_ = false;
  while (processing_image_) {
    usleep(100000);
  }
  map_.reset(new sparse_mapping::SparseMap(map_file, true));
  inst_.reset(new Localizer(map_.get()));
  // Check to see if any params were changed when map was reset
  ReadParams();
  enabled_ = true;
  return true;
}

void LocalizationNodelet::Initialize(ros::NodeHandle* nh) {
  ff_common::InitFreeFlyerApplication(getMyArgv());

  config_.AddFile("cameras.config");
  config_.AddFile("localization.config");
  config_.ReadFiles();

  // Resolve the full path to the AR tag file specified for the current world
  std::string map_file;
  if (!config_.GetStr("world_vision_map_filename", &map_file))
    ROS_ERROR("Cannot read world_vision_map_filename from LUA config");

  // Reset all internal shared pointers
  it_.reset(new image_transport::ImageTransport(*nh));
  map_.reset(new sparse_mapping::SparseMap(map_file, true));
  inst_.reset(new Localizer(map_.get()));

  registration_publisher_ = nh->advertise<ff_msgs::CameraRegistration>(
      TOPIC_LOCALIZATION_ML_REGISTRATION, 10);
  landmark_publisher_     = nh->advertise<ff_msgs::VisualLandmarks>(
      TOPIC_LOCALIZATION_ML_FEATURES, 10);

  // Subscribe to input video feed and publish output odometry info
  image_sub_ = it_->subscribe(TOPIC_HARDWARE_NAV_CAM, 1, &LocalizationNodelet::ImageCallback, this);

  matched_features_on_ = false;
  all_features_on_ = false;
  config_.GetBool("matched_features_on", &matched_features_on_);
  config_.GetBool("all_features_on", &all_features_on_);

  if (matched_features_on_) {
    used_features_publisher_ = nh->advertise<sensor_msgs::Image>("rviz/used_features", 10);
  }
  if (all_features_on_) {
    detected_features_publisher_ = nh->advertise<sensor_msgs::Image>("rviz/detected_features", 10);
  }

  // start a new thread to run everything
  thread_.reset(new std::thread(&localization_node::LocalizationNodelet::Run, this));

  ReadParams();

  // only do this once, will cause a crash if done in middle of thread execution
  int num_threads;
  if (!config_.GetInt("num_threads", &num_threads))
    ROS_FATAL("num_threads not specified in localization.");
  cv::setNumThreads(num_threads);

  config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&LocalizationNodelet::ReadParams, this));}, false, true);

  enable_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_ML_ENABLE, &LocalizationNodelet::EnableService, this);
  reset_map_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_RESET_MAP, &LocalizationNodelet::ResetMapService, this);
}

void LocalizationNodelet::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }
  if (inst_) inst_->ReadParams(&config_);
}

bool LocalizationNodelet::EnableService(ff_msgs::SetBool::Request & req, ff_msgs::SetBool::Response & res) {
  enabled_ = req.enable;
  res.success = true;
  return true;
}

bool LocalizationNodelet::ResetMapService(ff_msgs::ResetMap::Request& req, ff_msgs::ResetMap::Response& res) {
  std::string map_file;
  if (req.map_file == "default") {
    if (!config_.GetStr("world_vision_map_filename", &map_file)) {
      ROS_ERROR("Cannot read world_vision_map_filename from LUA config");
      return false;
    }
  } else {
    map_file = req.map_file;
  }
  LOG(INFO) << "Resetting map to " << map_file;
  return ResetMap(map_file);
}

void LocalizationNodelet::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time timestamp = ros::Time::now();
  pthread_mutex_lock(&mutex_features_);
  bool cont = processing_image_;
  pthread_mutex_unlock(&mutex_features_);
  if (cont) return;

  ff_msgs::CameraRegistration r;
  r.header = std_msgs::Header();
  r.header.stamp = timestamp;
  r.camera_id = count_;
  registration_publisher_.publish(r);
  ros::spinOnce();

  try {
    image_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  pthread_mutex_lock(&mutex_features_);
  processing_image_ = true;
  pthread_cond_signal(&cond_features_);
  pthread_mutex_unlock(&mutex_features_);
}

void LocalizationNodelet::Localize(void) {
  ff_msgs::VisualLandmarks vl;
  Eigen::Matrix2Xd image_keypoints;

  bool success = inst_->Localize(image_ptr_, &vl, &image_keypoints);

  vl.camera_id = count_;
  if (enabled_) landmark_publisher_.publish(vl);
  ros::spinOnce();

  // only send transform if succeeded
  if (!success)
    return;

  // send rviz feature overlay messages
  sensor_msgs::ImagePtr image_pointer;
  if (matched_features_on_ || all_features_on_) {
    image_pointer = (*image_ptr_).toImageMsg();
  }
  if (matched_features_on_) {
    cv_bridge::CvImagePtr used_image = cv_bridge::toCvCopy(image_pointer);
    for (size_t i = 0; i < vl.landmarks.size(); i++) {
      Eigen::Vector2d undistorted, distorted;
      undistorted[0] = vl.landmarks[i].u;
      undistorted[1] = vl.landmarks[i].v;
      (map_->GetCameraParameters()).Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted, &distorted);
      cv::circle(used_image->image, cv::Point(distorted[0], distorted[1]), 10, CV_RGB(255, 255, 255), 3, 8);
      cv::circle(used_image->image, cv::Point(distorted[0], distorted[1]), 6, CV_RGB(0, 0, 0), 3, 8);
    }
    used_features_publisher_.publish(*used_image);
  }
  if (all_features_on_) {
    cv_bridge::CvImagePtr detected_image = cv_bridge::toCvCopy(image_pointer);
    for (int i = 0; i < image_keypoints.cols(); i++) {
      Eigen::Vector2d undistorted, distorted;
      undistorted[0] = image_keypoints.col(i)[0];
      undistorted[1] = image_keypoints.col(i)[1];
      (map_->GetCameraParameters()).Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted, &distorted);
      cv::circle(detected_image->image, cv::Point(distorted[0], distorted[1]), 10, CV_RGB(255, 255, 255), 3, 8);
      cv::circle(detected_image->image, cv::Point(distorted[0], distorted[1]), 6, CV_RGB(0, 0, 0), 2, 8);
    }
    detected_features_publisher_.publish(*detected_image);
  }

  // now publish the transform
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.seq = count_;
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "localization";
  transformStamped.transform.translation.x = vl.pose.position.x;
  transformStamped.transform.translation.y = vl.pose.position.y;
  transformStamped.transform.translation.z = vl.pose.position.z;
  transformStamped.transform.rotation = vl.pose.orientation;

  br.sendTransform(transformStamped);
}

void LocalizationNodelet::Run(void) {
  struct timespec ts;
  bool running = false;
  while (ros::ok()) {
    if (!enabled_) {
      image_sub_.shutdown();
      running = false;
    }
    if (!running) {
      if (enabled_) {
        image_sub_ = it_->subscribe(TOPIC_HARDWARE_NAV_CAM, 1, &LocalizationNodelet::ImageCallback, this);
        running = true;
      } else {
        usleep(100000);
        continue;
      }
    }
    pthread_mutex_lock(&mutex_features_);
    processing_image_ = false;  // initialize this here so we don't get images before thread starts
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;
    pthread_cond_timedwait(&cond_features_, &mutex_features_, &ts);
    bool ready = processing_image_;
    pthread_mutex_unlock(&mutex_features_);
    if (!ready)
      continue;
    // unlock the mutex for localizing so we can ignore images we get during this
    Localize();
    count_++;
    pthread_mutex_lock(&mutex_features_);
    processing_image_ = false;
    pthread_mutex_unlock(&mutex_features_);
  }
}

};  // namespace localization_node

PLUGINLIB_EXPORT_CLASS(localization_node::LocalizationNodelet, nodelet::Nodelet)
