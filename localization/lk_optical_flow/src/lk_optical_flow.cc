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

#include <lk_optical_flow/lk_optical_flow.h>
#include <ff_msgs/CameraRegistration.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>

#define CV_FONT_NORMAL CV_RAND_NORMAL

namespace lk_optical_flow {

LKOpticalFlow::LKOpticalFlow(void) :
  id_cnt_(0), camera_param_(Eigen::Vector2i::Zero(),
      Eigen::Vector2d::Ones(),
      Eigen::Vector2d::Zero()) {
}


void LKOpticalFlow::ReadParams(config_reader::ConfigReader* config) {
  int max_feature;
  if (!config->GetReal("max_flow_magnitude", &max_flow_magnitude_))
    ROS_FATAL("Unspecified max_flow_magnitude.");
  if (!config->GetInt("max_lk_pyr_level", &max_lk_pyr_level_))
    ROS_FATAL("Unspecified max_lk_pyr_level.");
  if (!config->GetInt("max_lk_itr", &max_lk_itr_))
    ROS_FATAL("Unspecified max_lk_itr.");
  if (!config->GetInt("max_gap", &max_gap_))
    ROS_FATAL("Unspecified max_gap.");
  if (!config->GetInt("win_size_width", &win_size_.width))
    ROS_FATAL("Unspecified win_size_width.");
  if (!config->GetInt("win_size_height", &win_size_.height))
    ROS_FATAL("Unspecified win_size_height.");
  if (!config->GetInt("max_feature", &max_feature))
    ROS_FATAL("Unspecified max_feature.");
  scale_factor_ = 2.0;
  max_feature_ = static_cast<size_t>(max_feature);

  scale_factor_ = 2.0;
  id_max_ = max_feature_ * 10000;
  ignored_last_frame_ = true;
  camera_param_ = camera::CameraParameters(config, "nav_cam");
}

void LKOpticalFlow::OpticalFlow(const sensor_msgs::ImageConstPtr& msg,
                                  ff_msgs::Feature2dArray* features) {
  // Convert the ros image message type into cv::Mat
  try {
    image_curr_ = cv_bridge::toCvShare(msg, msg->encoding)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::resize(image_curr_, image_curr_, cv::Size(), 1.0 / scale_factor_, 1.0 / scale_factor_);
  std::vector<cv::Point2f> new_corners;
  GetNewFeatures(&new_corners);
  if (!curr_corners_.empty()) {
    // Run LK optical flow algorithm for consecutive image frames
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, max_lk_itr_, 0.03);
    cv::calcOpticalFlowPyrLK(image_prev_, image_curr_, prev_corners_, curr_corners_, status_, err_,
                             win_size_, max_lk_pyr_level_, termcrit,
                             0, 0.001);
    cv::calcOpticalFlowPyrLK(image_curr_, image_prev_, curr_corners_, backwards_corners_,
                             backwards_status_, backwards_err_, win_size_, max_lk_pyr_level_, termcrit,
                             0, 0.001);

    // Remove corners with false status and with large displacements
    RefineCorners();

    // Add new corners from new_corners to maintain the maximum number of corners
    AddNewFeatures(new_corners);
  } else {
    // printf("COLD START!\n");
    // Cold start, or we have all new features
    curr_corners_ = new_corners;
    prev_corners_ = new_corners;
    UpdateIdList(curr_corners_.size());
  }

  CreateFeatureArray(features);
  features->header.stamp = msg->header.stamp;

  // Update previous features and image
  prev_corners_ = curr_corners_;
  // We swap and we also need to copy the ImageConstPtr so the data behind the
  // cv::Mat will not be deallocated by the smart pointer.
  image_prev_ptr_ = msg;
  cv::swap(image_prev_, image_curr_);
}

void LKOpticalFlow::GetNewFeatures(std::vector<cv::Point2f>* new_corners) {
  // If there are already too many features .. don't do anything
  if (curr_corners_.size() >= max_feature_)
    return;

  // If it hasn't been a long enough time and we don't really need many points,
  // don't do anything. Save CPU time.
  if (curr_corners_.size() > 8 * max_feature_ / 10)
    return;

  // Get new corners from LK
  cv::goodFeaturesToTrack(image_curr_, *new_corners, 100, 0.01, max_gap_, cv::Mat(), 3, false, 0.04);
  // printf("Get new corners.\n");

  // printf("Found %lu new features, reduced to %lu.\n", raw_corners.size(), new_corners->size());
}

void LKOpticalFlow::AddNewFeatures(const std::vector<cv::Point2f>& new_corners) {
  // To ensure that push_back is fast, reserve space for all the features we
  // are going to add. Does nothing if vector is already larger.

  curr_corners_.reserve(max_feature_);
  prev_corners_.reserve(max_feature_);
  // Add new features avoiding duplication of the previous features->
  bool add_flag = true;
  for (auto const& it1 : new_corners) {
    // If we are over the limit, early exit
    if (curr_corners_.size() >= max_feature_)
      break;

    add_flag = true;
    for (auto const& it2 : curr_corners_) {
      if (cv::norm(it1 - it2) < max_gap_) {
        add_flag = false;
        break;
      }
    }
    if (add_flag) {
      curr_corners_.push_back(it1);
      prev_corners_.push_back(it1);
      UpdateIdList(1);
    }
  }
  // printf("Added corners from %lu to %lu.\n", initial_corners, curr_corners_.size());
}


void LKOpticalFlow::RefineCorners() {
  // Remove corners with 'false' status and with more displacement than max_flow_magnitude
  size_t k = 0;
  for (size_t i = 0; i < curr_corners_.size(); ++i) {
    // ignore pixels that weren't tracked, were too far away, and are near the image border
    bool too_far = cv::norm(prev_corners_[i] - curr_corners_[i]) > max_flow_magnitude_;
    int x_border_dist = std::min(curr_corners_[i].x, image_curr_.cols - curr_corners_[i].x);
    int y_border_dist = std::min(curr_corners_[i].y, image_curr_.rows - curr_corners_[i].y);
    bool on_border = x_border_dist < 10 || y_border_dist < 10;
    bool on_corner = x_border_dist < 60 && y_border_dist < 60;
    bool backwards_ok = cv::norm(prev_corners_[i] - backwards_corners_[i]) < 0.5 &&
                        backwards_status_[i];

    // ignore points that don't have the same matching in both directions
    if (status_[i] && !too_far && !on_border && !on_corner && backwards_ok) {
      curr_corners_[k] = curr_corners_[i];
      prev_corners_[k] = prev_corners_[i];
      id_list_[k] = id_list_[i];
      ++k;
    }
  }
  // printf("Filtered corners from %lu to %lu.\n", corners_[1].size(), k);
  curr_corners_.resize(k);
  prev_corners_.resize(k);
  id_list_.resize(k);
}

void LKOpticalFlow::UpdateIdList(const size_t& num_itr) {
  // Update ID while avoiding possible duplication
  for (size_t i = 0; i < num_itr; ++i) {
    id_list_.push_back(id_cnt_);
    do {
      ++id_cnt_;
      id_cnt_ %= id_max_;
    } while (std::find(id_list_.begin(), id_list_.end(), id_cnt_) != id_list_.end());
  }
}

void LKOpticalFlow::CreateFeatureArray(ff_msgs::Feature2dArray* features) {
  features->header = std_msgs::Header();
  features->feature_array.resize(id_list_.size());
  for (size_t i = 0; i < id_list_.size(); ++i) {
    features->feature_array[i].id = id_list_[i];

    // EKF expects measurements in the UNDISTORTED_C coordinate frame
    Eigen::Vector2d undistorted_c;
    camera_param_.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
    (Eigen::Vector2d(curr_corners_[i].x * scale_factor_, curr_corners_[i].y * scale_factor_),
     &undistorted_c);

    features->feature_array[i].x = undistorted_c[0];
    features->feature_array[i].y = undistorted_c[1];
  }
}

sensor_msgs::Image::Ptr LKOpticalFlow::ShowDebugWindow(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat image_drawing;
  cv::cvtColor(image_curr_, image_drawing, CV_GRAY2BGR);
  if (prev_corners_.size() != curr_corners_.size())
    prev_corners_ = curr_corners_;

  for (size_t i = 0; i < curr_corners_.size(); ++i) {
    // Visualize feature and vector
    cv::putText(image_drawing, std::to_string(id_list_[i]), curr_corners_[i],
                CV_FONT_NORMAL, 0.4, CV_RGB(255, 0, 0));
    cv::circle(image_drawing, curr_corners_[i], 2, cv::Scalar(0, 255, 255), -1, 8);
    cv::line(image_drawing, curr_corners_[i], prev_corners_[i], cv::Scalar(0, 255, 0), 2, 8, 0);
  }

  // Publish ros image
  sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", image_drawing).toImageMsg();
  return out_img;
}

}  // end namespace lk_optical_flow



