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

#include <camera/undistorter_nodelet.h>

namespace camera {
UndistorterNodelet::UndistorterNodelet() : ff_util::FreeFlyerNodelet(NODE_UNDISTORTER, true) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }
  camera::CameraParameters cam_params(&config, "nav_cam");
  initializeRectMap(cam_params);
}

void UndistorterNodelet::initializeRectMap(camera::CameraParameters& cam_params) {
  // Largely taken from undistort_image.cc
  cv::Mat floating_remap;
  cam_params.GenerateRemapMaps(&floating_remap, 1);

  // Deal with the fact that this map may request pixels from the
  // image that are out of bounds. Hence we will need to grow the
  // image before interpolating into it.

  // Find the expanded image bounds
  cv::Vec2f start = floating_remap.at<cv::Vec2f>(0, 0);
  double min_x = start[0], max_x = start[0];
  double min_y = start[1], max_y = start[1];
  for (int col = 0; col < floating_remap.cols; col++) {
    for (int row = 0; row < floating_remap.rows; row++) {
      cv::Vec2f pix = floating_remap.at<cv::Vec2f>(row, col);
      if (pix[0] < min_x)
        min_x = pix[0];
      if (pix[0] > max_x)
        max_x = pix[0];
      if (pix[1] < min_y)
        min_y = pix[1];
      if (pix[1] > max_y)
        max_y = pix[1];
    }
  }
  min_x = floor(min_x); max_x = ceil(max_x);
  min_y = floor(min_y); max_y = ceil(max_y);

  // The image dimensions
  Eigen::Vector2i dims = cam_params.GetDistortedSize();
  int img_cols = dims[0], img_rows = dims[1];

  // Ensure that the expanded image is not smaller than the old one,
  // to make the logic simpler
  if (min_x > 0)
    min_x = 0;
  if (max_x < img_cols)
    max_x = img_cols;
  if (min_y > 0)
    min_y = 0;
  if (max_y < img_rows)
    max_y = img_rows;

  // Convert the bounds to what cv::copyMakeBorder() will expect
  rect_params_.border_top  = -min_y, rect_params_.border_bottom = max_y - img_rows;
  rect_params_.border_left = -min_x, rect_params_.border_right  = max_x - img_cols;

  // Adjust the remapping function to the expanded image.
  // Now all its values will be within bounds of that image.
  for (int col = 0; col < floating_remap.cols; col++) {
    for (int row = 0; row < floating_remap.rows; row++) {
      cv::Vec2f pix = floating_remap.at<cv::Vec2f>(row, col);
      pix[0] += rect_params_.border_left;
      pix[1] += rect_params_.border_top;
      floating_remap.at<cv::Vec2f>(row, col) = pix;
    }
  }

  // Convert the map for speed
  cv::convertMaps(floating_remap, cv::Mat(), rect_params_.fixed_map, rect_params_.interp_map, CV_16SC2);

  // Handle the cropping
  Eigen::Vector2i start_remap = (cam_params.GetUndistortedSize() - dims)/2;
  rect_params_.crop_roi = cv::Rect(start_remap[0], start_remap[1], dims[0], dims[1]);
}

void UndistorterNodelet::Initialize(ros::NodeHandle* nh) {
  ff_common::InitFreeFlyerApplication(getMyArgv());
  nh_ = nh;

  image_sub_ = nh_->subscribe<sensor_msgs::Image>(TOPIC_HARDWARE_NAV_CAM, 1, &UndistorterNodelet::imageCallback, this);
  rect_image_pub_ = nh_->advertise<sensor_msgs::Image>(std::string(TOPIC_HARDWARE_NAV_CAM) + "/rectified", 1);
}

void UndistorterNodelet::imageCallback(const sensor_msgs::Image::ConstPtr& img_msg) {
  auto img_cv = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  // Expand the image before interpolating into it
  cv::Scalar paddingColor = 0;
  cv::Mat expanded_image;
  cv::copyMakeBorder(img_cv->image, expanded_image, rect_params_.border_top, rect_params_.border_bottom,
                     rect_params_.border_left, rect_params_.border_right,
                     cv::BORDER_CONSTANT, paddingColor);

  // Undistort it
  cv::Mat undist_image;
  cv::remap(expanded_image, undist_image, rect_params_.fixed_map, rect_params_.interp_map, cv::INTER_LINEAR);

  auto rect_img_msg = cv_bridge::CvImage(img_msg->header, "mono8", undist_image(rect_params_.crop_roi));
  rect_image_pub_.publish(rect_img_msg.toImageMsg());
}
}

PLUGINLIB_EXPORT_CLASS(camera::UndistorterNodelet, nodelet::Nodelet);
