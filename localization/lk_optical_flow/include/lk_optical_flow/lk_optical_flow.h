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

#ifndef LK_OPTICAL_FLOW_LK_OPTICAL_FLOW_H_
#define LK_OPTICAL_FLOW_LK_OPTICAL_FLOW_H_

#include <camera/camera_params.h>
#include <ff_msgs/Feature2d.h>
#include <ff_msgs/Feature2dArray.h>

#include <cv_bridge/cv_bridge.h>
#include <config_reader/config_reader.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

namespace lk_optical_flow {

class LKOpticalFlow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LKOpticalFlow(void);
  ~LKOpticalFlow(void) {}

  void ReadParams(config_reader::ConfigReader* config);
  void OpticalFlow(const sensor_msgs::ImageConstPtr& msg,
                         ff_msgs::Feature2dArray* features);
  sensor_msgs::Image::Ptr ShowDebugWindow(const sensor_msgs::ImageConstPtr& msg);

 private:
  void AddNewFeatures(const std::vector<cv::Point2f>& new_points);
  void GetNewFeatures(std::vector<cv::Point2f>* new_corners);
  void CreateFeatureArray(ff_msgs::Feature2dArray* features);
  void RefineCorners();
  void UpdateIdList(const size_t& num_itr);

  cv::Mat image_curr_, image_prev_;

  sensor_msgs::ImageConstPtr image_prev_ptr_;
  std::vector<cv::Point2f> prev_corners_, curr_corners_, backwards_corners_;

  std::vector<uchar> status_, backwards_status_;
  std::vector<float> err_, backwards_err_;
  cv::Size win_size_;

  bool ignored_last_frame_;
  size_t max_feature_;
  int max_lk_pyr_level_;
  int max_lk_itr_;
  int max_gap_;
  float scale_factor_;

  float max_flow_magnitude_, font_size_, max_feature_rad_;
  int id_cnt_, id_max_;
  std::vector<int> id_list_;

  camera::CameraParameters camera_param_;
};
}  // end namespace lk_optical_flow

#endif  // LK_OPTICAL_FLOW_LK_OPTICAL_FLOW_H_
