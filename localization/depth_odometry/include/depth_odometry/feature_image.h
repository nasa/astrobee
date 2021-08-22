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
#ifndef DEPTH_ODOMETRY_FEATURE_IMAGE_H_
#define DEPTH_ODOMETRY_FEATURE_IMAGE_H_

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace depth_odometry {
class FeatureImage {
 public:
  FeatureImage(const cv::Mat& image, const cv::Ptr<cv::Feature2D> feature_detector);
  const cv::Mat& image() const { return image_; }
  const std::vector<cv::KeyPoint>& keypoints() const { return keypoints_; }
  const cv::Mat& descriptors() const { return descriptors_; }

 private:
  cv::Mat image_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;
};
}  // namespace depth_odometry

#endif  // DEPTH_ODOMETRY_FEATURE_IMAGE_H_
