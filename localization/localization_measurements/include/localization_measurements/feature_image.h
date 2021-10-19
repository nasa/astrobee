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
#ifndef LOCALIZATION_MEASUREMENTS_FEATURE_IMAGE_H_
#define LOCALIZATION_MEASUREMENTS_FEATURE_IMAGE_H_

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace localization_measurements {
class FeatureImage {
 public:
  FeatureImage(const cv::Mat& image, const cv::Ptr<cv::Feature2D> feature_detector) : image_(image) {
    feature_detector->detectAndCompute(image_, cv::Mat(), keypoints_, descriptors_);
    cv::KeyPoint::convert(keypoints_, feature_points_);
  }
  const cv::Mat& image() const { return image_; }
  const int rows() const { return image_.rows; }
  const int cols() const { return image_.cols; }
  const std::vector<cv::KeyPoint>& keypoints() const { return keypoints_; }
  const std::vector<cv::Point2f>& feature_points() const { return feature_points_; }
  const cv::Mat& descriptors() const { return descriptors_; }

 private:
  cv::Mat image_;
  std::vector<cv::KeyPoint> keypoints_;
  std::vector<cv::Point2f> feature_points_;
  cv::Mat descriptors_;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_FEATURE_IMAGE_H_
