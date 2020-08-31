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

#include <interest_point/matching.h>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <vector>

class MatchingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // TEST_DIR is defined as this modules test directory. However
    // that is not where the test imagery is located.
    std::string data_dir = std::string(TEST_DIR) + "/../../sparse_mapping/test/data";
    image1               = cv::imread(data_dir + "/m0004000.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    image2               = cv::imread(data_dir + "/m0004025.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    // Decimate the image to speed things up for our unit tests
    cv::resize(image1, image1, cv::Size(), 0.7, 0.7, cv::INTER_AREA);
    cv::resize(image2, image2, cv::Size(), 0.7, 0.7, cv::INTER_AREA);
  }

  void DetectKeyPoints(std::string const& detector) {
    interest_point::FeatureDetector ipdetect(detector);
    ipdetect.Detect(image1, &keypoints1, &descriptor1);
    ipdetect.Detect(image2, &keypoints2, &descriptor2);
  }

  cv::Mat                   image1, image2;
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  cv::Mat                   descriptor1, descriptor2;
  std::vector<cv::DMatch>   matches;
};

TEST_F(MatchingTest, SURF) {
  DetectKeyPoints("SURF");
  interest_point::FindMatches(descriptor1, descriptor2, &matches);
  EXPECT_NEAR(keypoints1.size(), 2023u, 10);
  EXPECT_NEAR(keypoints2.size(), 2047u, 10);
  EXPECT_EQ(descriptor1.cols, 64);
  // KNN matching seems to be variable
  EXPECT_NEAR(matches.size(), 798u, 10);
}

TEST_F(MatchingTest, ORGBRISK) {
  DetectKeyPoints("ORGBRISK");
  interest_point::FindMatches(descriptor1, descriptor2, &matches);
  EXPECT_LT(100u, keypoints1.size());
  EXPECT_LT(100u, keypoints2.size());
  EXPECT_EQ(64, descriptor1.cols);
  EXPECT_LT(50u, matches.size());
}
