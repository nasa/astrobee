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

#include <marker_tracking/marker_detector.h>
#include <camera/camera_params.h>

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <utility>
#include <vector>

TEST(MarkerDetector, TestDetection) {
  std::string data_dir = std::string(std::getenv("DATA_DIR"));

  // Create a fake alvar camera for our work.
  camera::CameraParameters cam(Eigen::Vector2i(816, 612),
      Eigen::Vector2d::Constant(2), Eigen::Vector2d(408, 306));

  marker_tracking::MarkerCornerDetector detector(cam);

  // These are the 3 images that we expect to detect an AR tag in.
  std::vector<std::string> image_filenames;
  image_filenames.push_back("IMG_20141217_160451.small.opt.jpg");
  image_filenames.push_back("IMG_20141217_160458.small.opt.jpg");
  image_filenames.push_back("IMG_20141217_160509.small.opt.jpg");

  // Expected measurements
  // 317.795 464.781
  // 349.533 482.319
  // 51.8153 536.814
  std::vector<std::pair<int, int> > expected_corner_loc;
  expected_corner_loc.push_back(std::make_pair(318, 465));
  expected_corner_loc.push_back(std::make_pair(350, 482));
  expected_corner_loc.push_back(std::make_pair(52, 537));

  cv::Mat image;
  IplImage ipl_image;
  std::vector<std::pair<int, int> >::iterator expected_it = expected_corner_loc.begin();
  for (std::string const& image_filename : image_filenames) {
    image = cv::imread(data_dir + image_filename, CV_LOAD_IMAGE_GRAYSCALE);
    ipl_image = image;
    detector.Detect(&ipl_image, 0.08, 0.2);
    EXPECT_EQ(1u, detector.NumMarkers());

    alvar::MarkerData& marker = detector.GetMarker(0);
    ASSERT_NE(static_cast<alvar::MarkerData*>(NULL), &marker);
    EXPECT_EQ(2u, marker.GetId());
    EXPECT_NEAR(expected_it->first, marker.marker_corners_img[0].x, 1);
    EXPECT_NEAR(expected_it->second, marker.marker_corners_img[0].y, 1);
    expected_it++;
  }
}
