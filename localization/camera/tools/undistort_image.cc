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

#include <common/init.h>
#include <common/thread.h>
#include <config_reader/config_reader.h>
#include <camera/camera_params.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <iostream>

/*
  Undistort nav_cam images for a robot.
  Usage:

  export ASTROBEE_RESOURCE_DIR=$HOME/freeflyer/astrobee/resources
  export ASTROBEE_CONFIG_DIR=$HOME/freeflyer/astrobee/config
  export ASTROBEE_WORLD=granite
  export ASTROBEE_ROBOT=p4d
  undistort_image *[0-9].jpg

  Distorted images will be written to the same directory. 
  It is important to specify the correct robot name above.
*/

int main(int argc, char ** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (argc < 2)
    LOG(FATAL) << "Expecting at least one input image.";

  // Read the nav_cam
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles()) {
    LOG(ERROR) << "Failed to read config files.";
    return 1;
  }
  camera::CameraParameters cam_params(&config, "nav_cam");

  // Useful for over-riding any config files when debugging
  // camera::CameraParameters cam_params(Eigen::Vector2i(776, 517),
  //                                    Eigen::Vector2d::Constant(610.502),
  //                                    Eigen::Vector2d(776/2.0, 517/2.0));


  // Create the undistortion map
  cv::Mat floating_remap, fixed_map, interp_map;
  cam_params.GenerateRemapMaps(&floating_remap);

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
  int border_top  = -min_y, border_bottom = max_y - img_rows;
  int border_left = -min_x, border_right  = max_x - img_cols;

  // Adjust the remapping function to the expanded image.
  // Now all its values will be within bounds of that image.
  for (int col = 0; col < floating_remap.cols; col++) {
    for (int row = 0; row < floating_remap.rows; row++) {
      cv::Vec2f pix = floating_remap.at<cv::Vec2f>(row, col);
      pix[0] += border_left;
      pix[1] += border_top;
      floating_remap.at<cv::Vec2f>(row, col) = pix;
    }
  }

  // Convert the map for speed
  cv::convertMaps(floating_remap, cv::Mat(), fixed_map, interp_map, CV_16SC2);

  // Write some very useful info
  std::cout << "Distorted image size:   " << cam_params.GetDistortedSize().transpose() << "\n";
  std::cout << "Undistorted image size: " << cam_params.GetUndistortedSize().transpose() << "\n";
  std::cout << "Focal length:           " << cam_params.GetFocalLength() << "\n";
  std::cout << "Undistorted half size:  " << cam_params.GetUndistortedHalfSize().transpose() << "\n";

  for (int i = 1; i < argc; i++) {
    std::string filename(argv[i]);

    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    // Ensure that image dimensions are as expected
    if (image.rows != img_rows || image.cols != img_cols)
      LOG(FATAL) << "The input images have wrong dimensions.";

    // Expand the image before interpolating into it
    cv::Scalar paddingColor = 0;
    cv::Mat expanded_image;
    cv::copyMakeBorder(image, expanded_image, border_top, border_bottom, border_left, border_right,
                       cv::BORDER_CONSTANT, paddingColor);

    // Undistort it
    cv::Mat undist_image;
    cv::remap(expanded_image, undist_image, fixed_map, interp_map, cv::INTER_LINEAR);

    // Save to disk the undistorted image
    std::string undist_file = filename.substr(0, filename.size() - 4) + "_undistort.jpg";
    LOG(INFO) << "Writing: " << undist_file;
    cv::imwrite(undist_file, undist_image);
  }

  return 0;
}
