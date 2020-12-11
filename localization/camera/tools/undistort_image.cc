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

#include <ff_common/init.h>
#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <config_reader/config_reader.h>
#include <camera/camera_params.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <string>
#include <iostream>

/*
  Undistort camera images for a robot.
  Usage:

  export ASTROBEE_RESOURCE_DIR=$SOURCE_PATH/astrobee/resources
  export ASTROBEE_CONFIG_DIR=$SOURCE_PATH/astrobee/config
  export ASTROBEE_WORLD=granite
  export ASTROBEE_ROBOT=p4d
  undistort_image *[0-9].jpg

  Distorted images will be written to the same directory, unless a
  separate directory is specified.
*/

DEFINE_string(image_list, "", "The list of images to undistort, one per line. If not specified, "
              "it is assumed they are passed in directly on the command line.");

DEFINE_string(output_directory, "", "If not specified, undistorted images will "
              "saved in the same directory as the inputs.");

DEFINE_double(scale, 1.0, "Undistort images at different resolution, with their width "
              "being a multiple of this scale compared to the camera model.");

DEFINE_string(undistorted_crop_win, "",
              "After undistorting, apply a crop window of these dimensions "
              "centered at the undistorted image center. The adjusted "
              "dimensions and optical center will be displayed below. "
              "Specify as: 'crop_x crop_y'.");

DEFINE_bool(save_bgr, false,
            "Save the undistorted images as BGR instead of grayscale. (Some tools expect BGR.)");

DEFINE_bool(histogram_equalization, false,
            "If true, do histogram equalization.");

DEFINE_string(robot_camera, "nav_cam",
              "Which of bot's cameras to use. Anything except nav_cam is experimental.");

int main(int argc, char ** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  // The images can either be in a list or passed in
  std::vector<std::string> images;
  if (FLAGS_image_list != "") {
    std::ifstream ifs(FLAGS_image_list);
    std::string image;
    while (ifs >> image)
      images.push_back(image);
  } else {
    for (int i = 1; i < argc; i++)
      images.push_back(argv[i]);
  }

  if (images.empty())
      LOG(FATAL) << "Expecting at least one input image.";

  // Read the camera
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }
  camera::CameraParameters cam_params(&config, FLAGS_robot_camera.c_str());

  // Useful for over-riding any config files when debugging
  // camera::CameraParameters cam_params(Eigen::Vector2i(776, 517),
  //                                    Eigen::Vector2d::Constant(610.502),
  //                                    Eigen::Vector2d(776/2.0, 517/2.0));

  if (!FLAGS_output_directory.empty()) {
    if (!boost::filesystem::create_directories(FLAGS_output_directory) &&
       !boost::filesystem::exists(FLAGS_output_directory)) {
      LOG(FATAL) << "Failed to create directory: " << FLAGS_output_directory;
    }
  }

  // Create the undistortion map
  cv::Mat floating_remap, fixed_map, interp_map;
  cam_params.GenerateRemapMaps(&floating_remap, FLAGS_scale);

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
  Eigen::Vector2i dims = FLAGS_scale*cam_params.GetDistortedSize();
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

  Eigen::Vector2i dist_size      = FLAGS_scale*cam_params.GetDistortedSize();
  Eigen::Vector2i undist_size    = FLAGS_scale*cam_params.GetUndistortedSize();
  double focal_length            = FLAGS_scale*cam_params.GetFocalLength();
  Eigen::Vector2d optical_center = FLAGS_scale*cam_params.GetUndistortedHalfSize();

  // Handle the cropping
  cv::Rect cropROI;
  if (!FLAGS_undistorted_crop_win.empty()) {
    std::vector<double> vals;
    ff_common::parseStr(FLAGS_undistorted_crop_win, vals);
    if (vals.size() < 2)
      LOG(FATAL) << "Could not parse --undistorted_crop_win.";

    int widx = vals[0];
    int widy = vals[1];

    // A couple of sanity checks
    if (widx % 2 != 0 || widy % 2 != 0 )
      LOG(FATAL) << "The cropped undistorted image dimensions must be even.";
    if (undist_size[0] % 2 != 0 || undist_size[1] % 2 != 0 )
      LOG(FATAL) << "The undistorted image dimensions must be even.";

    int startx = (undist_size[0] - widx)/2;
    int starty = (undist_size[1] - widy)/2;

    cropROI = cv::Rect(startx, starty, widx, widy);
    if (cropROI.empty())
      LOG(FATAL) << "Empty crop region.";
    std::cout << "Crop region: " << cropROI << std::endl;

    // Update these quantities
    undist_size[0]     = widx;
    undist_size[1]     = widy;
    optical_center[0] -= startx;
    optical_center[1] -= starty;
  }

  for (size_t i = 0; i < images.size(); i++) {
    std::string filename(images[i]);

    cv::Mat image = cv::imread(filename, cv::IMREAD_UNCHANGED);

    if (FLAGS_histogram_equalization) {
      cv::Mat tmp_image;
      cv::equalizeHist(image, tmp_image);
      image = tmp_image;
    }

    // Ensure that image dimensions are as expected
    if (image.rows != img_rows || image.cols != img_cols)
      LOG(FATAL) << "The input images have wrong dimensions.";

    // Expand the image before interpolating into it
    cv::Scalar paddingColor = 0;
    cv::Mat expanded_image;
    cv::copyMakeBorder(image, expanded_image, border_top, border_bottom,
                       border_left, border_right,
                       cv::BORDER_CONSTANT, paddingColor);

    // Undistort it
    cv::Mat undist_image;
    cv::remap(expanded_image, undist_image, fixed_map, interp_map, cv::INTER_LINEAR);

    // Crop, if desired
    if (!cropROI.empty()) {
      cv::Mat cropped_image;
      undist_image(cropROI).copyTo(cropped_image);  // without copyTo it is a shallow copy
      undist_image = cropped_image;  // this makes a shallow copy
    }

    // The output file name
    std::string undist_file;
    if (!FLAGS_output_directory.empty()) {
      // A separate output directory was specified
      undist_file = FLAGS_output_directory + "/" + ff_common::basename(filename);
    } else {
      // Save in same directory with new name
      undist_file = filename.substr(0, filename.size() - 4) + "_undistort.jpg";
    }

    // Save to disk the undistorted image
    std::cout << "Writing: " << undist_file << std::endl;
    cv::Mat bgr_image;
    if (FLAGS_save_bgr && undist_image.channels() == 1) {
      // Convert from grayscale to color if needed
      cvtColor(undist_image, bgr_image, CV_GRAY2BGR);
      undist_image = bgr_image;
    }
    cv::imwrite(undist_file, undist_image);
  }

  // Write some very useful info
  std::cout << "Distorted image size:       " << dist_size.transpose()      << "\n";
  std::cout << "Undistorted image size:     " << undist_size.transpose()    << "\n";
  std::cout << "Focal length:               " << focal_length               << "\n";
  std::cout << "Undistorted optical center: " << optical_center.transpose() << "\n";

  if (!FLAGS_output_directory.empty()) {
    std::string intrinsics_file = FLAGS_output_directory + "/undistorted_intrinsics.txt";
    std::ofstream ofs(intrinsics_file.c_str());
    ofs.precision(17);
    ofs << "# Unidistored width and height, focal length, undistorted optical center\n";
    ofs << undist_size.transpose() << " " << focal_length << " "
        << optical_center.transpose() << "\n";
    ofs.close();
    std::cout << "Wrote: " << intrinsics_file << std::endl;
  }

  return 0;
}
