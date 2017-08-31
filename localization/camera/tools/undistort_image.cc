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
#include <common/utils.h>
#include <camera/camera_params.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

DEFINE_string(camera_calibration, "",
              "The camera calibration file, in OpenCV's XML format.");

int main(int argc, char ** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_camera_calibration.empty())
    LOG(FATAL) << "Camera calibration file was not specified.";

  camera::CameraParameters cam_params(FLAGS_camera_calibration);

  cv::Mat floating_remap, fixed_map, interp_map;
  cam_params.GenerateRemapMaps(&floating_remap);
  cv::convertMaps(floating_remap, cv::Mat(),
      fixed_map, interp_map, CV_16SC2);

  for (int i = 1; i < argc; i++) {
    std::string filename(argv[i]);

    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    image = image(cv::Rect(cam_params.GetCropOffset()[0], cam_params.GetCropOffset()[1],
      cam_params.GetDistortedSize()[0], cam_params.GetDistortedSize()[1]));

    cv::Mat dewarped_image;
    cv::remap(image, dewarped_image, fixed_map, interp_map, cv::INTER_LINEAR);
    cv::imwrite(filename.substr(0, filename.size() - 4) + "_undistort.jpg", dewarped_image);
  }

  return 0;
}
