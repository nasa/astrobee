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

#include <camera/camera_params.h>
#include <common/init.h>
#include <marker_tracking/marker_detector.h>

#include <glog/logging.h>
#include <alvar/Camera.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

DEFINE_string(camera_calibration, "",
              "The camera calibration file, in OpenCV's XML format.");

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);

  if (FLAGS_camera_calibration.empty())
    LOG(FATAL) << "Camera calibration file was not specified.";

  camera::CameraParameters cam_params(FLAGS_camera_calibration);
  marker_tracking::MarkerCornerDetector detector(cam_params);

  cv::Scalar colors[4];
  colors[0] = cv::Scalar(0);
  colors[1] = cv::Scalar(128);
  colors[2] = cv::Scalar(0);
  colors[3] = cv::Scalar(255);

  cv::Mat image;
  IplImage ipl_image;
  for (int i = 1; i < argc; i++) {
    std::string image_filename(argv[i]);
    image = cv::imread(image_filename, CV_LOAD_IMAGE_GRAYSCALE);
    ipl_image = image;
    detector.Detect(&ipl_image, 0.08, 0.2);
    LOG(INFO) << image_filename << " : Detected " << detector.NumMarkers() << " markers.";

    for (size_t m_id = 0; m_id < detector.NumMarkers(); m_id++) {
      alvar::MarkerData& marker = detector.GetMarker(m_id);
      LOG(INFO) << "\tI see #" << marker.GetId();
      for (size_t c = 0; c < 4; c++) {
        cv::circle(image,
            cv::Point2f(marker.marker_corners_img[c].x,
              marker.marker_corners_img[c].y),
            3, colors[c]);
      }
    }
    cv::imwrite(image_filename.substr(0, image_filename.size() - 4) + "_plot.jpg", image);
  }

  return 0;
}
