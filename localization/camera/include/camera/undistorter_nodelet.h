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
#ifndef CAMERA_UNDISTORTER_NODELET_H_
#define CAMERA_UNDISTORTER_NODELET_H_

#include <ff_util/ff_nodelet.h>
#include <sensor_msgs/Image.h>
#include <camera/camera_params.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace camera {
class UndistorterNodelet : public ff_util::FreeFlyerNodelet {
 public:
  UndistorterNodelet();
  void Initialize(ros::NodeHandle* nh) final;

 private:
  void initializeRectMap(camera::CameraParameters& cam_params);
  void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg);

  ros::NodeHandle *nh_;
  ros::Subscriber image_sub_;
  ros::Publisher rect_image_pub_;

  struct RectParams {
    cv::Rect crop_roi;
    cv::Mat fixed_map;
    cv::Mat interp_map;
    int border_right;
    int border_left;
    int border_top;
    int border_bottom;
  } rect_params_;
};
}

#endif // CAMERA_UNDISTORTER_NODELET_H_
