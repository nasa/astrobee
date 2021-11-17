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

#ifndef IS_CAMERA_CAMERA_H_
#define IS_CAMERA_CAMERA_H_

#include <config_reader/config_reader.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ff_msgs/SetBool.h>
#include <ff_util/ff_nodelet.h>

#include <thread>
#include <atomic>
#include <string>

namespace is_camera {

// Functions used to interact with Video4Linux
struct buffer {
  void *start;
  size_t length;
};

static void xioctl(int fh, int request, void *arg);

// Class to hide V4L structures
struct V4LStruct;

// Nodelet class
class CameraNodelet : public ff_util::FreeFlyerNodelet {
 public:
  static constexpr size_t kImageMsgBuffer = 15;  // 17.6 Mb of buffer space. This
                                                 // number is so large because
                                                 // this sets the life time for
                                                 // each image message sent
                                                 // out. Eventually we'll write
                                                 // over the pointer we've
                                                 // handed out. 15 frames means
                                                 // .. an image will stay valid
                                                 // for 1.0 seconds.
                                                 // Localization can process at
                                                 // 2 Hz, so it only needs an
                                                 // image for 0.5 seconds.
  static constexpr size_t kImageWidth = 1280;
  static constexpr size_t kImageHeight = 960;

  CameraNodelet();
  virtual ~CameraNodelet();
  void ReadParams(void);

 protected:
  virtual void Initialize(ros::NodeHandle* nh);

 private:
  void PublishLoop();
  bool EnableService(ff_msgs::SetBool::Request& req, ff_msgs::SetBool::Response& res);  // NOLINT
  void LoadCameraInfo();

  sensor_msgs::ImagePtr img_msg_buffer_[kImageMsgBuffer];
  sensor_msgs::CameraInfo info_msg_;
  size_t img_msg_buffer_idx_;
  std::thread thread_;
  std::atomic<bool> thread_running_;
  ros::Publisher pub_, info_pub_;
  std::shared_ptr<V4LStruct> v4l_;

  config_reader::ConfigReader config_;
  ros::Timer config_timer_;
  std::string output_topic_;
  std::string camera_device_;
  std::string camera_topic_;
  std::string config_name_;
  int camera_gain_, camera_exposure_;
  bool calibration_mode_;
};

}  // end namespace is_camera

#endif  // IS_CAMERA_CAMERA_H_
