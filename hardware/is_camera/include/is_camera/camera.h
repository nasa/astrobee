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
  // The size of the image message ring buffer for publishing
  // grayscale images.  15 images = 17.6 MB of buffer space. This
  // number is so large because it sets the lifetime for each image
  // message sent out. Eventually we'll write over the pointer we've
  // handed out. At 15 Hz, 15 frames means an image will stay valid
  // for 1.0 seconds.  Localization can process at 2 Hz, so it only
  // needs an image for 0.5 seconds.
  static constexpr size_t kImageMsgBuffer = 15;

  // The size of the image message ring buffer for publishing raw
  // Bayer format images.  The same comments apply about message
  // lifetime, but in this case we're expecting the subscriber
  // callback to at most de-Bayer the image before passing it on, so
  // the buffer doesn't need to be so long.
  static constexpr size_t kBayerImageMsgBufferLength = 5;

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
  size_t getNumBayerSubscribers();

  sensor_msgs::ImagePtr img_msg_buffer_[kImageMsgBuffer];
  sensor_msgs::ImagePtr bayer_img_msg_buffer_[kBayerImageMsgBufferLength];
  size_t img_msg_buffer_idx_;
  size_t bayer_img_msg_buffer_idx_;
  std::thread thread_;
  std::atomic<bool> thread_running_;
  ros::Publisher pub_;
  ros::Publisher bayer_pub_;
  std::shared_ptr<V4LStruct> v4l_;

  config_reader::ConfigReader config_;
  ros::Timer config_timer_;
  std::string camera_device_;
  std::string camera_topic_;
  std::string bayer_camera_topic_;
  std::string config_name_;
  int camera_gain_, camera_exposure_;
  bool calibration_mode_;

  // bayer_enable: Set to true to enable publishing raw Bayer image
  // (can be converted to RGB). May incur significant I/O overhead.
  bool bayer_enable_;

  // bayer_throttle_ratio: Set to n to publish 1 raw Bayer image for
  // every n images grabbed. With n = 1, every image is
  // published. Larger n reduces I/O overhead.
  unsigned int bayer_throttle_ratio_;
  size_t bayer_throttle_ratio_counter_;
};

}  // end namespace is_camera

#endif  // IS_CAMERA_CAMERA_H_
