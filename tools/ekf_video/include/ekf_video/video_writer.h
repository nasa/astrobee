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

#ifndef EKF_VIDEO_VIDEO_WRITER_H_
#define EKF_VIDEO_VIDEO_WRITER_H_

#include <sensor_msgs/Image.h>

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;

class QImage;

namespace ekf_video {

class VideoWriter {
 public:
  VideoWriter(const char* videofile, int width, int height);
  virtual ~VideoWriter(void);

  virtual void AddFrame(const QImage & image);

 private:
  void OpenVideo(const char* videofile);
  void CloseVideo();

  int EncodeFrame(AVFrame* frame, int* output_received);

  int width_, height_;

  struct AVFormatContext* format_context_;
  struct AVCodecContext* codec_context_;
  struct AVFrame* frame_;
};

}  // end namespace ekf_video

#endif  // EKF_VIDEO_VIDEO_WRITER_H_

