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

#ifndef EKF_VIDEO_EKF_BAG_VIDEO_H_
#define EKF_VIDEO_EKF_BAG_VIDEO_H_

#include <ekf_bag/ekf_bag.h>
#include <ekf_bag/tracked_features.h>
#include <ekf_video/video_writer.h>

#include <ff_msgs/EkfState.h>
#include <geometry_msgs/Pose.h>

#include <list>
#include <string>

class QPainter;
class QRect;

#define MAHAL_NUM_BINS 20
#define MAHAL_BIN_SIZE 0.5

namespace ekf_video {

class EkfBagVideo : public ekf_bag::EkfBag {
 public:
  EkfBagVideo(const char* bagfile, const char* mapfile, const char* videofile);
  virtual ~EkfBagVideo(void);

 protected:
  virtual void UpdateImage(const ros::Time & time, const sensor_msgs::ImageConstPtr & image);

  virtual void UpdateEKF(const ff_msgs::EkfState & state);
  virtual void UpdateOpticalFlow(const ff_msgs::Feature2dArray & of);
  virtual void UpdateSparseMap(const ff_msgs::VisualLandmarks & vl);

  virtual void ReadParams(config_reader::ConfigReader* config);

 private:
  void DrawImage(QPainter & p, const sensor_msgs::ImageConstPtr & ros_image, const QRect & rect);
  void DrawTable(QPainter & p, const QRect & rect);

  void DrawVelocity(QPainter & p, const QRect & rect);
  void DrawOmega(QPainter & p, const QRect & rect);
  void DrawAccelBias(QPainter & p, const QRect & rect);
  void DrawGyroBias(QPainter & p, const QRect & rect);
  void DrawPositionCov(QPainter & p, const QRect & rect);
  void DrawVelocityCov(QPainter & p, const QRect & rect);
  void DrawAngleCov(QPainter & p, const QRect & rect);

  void DrawStatus(QPainter & p, const QRect & rect);
  void DrawSMFeatureCount(QPainter & p, const QRect & rect);
  void DrawOFFeatureCount(QPainter & p, const QRect & rect);

  void DrawMahalanobis(QPainter & p, const QRect & rect);

  void DrawBarGraph(QPainter & p, const QRect & rect,
                    int data_count, const float* data,
                    float y_min, float y_max,
                    const char* title, const char* xlabel, const char* ylabel,
                    const std::string* data_labels, bool data_labels_on_bars);

  VideoWriter video_;

  ekf_bag::TrackedOFFeatures tracked_of_;
  ekf_bag::TrackedSMFeatures tracked_sm_;

  std::list<geometry_msgs::Pose> pose_history_;
  int pose_count_;

  bool start_time_set_;
  ros::Time start_time_;

  ff_msgs::EkfState state_;
  int of_count_;
  int ml_count_;

  int mahal_bins_[MAHAL_NUM_BINS];
};

}  // end namespace ekf_video

#endif  // EKF_VIDEO_EKF_BAG_VIDEO_H_

