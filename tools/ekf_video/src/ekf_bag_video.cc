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

#include <ekf_video/ekf_bag_video.h>

#include <msg_conversions/msg_conversions.h>
#include <camera/camera_params.h>
#include <Eigen/Core>

#include <QtGui/QImage>
#include <QtGui/QPainter>

namespace ekf_video {

EkfBagVideo::EkfBagVideo(const char* bagfile, const char* mapfile, const char* videofile) :
          EkfBag(bagfile, mapfile), video_(videofile, 1920, 1080) {
  // virtual function has to be called in subclass since not initialized in superclass
  config_reader::ConfigReader config;
  ReadParams(&config);

  start_time_set_ = false;
  pose_count_ = 0;
}

EkfBagVideo::~EkfBagVideo(void) {
}

void EkfBagVideo::ReadParams(config_reader::ConfigReader* config) {
  EkfBag::ReadParams(config);

  Eigen::Vector3d trans;
  Eigen::Quaterniond rot;
  if (!msg_conversions::config_read_transform(config, "nav_cam_transform", &trans, &rot))
    ROS_FATAL("Unspecified nav_cam_transform.");
  Eigen::Affine3d nav_cam_to_body = Eigen::Affine3d(
                    Eigen::Translation3d(trans.x(), trans.y(), trans.z())) *
                    Eigen::Affine3d(rot);
  camera::CameraParameters cam_params(config, "nav_cam");
  tracked_of_.SetCameraParameters(cam_params);
  tracked_sm_.SetCameraParameters(cam_params);
  tracked_sm_.SetCameraToBody(nav_cam_to_body);
}

void EkfBagVideo::DrawImage(QPainter & p, const sensor_msgs::ImageConstPtr & ros_image, const QRect & rect) {
  const QImage gray(ros_image->data.data(), ros_image->width, ros_image->height,
                    ros_image->step, QImage::Format_Grayscale8);
  // draw on image
  p.drawImage(rect, gray);
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::HighQualityAntialiasing);

  p.translate(rect.center().x(), rect.center().y());

  // draw optical flow
  p.setBrush(QColor::fromRgb(0x99, 0x99, 0xFF, 0xA0));
  for (auto it = tracked_of_.begin(); it != tracked_of_.end(); it++) {
    const auto & a = (*it).second;
    p.drawEllipse(QPointF(a.x, a.y), 4, 4);
  }

  // draw sparse mapping
  p.setBrush(QColor::fromRgb(0xFF, 0x00, 0x66, 0xA0));
  for (auto it = tracked_sm_.begin(); it != tracked_sm_.end(); it++) {
    Eigen::Vector2d pixel = tracked_sm_.FeatureToCurrentPixel(*it);
    if (pixel.x() < rect.width() / 2 && pixel.y() < rect.height() / 2)
      p.drawEllipse(QPointF(pixel.x(), pixel.y()), 4, 4);
  }

  p.resetTransform();
}

void EkfBagVideo::DrawTable(QPainter & p, const QRect & rect) {
  // draw robot pose
  p.translate(rect.center().x(), rect.center().y());
  p.scale(rect.width() / 2.13333, rect.height() / 2.13333);

  if (pose_history_.size() > 0) {
    p.fillRect(QRectF(-1.0, -1.0, 2.0, 2.0), QBrush(QColor::fromRgb(0xDD, 0xDD, 0xDD)));
    auto & pose = pose_history_.back();
    Eigen::Quaternionf o(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    auto euler = o.toRotationMatrix().eulerAngles(0, 1, 2);
    float zrot = (euler[2] + M_PI) * 180.0 / M_PI;
    p.translate(pose.position.x, pose.position.y);
    p.rotate(zrot);
    p.fillRect(QRectF(-0.1, -0.1, 0.2, 0.2), QColor::fromRgb(0xBB, 0xBB, 0x00));
    QPen pen(QColor::fromRgb(0, 0, 0));
    pen.setWidth(0.2);
    p.setPen(pen);
    p.drawLine(QLineF(0.0, 0.0, 0.25, 0.0));
    p.rotate(-zrot);
    p.translate(-pose.position.x, -pose.position.y);

    int i = 0;
    for (auto it = pose_history_.begin(); it != pose_history_.end(); it++) {
      pen.setColor(QColor::fromRgb(0x10, 0xDD, 0x10, (unsigned char)(255 * (i / 200.0))));
      p.setPen(pen);
      auto second = std::next(it);
      if (second == pose_history_.end())
        break;
      p.drawLine(QLineF(it->position.x, it->position.y, second->position.x, second->position.y));
      i++;
    }
  }

  p.resetTransform();
}

void EkfBagVideo::DrawMahalanobis(QPainter & p, const QRect & rect) {
  p.translate(rect.x(), rect.y());

  for (int i = 0; i < MAHAL_NUM_BINS; i++)
    if (mahal_bins_[i] > 0) {
      float height = std::min(mahal_bins_[i], MAHAL_BIN_MAX) / static_cast<float>(MAHAL_BIN_MAX) * rect.height();
      p.fillRect(QRectF(i * rect.width() / MAHAL_NUM_BINS, rect.height() - height,
            rect.width() / static_cast<float>(MAHAL_NUM_BINS), height),
            QBrush(QColor::fromRgb(0x00, 0xFF, 0x00)));
    }

  QPen pen(QColor::fromRgb(0xFF, 0xFF, 0xFF));
  pen.setWidth(1.0);
  p.setPen(pen);
  p.drawRect(rect);

  p.resetTransform();
}

void EkfBagVideo::UpdateImage(const ros::Time & time, const sensor_msgs::ImageConstPtr & ros_image) {
  EkfBag::UpdateImage(time, ros_image);

  if (!start_time_set_) {
    start_time_ = time;
    start_time_set_ = true;
  }

  QImage image(1920, 1080, QImage::Format_ARGB32);
  image.fill(QColor::fromRgb(0, 0, 0));
  QPainter p(&image);

  DrawImage(p, ros_image, QRect(0, 0, 1280, 960));

  // draw side widgets
  // starts at pixel 1280, width is 640
  DrawTable(p, QRect(1280, 0, 640, 640));
  DrawMahalanobis(p, QRect(1280, 640, 640, 60));

  p.end();
  video_.AddFrame(image);
}

void EkfBagVideo::UpdateEKF(const ff_msgs::EkfState & s) {
  EkfBag::UpdateEKF(s);

  state_ = s;

  pose_count_++;
  if (pose_count_ == 10) {
    pose_count_ = 0;
    if (pose_history_.size() > 200)
      pose_history_.pop_front();
    pose_history_.push_back(s.pose);
  }

  tracked_sm_.UpdatePose(s.pose);

  if (state_.ml_count > 0) {
    memset(mahal_bins_, 0, sizeof(int) * MAHAL_NUM_BINS);
    for (unsigned int i = 0; i < state_.ml_mahal_dists.size(); i++) {
      float m = state_.ml_mahal_dists[i];
      if (std::isnan(m))
        continue;
      mahal_bins_[std::min(MAHAL_NUM_BINS - 1, static_cast<int>(m / MAHAL_BIN_SIZE))]++;
    }
  }
}

void EkfBagVideo::UpdateOpticalFlow(const ff_msgs::Feature2dArray & of) {
  EkfBag::UpdateOpticalFlow(of);

  if (!start_time_set_)
    return;
  float t = (of.header.stamp - start_time_).toSec();

  tracked_of_.UpdateFeatures(of, t);
}

void EkfBagVideo::UpdateSparseMap(const ff_msgs::VisualLandmarks & vl) {
  EkfBag::UpdateSparseMap(vl);

  if (!start_time_set_)
    return;
  float t = (vl.header.stamp - start_time_).toSec();

  tracked_sm_.UpdateFeatures(vl, t);
}

}  // namespace ekf_video

