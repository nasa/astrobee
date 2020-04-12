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

#include <camera/camera_params.h>
#include <msg_conversions/msg_conversions.h>
#include <Eigen/Core>

#include <QtGui/QImage>
#include <QtGui/QPainter>

#include <iomanip>
#include <sstream>

namespace ekf_video {

EkfBagVideo::EkfBagVideo(const char* bagfile, const char* mapfile,
                         const char* videofile, bool run_ekf, bool gen_features,
                         bool use_jem, const char* biasfile,
                         std::string image_topic)
    : EkfBag(bagfile, mapfile, run_ekf, gen_features, biasfile, image_topic),
      video_(videofile, 1920, 1080),
      use_jem_(use_jem) {
  // virtual function has to be called in subclass since not initialized in
  // superclass
  config_reader::ConfigReader config;
  ReadParams(&config);

  start_time_set_ = false;
  last_draw_time_ = ros::Time(0, 0);
  pose_count_ = 0;
}

EkfBagVideo::~EkfBagVideo(void) {}

void EkfBagVideo::ReadParams(config_reader::ConfigReader* config) {
  EkfBag::ReadParams(config);

  Eigen::Vector3d trans;
  Eigen::Quaterniond rot;
  if (!msg_conversions::config_read_transform(config, "nav_cam_transform",
                                              &trans, &rot))
    ROS_FATAL("Unspecified nav_cam_transform.");
  Eigen::Affine3d nav_cam_to_body =
      Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) *
      Eigen::Affine3d(rot);
  camera::CameraParameters cam_params(config, "nav_cam");
  tracked_of_.SetCameraParameters(cam_params);
  tracked_sm_.SetCameraParameters(cam_params);
  tracked_sm_.SetCameraToBody(nav_cam_to_body);
}

void EkfBagVideo::DrawImage(QPainter& p,
                            const sensor_msgs::ImageConstPtr& ros_image,
                            const QRect& rect) {
  if (ros_image) {
    const QImage gray(ros_image->data.data(), ros_image->width,
                      ros_image->height, ros_image->step,
                      QImage::Format_Grayscale8);
    // draw on image
    p.drawImage(rect, gray);
  }
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::HighQualityAntialiasing);

  p.translate(rect.center().x(), rect.center().y());

  // draw optical flow
  p.setBrush(QColor::fromRgb(0x99, 0x99, 0xFF, 0xA0));
  for (auto it = tracked_of_.begin(); it != tracked_of_.end(); it++) {
    const auto& a = (*it).second;
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

void EkfBagVideo::DrawPose(QPainter& p, const QRect& rect) {
  if (use_jem_) {
    p.fillRect(QRectF(rect.x(), rect.y(), rect.width(), rect.height()),
               QBrush(QColor::fromRgb(0xDD, 0xDD, 0xDD)));

    // Bounding volume of the JEM
    float y_range[2] = {-11.5, -2.5};  // 9.0m
    float x_range[2] = {9.85, 12.0};   // 2.15m
    float z_range[2] = {3.8, 5.95};    // 2.15m
    // screen-X = world-Y and screen-Y = -world-X
    // Limitation dimmension is along Y axis
    float margin = 4.0;
    float scale = (rect.width() - 2.0 * margin) / (y_range[1] - y_range[0]);

    // Top View
    p.resetTransform();
    p.translate(rect.x() + margin, rect.y() + margin);
    p.scale(scale, scale);
    p.translate(-y_range[0], x_range[1]);

    // Draw axes (X=red, Y=green)
    p.setPen(QPen(Qt::red, 0.03));
    p.drawLine(QLineF(y_range[0], -x_range[0], y_range[0], -x_range[1]));
    p.setPen(QPen(Qt::green, 0.03));
    p.drawLine(QLineF(y_range[0], -x_range[0], y_range[1], -x_range[0]));
    p.setPen(QPen(Qt::gray, 0.03));
    p.drawLine(QLineF(y_range[1], -x_range[1], y_range[1], -x_range[0]));
    p.drawLine(QLineF(y_range[1], -x_range[1], y_range[0], -x_range[1]));

    if (pose_history_.size() > 0) {
      // Draw robot
      auto& pose = pose_history_.back();
      Eigen::Quaternionf o(pose.orientation.w, pose.orientation.x,
                           pose.orientation.y, pose.orientation.z);
      auto rot = o.toRotationMatrix();
      auto euler = rot.eulerAngles(0, 1, 2);
      float zrot = (euler[2] - M_PI / 2.0) * 180.0 / M_PI;
      p.translate(pose.position.y, -pose.position.x);
      p.rotate(zrot);
      p.fillRect(QRectF(-0.1, -0.1, 0.2, 0.2),
                 QColor::fromRgb(0xBB, 0xBB, 0x00));
      p.rotate(-zrot);
      p.setPen(QPen(Qt::black, 0.02));
      // Draw X robot axis
      p.drawLine(QLineF(0.0, 0.0, 0.5 * rot(1, 0), -0.5 * rot(0, 0)));
      p.translate(-pose.position.y, pose.position.x);
      // Draw pose history
      p.setPen(QPen(Qt::cyan, 0.01));
      for (auto it = pose_history_.begin(); it != pose_history_.end(); it++) {
        auto second = std::next(it);
        if (second == pose_history_.end()) break;
        p.drawLine(QLineF(it->position.y, -it->position.x, second->position.y,
                          -second->position.x));
      }
    }
    // Side View
    p.resetTransform();
    p.translate(rect.x() + margin, rect.y() + margin + rect.height() / 3.0);
    p.scale(scale, scale);
    p.translate(-y_range[0], -z_range[0]);
    // Draw axes (Z=blue, Y=green)
    p.setPen(QPen(Qt::blue, 0.03));
    p.drawLine(QLineF(y_range[0], z_range[0], y_range[0], z_range[1]));
    p.setPen(QPen(Qt::green, 0.03));
    p.drawLine(QLineF(y_range[0], z_range[0], y_range[1], z_range[0]));
    p.setPen(QPen(Qt::gray, 0.03));
    p.drawLine(QLineF(y_range[1], z_range[0], y_range[1], z_range[1]));
    p.drawLine(QLineF(y_range[1], z_range[1], y_range[0], z_range[1]));

    if (pose_history_.size() > 0) {
      // Draw robot
      auto& pose = pose_history_.back();
      Eigen::Quaternionf o(pose.orientation.w, pose.orientation.x,
                           pose.orientation.y, pose.orientation.z);
      auto rot = o.toRotationMatrix();
      auto euler = rot.eulerAngles(2, 0, 1);
      float yrot = -euler[2] * 180.0 / M_PI;
      p.translate(pose.position.y, pose.position.z);
      p.rotate(yrot);
      p.fillRect(QRectF(-0.1, -0.1, 0.2, 0.2),
                 QColor::fromRgb(0xBB, 0xBB, 0x00));
      p.rotate(-yrot);
      p.setPen(QPen(Qt::black, 0.02));
      // Draw X robot axis
      p.drawLine(QLineF(0.0, 0.0, 0.5 * rot(1, 0), 0.5 * rot(2, 0)));
      p.translate(-pose.position.y, -pose.position.z);
      // Draw pose history
      p.setPen(QPen(Qt::magenta, 0.01));
      for (auto it = pose_history_.begin(); it != pose_history_.end(); it++) {
        auto second = std::next(it);
        if (second == pose_history_.end()) break;
        p.drawLine(QLineF(it->position.y, it->position.z, second->position.y,
                          second->position.z));
      }
    }
  } else {
    // draw robot pose
    p.translate(rect.center().x(), rect.center().y());
    p.scale(rect.width() / 2.13333, rect.height() / 2.13333);

    if (pose_history_.size() > 0) {
      p.fillRect(QRectF(-1.0, -1.0, 2.0, 2.0),
                 QBrush(QColor::fromRgb(0xDD, 0xDD, 0xDD)));
      auto& pose = pose_history_.back();
      Eigen::Quaternionf o(pose.orientation.w, pose.orientation.x,
                           pose.orientation.y, pose.orientation.z);
      auto euler = o.toRotationMatrix().eulerAngles(0, 1, 2);
      float zrot = (euler[2] + M_PI) * 180.0 / M_PI;
      p.translate(pose.position.x, pose.position.y);
      p.rotate(zrot);
      p.fillRect(QRectF(-0.1, -0.1, 0.2, 0.2),
                 QColor::fromRgb(0xBB, 0xBB, 0x00));
      QPen pen(QColor::fromRgb(0, 0, 0));
      pen.setWidth(0.2);
      p.setPen(pen);
      p.drawLine(QLineF(0.0, 0.0, 0.25, 0.0));
      p.rotate(-zrot);
      p.translate(-pose.position.x, -pose.position.y);

      int i = 0;
      for (auto it = pose_history_.begin(); it != pose_history_.end(); it++) {
        pen.setColor(QColor::fromRgb(0x10, 0xDD, 0x10,
                                     (unsigned char)(255 * (i / 200.0))));
        p.setPen(pen);
        auto second = std::next(it);
        if (second == pose_history_.end()) break;
        p.drawLine(QLineF(it->position.x, it->position.y, second->position.x,
                          second->position.y));
        i++;
      }
    }
  }
  p.resetTransform();
}

void EkfBagVideo::DrawBarGraph(QPainter& p, const QRect& rect, int data_count,
                               const float* data, float y_min, float y_max,
                               const char* title, const char* xlabel,
                               const char* ylabel,
                               const std::string* data_labels,
                               bool data_labels_on_bars) {
  int TITLE_HEIGHT = title == NULL ? 0 : 25;
  int LABELS_HEIGHT = data_labels == NULL ? 0 : 20;
  int YLABEL_WIDTH = ylabel == NULL ? 0 : 20;
  int XLABEL_HEIGHT = xlabel == NULL ? 0 : 30;
  int TICKS_WIDTH = 25;
  int GRAPH_HEIGHT =
      rect.height() - LABELS_HEIGHT - TITLE_HEIGHT - XLABEL_HEIGHT - 5;
  int GRAPH_WIDTH = rect.width() - TICKS_WIDTH - YLABEL_WIDTH - 10;
  float BIN_WIDTH = GRAPH_WIDTH / static_cast<float>(data_count);

  QPen white(QColor::fromRgb(0xFF, 0xFF, 0xFF));
  p.setPen(white);
  QFont font("Arial");

  p.translate(rect.x(), rect.y() + 5);

  // draw title
  font.setPointSize(8);
  p.setFont(font);
  p.drawText(QRectF(0, 0, rect.width(), TITLE_HEIGHT), Qt::AlignCenter, title);
  p.translate(0, TITLE_HEIGHT);

  float ZERO_HEIGHT = (std::max(std::min(0.0f, y_max), y_min) - y_min) /
                      (y_max - y_min) * GRAPH_HEIGHT;

  // draw left side
  // ylabel
  if (ylabel != NULL) {
    p.save();
    p.translate(YLABEL_WIDTH / 2, GRAPH_HEIGHT / 2);
    p.rotate(-90);
    font.setPointSize(6);
    p.setFont(font);
    p.drawText(QRectF(-GRAPH_HEIGHT / 2, -YLABEL_WIDTH / 2, GRAPH_HEIGHT,
                      YLABEL_WIDTH),
               Qt::AlignCenter, ylabel);
    p.restore();
  }

  p.translate(YLABEL_WIDTH, 0);
  font.setPointSize(4);
  int font_height = p.fontMetrics().height() / 2;
  p.setFont(font);
  // axis labels
  {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << y_max;
    p.drawText(QRectF(0, -font_height / 2, TICKS_WIDTH - 6, font_height),
               Qt::AlignRight, stream.str().c_str());
  }
  p.drawLine(QLineF(TICKS_WIDTH - 5, 0, TICKS_WIDTH, 0));
  {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << y_min;
    p.drawText(
        QRectF(0, GRAPH_HEIGHT - font_height / 2, TICKS_WIDTH - 6, font_height),
        Qt::AlignRight, stream.str().c_str());
  }
  p.drawLine(QLineF(TICKS_WIDTH - 5, GRAPH_HEIGHT, TICKS_WIDTH, GRAPH_HEIGHT));
  if (y_max > 0.0 && y_min < 0.0) {
    p.drawText(
        QRectF(0, ZERO_HEIGHT - font_height / 2, TICKS_WIDTH - 6, font_height),
        Qt::AlignRight, "0");
    p.drawLine(QLineF(TICKS_WIDTH - 5, ZERO_HEIGHT, TICKS_WIDTH, ZERO_HEIGHT));
  }

  // draw content of graph
  p.translate(TICKS_WIDTH, 0);
  for (int i = 0; i < data_count; i++) {
    float height = (std::max(std::min(data[i], y_max), y_min) - y_min) /
                   (y_max - y_min) * GRAPH_HEIGHT;
    p.fillRect(QRectF(i * BIN_WIDTH, GRAPH_HEIGHT - height, BIN_WIDTH,
                      height - ZERO_HEIGHT),
               QBrush(QColor::fromRgb(0x00, 0xFF, 0x00)));
  }
  // draw bounding box
  white.setWidth(1.0);
  p.setPen(white);
  p.setBrush(Qt::NoBrush);
  p.drawRect(QRectF(0, 0, GRAPH_WIDTH, GRAPH_HEIGHT));

  // draw bottom tick marks
  if (data_labels != NULL) {
    font.setPointSize(4);
    p.setFont(font);
    for (int i = 0; i < data_count + (data_labels_on_bars ? 0 : 1); i++) {
      p.drawText(
          QRectF(i * BIN_WIDTH - (data_labels_on_bars ? 0 : BIN_WIDTH / 2),
                 GRAPH_HEIGHT, BIN_WIDTH, LABELS_HEIGHT),
          Qt::AlignCenter, data_labels[i].c_str());
    }
    for (int i = 0; i < data_count + 1; i++)
      p.drawLine(QLineF(i * BIN_WIDTH, 0, i * BIN_WIDTH, GRAPH_HEIGHT + 5));
  }

  font.setPointSize(6);
  p.setFont(font);
  p.translate(0, GRAPH_HEIGHT + LABELS_HEIGHT);
  if (xlabel != NULL)
    p.drawText(QRectF(0, 0, GRAPH_WIDTH, LABELS_HEIGHT), Qt::AlignCenter,
               xlabel);

  p.resetTransform();
}

void EkfBagVideo::DrawMahalanobis(QPainter& p, const QRect& rect) {
  float data[MAHAL_NUM_BINS];
  std::string data_labels[MAHAL_NUM_BINS + 1];
  for (int i = 0; i < MAHAL_NUM_BINS; i++) {
    data[i] = static_cast<float>(mahal_bins_[i]);
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << (i * MAHAL_BIN_SIZE);
    data_labels[i] = stream.str();
  }
  data_labels[MAHAL_NUM_BINS] = std::string("\u221E");
  DrawBarGraph(p, rect, MAHAL_NUM_BINS, data, 0.0, 10.0,
               "Visual Landmarks Mahalanobis Distance Histogram", NULL, NULL,
               data_labels, false);
}

void EkfBagVideo::DrawSMFeatureCount(QPainter& p, const QRect& rect) {
  float count = static_cast<float>(ml_count_);
  std::string data_label("Map Features");
  DrawBarGraph(p, rect, 1, &count, 0.0, 50.0, NULL, NULL, NULL, &data_label,
               true);
}

void EkfBagVideo::DrawOFFeatureCount(QPainter& p, const QRect& rect) {
  float count = static_cast<float>(of_count_);
  std::string data_label("VO Features");
  DrawBarGraph(p, rect, 1, &count, 0.0, 50.0, NULL, NULL, NULL, &data_label,
               true);
}

void EkfBagVideo::DrawStatus(QPainter& p, const QRect& rect) {
  QColor color = QColor::fromRgb(0xFF, 0x00, 0x00);
  std::string text("Lost");
  if (state_.confidence == ff_msgs::EkfState::CONFIDENCE_GOOD) {
    color = QColor::fromRgb(0x00, 0xFF, 0x00);
    text = std::string("Good");
  } else if (state_.confidence == ff_msgs::EkfState::CONFIDENCE_POOR) {
    color = QColor::fromRgb(0xFF, 0xFF, 0x00);
    text = std::string("Poor");
  }

  QPen white(QColor::fromRgb(0xFF, 0xFF, 0xFF));
  QPen black(QColor::fromRgb(0xFF, 0xFF, 0xFF));
  white.setWidth(1.0);
  p.setPen(white);
  p.setBrush(QBrush(color));
  QFont font("Arial");
  font.setPointSize(10);
  p.setFont(font);

  p.drawRect(rect);

  p.setPen(black);
  p.drawText(rect, Qt::AlignCenter, text.c_str());
}

void EkfBagVideo::DrawVelocity(QPainter& p, const QRect& rect) {
  std::string data_labels[] = {std::string("x"), std::string("y"),
                               std::string("z")};
  float data[] = {static_cast<float>(state_.velocity.x),
                  static_cast<float>(state_.velocity.y),
                  static_cast<float>(state_.velocity.z)};
  DrawBarGraph(p, rect, 3, data, -0.25, 0.25, "Velocity", NULL, "m/s",
               data_labels, true);
}

void EkfBagVideo::DrawAccelBias(QPainter& p, const QRect& rect) {
  std::string data_labels[] = {std::string("x"), std::string("y"),
                               std::string("z")};
  float data[] = {static_cast<float>(state_.accel_bias.x),
                  static_cast<float>(state_.accel_bias.y),
                  static_cast<float>(state_.accel_bias.z)};
  DrawBarGraph(p, rect, 3, data, -0.01, 0.01, "Accel Bias", NULL, "m/s^2",
               data_labels, true);
}

void EkfBagVideo::DrawOmega(QPainter& p, const QRect& rect) {
  std::string data_labels[] = {std::string("\u03D1"), std::string("\u03C6"),
                               std::string("\u03C8")};
  float data[] = {static_cast<float>(state_.omega.x * 180.0 / M_PI),
                  static_cast<float>(state_.omega.y * 180.0 / M_PI),
                  static_cast<float>(state_.omega.z * 180.0 / M_PI)};
  DrawBarGraph(p, rect, 3, data, -45.0, 45.0, "Angular Velocity", NULL,
               "\u00B0/s", data_labels, true);
}

void EkfBagVideo::DrawGyroBias(QPainter& p, const QRect& rect) {
  std::string data_labels[] = {std::string("\u03D1"), std::string("\u03C6"),
                               std::string("\u03C8")};
  float data[] = {static_cast<float>(state_.gyro_bias.x * 180.0 / M_PI),
                  static_cast<float>(state_.gyro_bias.y * 180.0 / M_PI),
                  static_cast<float>(state_.gyro_bias.z * 180.0 / M_PI)};
  DrawBarGraph(p, rect, 3, data, -0.1, 0.1, "Gyro Bias", NULL, "\u00B0/s",
               data_labels, true);
}

void EkfBagVideo::DrawPositionCov(QPainter& p, const QRect& rect) {
  std::string data_labels[] = {std::string("x"), std::string("y"),
                               std::string("z")};
  float data[] = {static_cast<float>(sqrt(state_.cov_diag[12]) * 100.0),
                  static_cast<float>(sqrt(state_.cov_diag[13]) * 100.0),
                  static_cast<float>(sqrt(state_.cov_diag[14]) * 100.0)};
  DrawBarGraph(p, rect, 3, data, 0.0, 2.0, "Position Std. Dev.", NULL, "cm",
               data_labels, true);
}

void EkfBagVideo::DrawVelocityCov(QPainter& p, const QRect& rect) {
  std::string data_labels[] = {std::string("x"), std::string("y"),
                               std::string("z")};
  float data[] = {static_cast<float>(sqrt(state_.cov_diag[7]) * 100.0),
                  static_cast<float>(sqrt(state_.cov_diag[8]) * 100.0),
                  static_cast<float>(sqrt(state_.cov_diag[9]) * 100.0)};
  DrawBarGraph(p, rect, 3, data, 0.0, 0.2, "Velocity Std. Dev.", NULL, "cm/s",
               data_labels, true);
}

void EkfBagVideo::DrawAngleCov(QPainter& p, const QRect& rect) {
  std::string data_labels[] = {std::string("\u03D1"), std::string("\u03C6"),
                               std::string("\u03C8")};
  float data[] = {static_cast<float>(sqrt(state_.cov_diag[0]) * 180.0 / M_PI),
                  static_cast<float>(sqrt(state_.cov_diag[1]) * 180.0 / M_PI),
                  static_cast<float>(sqrt(state_.cov_diag[2]) * 180.0 / M_PI)};
  DrawBarGraph(p, rect, 3, data, 0.0, 0.1, "Orientation Std. Dev.", NULL,
               "\u00B0", data_labels, true);
}

void EkfBagVideo::DrawTimestamp(QPainter& p, const QRect& rect) {
  QPen white(QColor::fromRgb(0xFF, 0xFF, 0xFF));
  p.setPen(white);
  QFont font("Arial");

  p.translate(rect.x() + 20, rect.y() + 20);
  font.setPointSize(28);
  p.setFont(font);
  std::stringstream ros_ts;
  std::stringstream bag_ts;
  int millis = last_image_time_.nsec / 1E6;
  ros::Duration ellapsed = last_image_time_ - bag_start_time_;
  ros_ts << last_image_time_.sec << "." << std::setfill('0') << std::setw(3)
         << millis;
  int hours = ellapsed.sec / 3600;
  int total_minutes = ellapsed.sec / 60;
  int minutes = total_minutes % 60;
  int seconds = ellapsed.sec % 60;
  bag_ts << std::setfill('0') << std::setw(2) << hours << ":"
         << std::setfill('0') << std::setw(2) << minutes << ":"
         << std::setfill('0') << std::setw(2) << seconds << "." << millis / 100;
  p.drawText(QRectF(0, 0, rect.width(), 60), Qt::AlignLeft,
             ros_ts.str().c_str());
  p.translate(0, 40);
  p.drawText(QRectF(0, 0, rect.width(), 60), Qt::AlignLeft,
             bag_ts.str().c_str());
  p.resetTransform();
}

void EkfBagVideo::UpdateImage(const ros::Time& time,
                              const sensor_msgs::ImageConstPtr& ros_image) {
  EkfBag::UpdateImage(time, ros_image);

  last_image_ = ros_image;
  last_image_time_ = time;
}

void EkfBagVideo::DrawFrame(void) {
  QImage image(1920, 1080, QImage::Format_ARGB32);
  image.fill(QColor::fromRgb(0, 0, 0));
  QPainter p(&image);

  DrawImage(p, last_image_, QRect(0, 0, 1280, 960));

  // draw side widgets
  // starts at pixel 1280, width is 640
  DrawPose(p, QRect(1280, 0, 640, 640));

  DrawVelocity(p, QRect(1280, 640, 320, 110));
  DrawAccelBias(p, QRect(1280, 750, 320, 110));
  DrawPositionCov(p, QRect(1280, 860, 320, 110));
  DrawVelocityCov(p, QRect(1280, 970, 320, 110));

  DrawOmega(p, QRect(1600, 640, 320, 110));
  DrawGyroBias(p, QRect(1600, 750, 320, 110));
  DrawAngleCov(p, QRect(1600, 860, 320, 110));
  // gyro_bias
  // accel
  // accel_bias

  DrawTimestamp(p, QRect(1600, 970, 320, 110));

  // bottom
  DrawStatus(p, QRect(0, 960, 120, 120));
  DrawSMFeatureCount(p, QRect(120, 960, 120, 120));
  DrawOFFeatureCount(p, QRect(240, 960, 120, 120));
  DrawMahalanobis(p, QRect(360, 960, 920, 120));

  p.end();
  video_.AddFrame(image);
}

void EkfBagVideo::UpdateEKF(const ff_msgs::EkfState& s) {
  EkfBag::UpdateEKF(s);

  state_ = s;

  pose_count_++;
  if (pose_count_ == 10) {
    pose_count_ = 0;
    if (pose_history_.size() > 200) pose_history_.pop_front();
    pose_history_.push_back(s.pose);
  }

  tracked_sm_.UpdatePose(s.pose);

  if (state_.ml_count > 0) {
    memset(mahal_bins_, 0, sizeof(int) * MAHAL_NUM_BINS);
    for (unsigned int i = 0; i < state_.ml_mahal_dists.size(); i++) {
      float m = state_.ml_mahal_dists[i];
      if (std::isnan(m)) continue;
      mahal_bins_[std::min(MAHAL_NUM_BINS - 1,
                           static_cast<int>(m / MAHAL_BIN_SIZE))]++;
    }
    ml_count_ = state_.ml_count;
  }

  if (state_.of_count > 0) of_count_ = state_.of_count;

  if (!start_time_set_) {
    start_time_ = s.header.stamp;
    start_time_set_ = true;
  }

  if ((s.header.stamp - last_draw_time_).toSec() >= 1.0 / 15) {
    last_draw_time_ = s.header.stamp;
    DrawFrame();
  }
}

void EkfBagVideo::UpdateOpticalFlow(const ff_msgs::Feature2dArray& of) {
  EkfBag::UpdateOpticalFlow(of);

  if (!start_time_set_) return;
  float t = (of.header.stamp - start_time_).toSec();

  tracked_of_.UpdateFeatures(of, t);
}

void EkfBagVideo::UpdateSparseMap(const ff_msgs::VisualLandmarks& vl) {
  EkfBag::UpdateSparseMap(vl);

  if (!start_time_set_) return;
  float t = (vl.header.stamp - start_time_).toSec();

  tracked_sm_.UpdateFeatures(vl, t);
}

}  // namespace ekf_video
