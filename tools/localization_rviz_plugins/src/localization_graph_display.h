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

// Header file must go in src directory for Qt/Rviz plugin
#ifndef LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_DISPLAY_H_  // NOLINT
#define LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_DISPLAY_H_  // NOLINT

// Required for Qt
#ifndef Q_MOC_RUN
#include <camera/camera_params.h>
#include <ff_msgs/LocalizationGraph.h>
#include <graph_localizer/graph_localizer.h>
#include <graph_localizer/loc_projection_factor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <image_transport/image_transport.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/float_property.h>
#include <opencv2/core/types.hpp>
#include <map>
#include <string>
#include <vector>
#include "slider_property.h"  // NOLINT
#endif

// Forward declarations for ogre and rviz
namespace Ogre {
class SceneNode;
}

namespace localization_rviz_plugins {
// TODO(rsoussan): put these somewhere else!
using Calibration = gtsam::Cal3_S2;
using Camera = gtsam::PinholePose<Calibration>;
using SmartFactor = gtsam::RobustSmartProjectionPoseFactor<Calibration>;

class LocalizationGraphDisplay : public rviz::MessageFilterDisplay<ff_msgs::LocalizationGraph> {
  Q_OBJECT    // NOLINT
    public :  // NOLINT
              LocalizationGraphDisplay();
  ~LocalizationGraphDisplay() = default;

  // private:
 protected:
  void onInitialize() final;
  void reset() final;

 private Q_SLOTS:  // NOLINT
  void addSmartFactorsProjectionVisual();

 private:
  void processMessage(const ff_msgs::LocalizationGraph::ConstPtr& graph_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
  void clearDisplay();
  void addImuVisual(const graph_localizer::GraphLocalizer& graph_localizer,
                    const gtsam::CombinedImuFactor* const imu_factor);
  void addProjectionVisual(const gtsam::CameraSet<Camera>& cameras, const Camera::MeasurementVector& measurements,
                           const gtsam::Point3& world_t_landmark, std::vector<cv::Mat>& images);
  void addLocProjectionVisual(const std::vector<gtsam::LocProjectionFactor<>*> loc_projection_factors,
                              const graph_localizer::GraphValues& graph_values);
  void addOpticalFlowVisual(const graph_localizer::FeatureTrackIdMap& feature_tracks,
                            const localization_common::Time latest_graph_time);
  void clearImageBuffer(const localization_common::Time oldest_graph_time);
  sensor_msgs::ImageConstPtr getImage(const localization_common::Time time);
  void addSmartFactorProjectionVisual(const SmartFactor& smart_factor,
                                      const graph_localizer::GraphValues& graph_values);
  cv::Scalar textColor(const double val, const double green_threshold, const double yellow_threshold);

  std::vector<std::unique_ptr<rviz::Axes>> graph_pose_axes_;
  std::vector<std::unique_ptr<rviz::Arrow>> imu_factor_arrows_;
  std::unique_ptr<rviz::BoolProperty> show_pose_axes_;
  std::unique_ptr<rviz::FloatProperty> pose_axes_size_;
  std::unique_ptr<rviz::BoolProperty> show_imu_factor_arrows_;
  std::unique_ptr<rviz::FloatProperty> imu_factor_arrows_diameter_;
  std::unique_ptr<rviz::BoolProperty> publish_optical_flow_images_;
  std::unique_ptr<rviz::BoolProperty> publish_smart_factor_images_;
  std::unique_ptr<rviz::BoolProperty> publish_loc_projection_factor_images_;
  std::unique_ptr<rviz::BoolProperty> publish_projection_factor_images_;
  std::unique_ptr<rviz::BoolProperty> show_projection_factor_visual_;
  std::unique_ptr<rviz::SliderProperty> projection_factor_slider_;
  image_transport::Publisher optical_flow_image_pub_;
  image_transport::Publisher smart_factor_projection_image_pub_;
  image_transport::Publisher projection_image_pub_;
  image_transport::Publisher loc_projection_factor_image_pub_;
  image_transport::Subscriber image_sub_;
  ros::NodeHandle nh_;
  std::map<localization_common::Time, sensor_msgs::ImageConstPtr> img_buffer_;
  std::unique_ptr<camera::CameraParameters> nav_cam_params_;
  std::vector<std::unique_ptr<rviz::Shape>> landmark_points_;
  std::vector<std::unique_ptr<rviz::Axes>> camera_pose_axes_;
  std::vector<std::unique_ptr<rviz::Line>> camera_t_landmark_lines_;
  std::unique_ptr<graph_localizer::GraphLocalizer> latest_graph_localizer_;
  std::vector<SmartFactor*> latest_smart_factors_;
};
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_DISPLAY_H_ NOLINT
