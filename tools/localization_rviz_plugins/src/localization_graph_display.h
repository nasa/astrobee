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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <image_transport/image_transport.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <opencv2/core/types.hpp>
#include <map>
#include <string>
#include <vector>
#endif

// Forward declarations for ogre and rviz
namespace Ogre {
class SceneNode;
}

namespace localization_rviz_plugins {
// TODO(rsoussan): put these somewhere else!
using Calibration = gtsam::Cal3_S2;
using Camera = gtsam::PinholeCamera<Calibration>;
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

 private:
  void processMessage(const ff_msgs::LocalizationGraph::ConstPtr& graph_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
  void clearDisplay();
  void addImuVisual(const graph_localizer::GraphLocalizer& graph_localizer,
                    const gtsam::CombinedImuFactor* const imu_factor);
  void addOpticalFlowVisual(const graph_localizer::FeatureTrackMap& feature_tracks,
                            const localization_common::Time latest_graph_time);
  void clearImageBuffer(const localization_common::Time oldest_graph_time);
  sensor_msgs::ImageConstPtr getImage(const localization_common::Time time);
  void addSmartFactorProjectionVisual(const SmartFactor& smart_factor,
                                      const graph_localizer::GraphValues& graph_values);
  cv::Scalar textColor(const double val, const double green_threshold, const double yellow_threshold);
  void loadConfigs(const std::string& world, const std::string& robot_name, const std::string& config_path);
  void reloadConfigsIfNecessary();

  std::vector<std::unique_ptr<rviz::Axes>> graph_pose_axes_;
  std::vector<std::unique_ptr<rviz::Arrow>> imu_factor_arrows_;
  std::unique_ptr<rviz::BoolProperty> show_pose_axes_;
  std::unique_ptr<rviz::FloatProperty> pose_axes_size_;
  std::unique_ptr<rviz::BoolProperty> show_imu_factor_arrows_;
  std::unique_ptr<rviz::FloatProperty> imu_factor_arrows_diameter_;
  std::unique_ptr<rviz::EditableEnumProperty> selected_world_;
  std::unique_ptr<rviz::EditableEnumProperty> selected_robot_name_;
  std::string world_;
  std::string robot_name_;
  std::string config_path_;
  image_transport::Publisher optical_flow_image_pub_;
  image_transport::Publisher smart_factor_projection_image_pub_;
  image_transport::Subscriber image_sub_;
  ros::NodeHandle nh_;
  std::map<localization_common::Time, sensor_msgs::ImageConstPtr> img_buffer_;
  std::unique_ptr<camera::CameraParameters> nav_cam_params_;
};
}  // namespace localization_rviz_plugins
#endif  // LOCALIZATION_RVIZ_PLUGINS_LOCALIZATION_GRAPH_DISPLAY_H_ NOLINT
