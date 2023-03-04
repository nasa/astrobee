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
#ifndef GROUND_TRUTH_LOCALIZER_GROUND_TRUTH_LOCALIZER_NODELET_H_
#define GROUND_TRUTH_LOCALIZER_GROUND_TRUTH_LOCALIZER_NODELET_H_

#include <ff_util/ff_component.h>
#include <ground_truth_localizer/twist.h>
#include <localization_common/time.h>

#include <ff_msgs/msg/heartbeat.hpp>
#include <ff_msgs/srv/set_ekf_input.hpp>
#include <ff_msgs/msg/ekf_state.hpp>
namespace ff_msgs {
typedef msg::Heartbeat Heartbeat;
typedef srv::SetEkfInput SetEkfInput;
typedef msg::EkfState EkfState;
}  // namespace ff_msgs

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
namespace geometry_msgs {
typedef msg::PoseStamped PoseStamped;
typedef msg::TwistStamped TwistStamped;
}  // namespace geometry_msgs

#include <std_msgs/msg/empty.hpp>
namespace std_msgs {
typedef msg::Empty Empty;
}  // namespace std_msgs
#include <std_srvs/srv/empty.hpp>
namespace std_srvs {
typedef srv::Empty Empty;
}  // namespace std_srvs

#include <tf2_ros/transform_broadcaster.h>

#include <boost/optional.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace ground_truth_localizer {
class GroundTruthLocalizerNodelet : public ff_util::FreeFlyerComponent {
 public:
  explicit GroundTruthLocalizerNodelet(const rclcpp::NodeOptions& options);

 private:
  void Initialize(NodeHandle &nh);

  void SubscribeAndAdvertise(NodeHandle &nh);

  bool SetMode(const std::shared_ptr<ff_msgs::SetEkfInput::Request> req,
               std::shared_ptr<ff_msgs::SetEkfInput::Response> res);

  bool DefaultServiceResponse(const std::shared_ptr<std_srvs::Empty::Request> req,
                              std::shared_ptr<std_srvs::Empty::Response> res);

  void PoseCallback(const std::shared_ptr<geometry_msgs::PoseStamped> pose);

  void TwistCallback(const std::shared_ptr<geometry_msgs::TwistStamped> twist);

  void PublishLocState(const localization_common::Time& timestamp);

  NodeHandle node_;
  std::string platform_name_;
  rclcpp::Time last_time_;
  boost::optional<Eigen::Isometry3d> pose_;
  boost::optional<Twist> twist_;
  int input_mode_ = ff_msgs::SetEkfInput::Request::MODE_TRUTH;
  rclcpp::Subscription<geometry_msgs::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<ff_msgs::EkfState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<ff_msgs::Heartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::Empty>::SharedPtr reset_pub_;
  ff_msgs::Heartbeat heartbeat_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_pub_;
  rclcpp::Service<ff_msgs::SetEkfInput>::SharedPtr input_mode_srv_;
  rclcpp::Service<std_srvs::Empty>::SharedPtr bias_srv_;
  rclcpp::Service<std_srvs::Empty>::SharedPtr bias_from_file_srv_;
  rclcpp::Service<std_srvs::Empty>::SharedPtr reset_srv_;
};
}  // namespace ground_truth_localizer

#endif  // GROUND_TRUTH_LOCALIZER_GROUND_TRUTH_LOCALIZER_NODELET_H_
