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

#ifndef FAM_FAM_H_
#define FAM_FAM_H_

#include <ff_common/ff_ros.h>

#include <ff_msgs/msg/fam_command.hpp>
#include <ff_msgs/msg/flight_mode.hpp>
#include <ff_hw_msgs/msg/pmc_command.hpp>

#include <ff_common/ff_names.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/inertia_stamped.hpp>

#include <config_reader/config_reader.h>
#include <pmc/fam.h>

#include <mutex>

namespace fam {

/**
* @brief Force Allocation Module implementation using GNC module
*/
class Fam {
 public:
  explicit Fam(NodeHandle & nh);
  ~Fam();
  void Step();

 protected:
  void ReadParams(void);
  void CtlCallBack(const std::shared_ptr<ff_msgs::msg::FamCommand> c);
  void FlightModeCallback(const std::shared_ptr<ff_msgs::msg::FlightMode> mode);
  void InertiaCallback(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> inertia);

  pmc::Fam fam_;

  rclcpp::Subscription<ff_msgs::msg::FamCommand>::SharedPtr ctl_sub_;
  rclcpp::Subscription<ff_msgs::msg::FlightMode>::SharedPtr flight_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::InertiaStamped>::SharedPtr inertia_sub_;
  rclcpp::Publisher<ff_hw_msgs::msg::PmcCommand>::SharedPtr pmc_pub_;
  rclcpp::Clock::SharedPtr clock_;

  // config_reader::ConfigReader config_;
  // ros::Timer config_timer_;

  std::mutex mutex_speed_;
  std::mutex mutex_mass_;
  Eigen::Vector3f center_of_mass_;
  bool inertia_received_;

  pmc::FamInput input_;
};
}  // end namespace fam

#endif  // FAM_FAM_H_

