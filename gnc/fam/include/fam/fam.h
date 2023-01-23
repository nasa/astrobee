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

#include <gnc_autocode/fam.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <ff_msgs/FamCommand.h>
#include <ff_msgs/FlightMode.h>

#include <ff_util/ff_names.h>
#include <ff_util/perf_timer.h>

#include <Eigen/Dense>
#include <geometry_msgs/InertiaStamped.h>

#include <config_reader/config_reader.h>
#include <pmc/fam.h>

#include <mutex>

namespace fam {

/**
* @brief Force Allocation Module implementation using GNC module
*/
class Fam {
 public:
  explicit Fam(ros::NodeHandle* nh);
  ~Fam();
  void Step(ex_time_msg* ex_time, cmd_msg* cmd, ctl_msg* ctl);

 protected:
  void ReadParams(void);
  void CtlCallBack(const ff_msgs::FamCommand & c);
  void FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode);
  void InertiaCallback(const geometry_msgs::InertiaStamped::ConstPtr& inertia);
  void TestTwoVectors(const char* name, const Eigen::Matrix<float, 6, 1> new_array,
                      const float old_array[], float tolerance);

  gnc_autocode::GncFamAutocode gnc_;
  pmc::Fam fam_;

  ros::Subscriber ctl_sub_;
  ros::Subscriber flight_mode_sub_, inertia_sub_;
  ros::Publisher pmc_pub_;

  config_reader::ConfigReader config_;
  ff_util::PerfTimer pt_fam_;
  ros::Timer config_timer_;

  std::mutex mutex_speed_;
  uint8_t speed_;
  std::mutex mutex_mass_;
  Eigen::Vector3f center_of_mass_;
  bool inertia_received_;
  bool use_old_fam_;
  bool compare_fam_;

  pmc::FamInput input_;
};
}  // end namespace fam

#endif  // FAM_FAM_H_

