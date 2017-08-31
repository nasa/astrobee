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

#ifndef GNC_ROS_WRAPPER_GNC_H_
#define GNC_ROS_WRAPPER_GNC_H_

#include <gnc_ros_wrapper/ekf_wrapper.h>
#include <gnc_ros_wrapper/ctl.h>
#include <gnc_ros_wrapper/fam.h>

#include <config_reader/config_reader.h>
#include <ff_util/perf_timer.h>

#include <string>

namespace gnc_ros_wrapper {

/**
 * Calls all GNC functions and publishes ros messages.
 */
class GNC {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit GNC(ros::NodeHandle* nh, std::string const& platform_name,
    std::string const& name);
  ~GNC();

  /**
   * Continually run everything.
   **/
  void Run();

 protected:
  void ReadParams(void);

  cmc_msg cmc_;       // The cmc is shared between EKF and CTL

  EkfWrapper ekf_;
  Ctl ctl_;
  Fam fam_;

  config_reader::ConfigReader config_;

  ff_util::PerfTimer pt_ekf_;
  ff_util::PerfTimer pt_ctl_;
  ff_util::PerfTimer pt_fam_;

  ros::Timer config_timer_;
};
}  // end namespace gnc_ros_wrapper

#endif  // GNC_ROS_WRAPPER_GNC_H_
