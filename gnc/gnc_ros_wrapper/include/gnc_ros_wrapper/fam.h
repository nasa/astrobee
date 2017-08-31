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

#ifndef GNC_ROS_WRAPPER_FAM_H_
#define GNC_ROS_WRAPPER_FAM_H_

#include <gnc_autocode/fam.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <ff_msgs/FamCommand.h>

#include <config_reader/config_reader.h>

namespace gnc_ros_wrapper {

/**
* @brief Force Allocation Module implementation using GNC module
*/
class Fam {
 public:
  explicit Fam(ros::NodeHandle* nh);
  ~Fam();

  void Step(ex_time_msg* ex_time, cmd_msg* cmd, ctl_msg* ctl);
  void ReadParams(config_reader::ConfigReader* config);
  act_msg* GetActMsg(void) {return &gnc_.act_;}
 protected:
  gnc_autocode::GncFamAutocode gnc_;
  ros::Publisher pmc_pub_;
};
}  // end namespace gnc_ros_wrapper

#endif  // GNC_ROS_WRAPPER_FAM_H_
