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

#ifndef DDS_ROS_BRIDGE_ROS_TELEMETRY_RAPID_TELEMETRY_H_
#define DDS_ROS_BRIDGE_ROS_TELEMETRY_RAPID_TELEMETRY_H_

#include <string>

#include "config_reader/config_reader.h"

#include "dds_ros_bridge/ros_sub_rapid_pub.h"

#include "ff_msgs/CameraState.h"
#include "ff_msgs/CameraStatesStamped.h"

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "ros/ros.h"

#include "AstrobeeConstants.h"
#include "TelemetryConfigSupport.h"
#include "TelemetryStateSupport.h"

namespace ff {

class RosTelemetryRapidTelemetry : public RosSubRapidPub {
 public:
  RosTelemetryRapidTelemetry(const std::string& subscribe_topic,
                             const std::string& pub_topic,
                             const ros::NodeHandle &nh,
                             config_reader::ConfigReader& config_params,
                             unsigned int queue_size = 10);

  void CameraStateCallback(ff_msgs::CameraStatesStampedConstPtr const& state);
  void SetCommStatusRate(float rate);
  void SetCpuStateRate(float rate);
  void SetDiskStateRate(float rate);
  void SetEkfStateRate(float rate);
  void SetGncStateRate(float rate);
  void SetPositionRate(float rate);

 protected:
  bool AssembleConfig(config_reader::ConfigReader& config_params);

  rapid::ext::astrobee::CameraResolution ConvertResolution(std::string const&
                                                                    resolution);
  using ConfigSupplier =
                    kn::DdsTypedSupplier<rapid::ext::astrobee::TelemetryConfig>;
  using ConfigSupplierPtr = std::unique_ptr<ConfigSupplier>;

  using StateSupplier =
                    kn::DdsTypedSupplier<rapid::ext::astrobee::TelemetryState>;
  using StateSupplierPtr = std::unique_ptr<StateSupplier>;

  ConfigSupplierPtr config_supplier_;
  StateSupplierPtr state_supplier_;
};

}  // end namespace ff

#endif  // DDS_ROS_BRIDGE_ROS_TELEMETRY_RAPID_TELEMETRY_H_
