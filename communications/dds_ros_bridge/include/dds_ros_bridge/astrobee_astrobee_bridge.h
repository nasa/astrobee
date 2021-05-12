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

#ifndef DDS_ROS_BRIDGE_ASTROBEE_ASTROBEE_BRIDGE_H_
#define DDS_ROS_BRIDGE_ASTROBEE_ASTROBEE_BRIDGE_H_

#include <pluginlib/class_list_macros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "config_reader/config_reader.h"

#include "dds_ros_bridge/rapid_pub.h"
#include "dds_ros_bridge/rapid_sub_ros_pub.h"
#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/ros_ekf_rapid_ekf.h"
#include "dds_ros_bridge/rapid_ekf_ros_ekf.h"

#include "ff_util/ff_names.h"
#include "ff_util/ff_nodelet.h"

// SoraCore Includes
#include "knDds/DdsSupport.h"
#include "knDds/DdsEntitiesFactory.h"
#include "knDds/DdsEntitiesFactorySvc.h"

// miro includes
#include "miro/Configuration.h"
#include "miro/Robot.h"
#include "miro/Log.h"

namespace kn {
  class DdsEntitiesFactorySvc;
}  // end namespace kn

namespace dds_ros_bridge {

class RosSubRapidPub;

class AstrobeeAstrobeeBridge : public ff_util::FreeFlyerNodelet {
 public:
  AstrobeeAstrobeeBridge();
  ~AstrobeeAstrobeeBridge();

  /**
   * Build Ros subscribers to Rapid publishers
   * @param  subTopic       topic to subscribe to  ex: "/camera/image"
   * @param  pubTopic       topic suffix for RAPID ex: "-debug"
   * @return                number of RosSubRapidPubs
   */
  int BuildEkfToRapid(const std::string& sub_topic,
                          const std::string& pub_topic,
                          const std::string& rapid_pub_name,
                          const std::string& name);
  /**
   * Build Rapid subscribers to Ros publishers
   * @param subscribeTopic topix suffix for RAPID ex : "-debug"
   * @param pubTopic topic to publish to
   *
   * @return number of RapidPubRosSubs
   */
  int BuildEkfToRos(const std::string& sub_topic,
                    const std::string& sub_partition,
                    const std::string& pub_topic,
                    const std::string& name);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();

 private:
  config_reader::ConfigReader config_params_;

  int components_;

  ros::NodeHandle nh_;

  std::map<std::string, ff::RosSubRapidPubPtr> ros_sub_rapid_pubs_;
  std::shared_ptr<kn::DdsEntitiesFactorySvc> dds_entities_factory_;
  std::string agent_name_, participant_name_;
  std::vector<ff::RapidPubPtr> rapid_pubs_;
  std::vector<ff::RapidSubRosPubPtr> rapid_sub_ros_pubs_;
};

}  // end namespace dds_ros_bridge

#endif  // DDS_ROS_BRIDGE_ASTROBEE_ASTROBEE_BRIDGE_H_
