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

#ifndef GROUND_DDS_ROS_BRIDGE_GROUND_DDS_ROS_BRIDGE_H_
#define GROUND_DDS_ROS_BRIDGE_GROUND_DDS_ROS_BRIDGE_H_

#include <pluginlib/class_list_macros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "config_reader/config_reader.h"

#include "ground_dds_ros_bridge/rapid_image_ros_compressed_image.h"
#include "ground_dds_ros_bridge/rapid_sub_ros_pub.h"
#include "ground_dds_ros_bridge/ros_sub_rapid_pub.h"

#include "ff_util/ff_names.h"

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

namespace ground_dds_ros_bridge {

class RosSubRapidPub;

class GroundDdsRosBridge {
 public:
  GroundDdsRosBridge();
  ~GroundDdsRosBridge();

  /**
   * Build Ros subscribers to Rapid publishers
   * @param  subTopic       topic to subscribe to  ex: "/camera/image"
   * @return                successful
   */

  /**
   * Build Rapid subscribers to Ros publishers
   * @param pubTopic topic to publish to
   *
   * @return successful
   */
  bool BuildSensorImageToCompressedImage(const std::string& pub_topic,
                                         const std::string& name);

  bool Initialize(ros::NodeHandle *nh);

 protected:
  bool ReadTopicInfo(const std::string& topic_abbr,
                    const std::string& sub_or_pub,
                    std::string& sub_topic,
                    bool& use);
  bool ReadParams();

 private:
  config_reader::ConfigReader config_params_;

  int components_, domain_id_;

  ros::NodeHandle nh_;

  std::vector<ff::RapidSubRosPubPtr> rapid_sub_ros_pubs_;
  std::vector<ff::RosSubRapidPubPtr> ros_sub_rapid_pubs_;
  std::shared_ptr<kn::DdsEntitiesFactorySvc> dds_entities_factory_;
  std::string participant_name_, agent_name_, connecting_robot_;
};

}  // end namespace ground_dds_ros_bridge

#endif  // GROUND_DDS_ROS_BRIDGE_GROUND_DDS_ROS_BRIDGE_H_
