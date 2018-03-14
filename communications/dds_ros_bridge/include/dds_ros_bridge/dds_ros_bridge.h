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

#ifndef DDS_ROS_BRIDGE_DDS_ROS_BRIDGE_H_
#define DDS_ROS_BRIDGE_DDS_ROS_BRIDGE_H_

#include <pluginlib/class_list_macros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "config_reader/config_reader.h"

#include "dds_ros_bridge/rapid_command_ros_command_plan.h"
#include "dds_ros_bridge/rapid_compressed_file_ros_compressed_file.h"
#include "dds_ros_bridge/rapid_pub.h"
#include "dds_ros_bridge/rapid_sub_ros_pub.h"
#include "dds_ros_bridge/ros_access_control.h"
#include "dds_ros_bridge/ros_ack.h"
#include "dds_ros_bridge/ros_agent_state.h"
#include "dds_ros_bridge/ros_arm_joint_sample.h"
#include "dds_ros_bridge/ros_arm_state.h"
#include "dds_ros_bridge/ros_battery_state.h"
#include "dds_ros_bridge/ros_command_config_rapid_command_config.h"
#include "dds_ros_bridge/ros_compressed_file_ack.h"
#include "dds_ros_bridge/ros_compressed_file_rapid_compressed_file.h"
#include "dds_ros_bridge/ros_compressed_image_rapid_image.h"
#include "dds_ros_bridge/ros_cpu_state.h"
#include "dds_ros_bridge/ros_data_to_disk.h"
#include "dds_ros_bridge/ros_disk_state.h"
#include "dds_ros_bridge/ros_fault_config.h"
#include "dds_ros_bridge/ros_fault_state.h"
#include "dds_ros_bridge/ros_gnc_fam_cmd_state.h"
#include "dds_ros_bridge/ros_gnc_control_state.h"
#include "dds_ros_bridge/ros_guest_science.h"
#include "dds_ros_bridge/ros_odom_rapid_position.h"
#include "dds_ros_bridge/ros_plan_status_rapid_plan_status.h"
#include "dds_ros_bridge/ros_string_rapid_text_message.h"
#include "dds_ros_bridge/ros_sub_rapid_pub.h"
#include "dds_ros_bridge/ros_telemetry_rapid_telemetry.h"

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

class DdsRosBridge : public ff_util::FreeFlyerNodelet {
 public:
  DdsRosBridge();
  ~DdsRosBridge();

  /**
   * Build Ros subscribers to Rapid publishers
   * @param  subTopic       topic to subscribe to  ex: "/camera/image"
   * @param  pubTopic       topic suffix for RAPID ex: "-debug"
   * @return                number of RosSubRapidPubs
   */
  int BuildAccessControlStateToRapid(const std::string& sub_topic,
                                     const std::string& pub_topic,
                                     const std::string& name);
  int BuildAckToRapid(const std::string& sub_topic,
                      const std::string& pub_topic,
                      const std::string& name);
  int BuildAgentStateToRapid(const std::string& sub_topic,
                             const std::string& pub_topic,
                             const std::string& name);
  int BuildArmJointSampleToRapid(const std::string& sub_topic,
                                 const std::string& pub_topic,
                                 const std::string& name);
  int BuildArmStateToRapid(const std::string& sub_topic,
                           const std::string& pub_topic,
                           const std::string& name);
  int BuildBatteryStateToRapid(const std::string& sub_topic_battery_state_TL,
                               const std::string& sub_topic_battery_state_TR,
                               const std::string& sub_topic_battery_state_BL,
                               const std::string& sub_topic_battery_state_BR,
                               const std::string& sub_topic_battery_temp_TL,
                               const std::string& sub_topic_battery_temp_TR,
                               const std::string& sub_topic_battery_temp_BL,
                               const std::string& sub_topic_battery_temp_BR,
                               const std::string& pub_topic,
                               const std::string& name);
  int BuildCompressedImageToImage(const std::string& sub_topic,
                                  const std::string& pub_topic,
                                  const std::string& name);
  int BuildCompressedFileToRapid(const std::string& sub_topic,
                                  const std::string& pub_topic,
                                  const std::string& name);
  int BuildCompressedFileAckToRapid(const std::string& sub_topic,
                                    const std::string& pub_topic,
                                    const std::string& name);
  int BuildCpuStateToRapid(const std::string& sub_topic,
                           const std::string& pub_topic,
                           const std::string& name);
  int BuildDataToDiskStateToRapid(const std::string& state_sub_topic,
                                  const std::string& topics_sub_topic,
                                  const std::string& pub_topic,
                                  const std::string& name);
  int BuildDiskStateToRapid(const std::string& sub_topic,
                            const std::string& pub_topic,
                            const std::string& name);
  int BuildFaultConfigToRapid(const std::string& sub_topic,
                              const std::string& pub_topic,
                              const std::string& name);
  int BuildFaultStateToRapid(const std::string& sub_topic,
                             const std::string& pub_topic,
                             const std::string& name);
  int BuildGncFamCmdStateToRapid(const std::string& sub_topic,
                                 const std::string& pub_topic,
                                 const std::string& name);
  int BuildGncControlStateToRapid(const std::string& sub_topic,
                                  const std::string& pub_topic,
                                  const std::string& name);
  int BuildGuestScienceToRapid(const std::string& state_sub_topic,
                               const std::string& config_sub_topic,
                               const std::string& data_sub_topic,
                               const std::string& pub_topic,
                               const std::string& name);
  int BuildOdomToPosition(const std::string& sub_topic,
                          const std::string& pub_topic,
                          const std::string& name);
  int BuildPlanStatusToPlanStatus(const std::string& sub_topic,
                                  const std::string& pub_topic,
                                  const std::string& name);
  int BuildStringToTextMessage(const std::string& sub_topic,
                               const std::string& pub_topic,
                               const std::string& name);
  int BuildTelemetryToRapid(const std::string& sub_topic,
                            const std::string& pub_topic,
                            const std::string& name);

  /**
   * Build Rapid subscribers to Ros publishers
   * @param subscribeTopic topix suffix for RAPID ex : "-debug"
   * @param pubTopic topic to publish to
   *
   * @return number of RapidPubRosSubs
   */
  int BuildCommandToCommand(const std::string& sub_topic,
                            const std::string& pub_topic,
                            const std::string& name);
  int BuildCompressedFileToCompressedFile(const std::string& sub_topic,
                                          const std::string& pub_topic,
                                          const std::string& name);


  /**
   * Build Rapid publisher for one time message
   */
  int BuildCommandConfigToCommandConfig(const std::string& pub_topic,
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

#endif  // DDS_ROS_BRIDGE_DDS_ROS_BRIDGE_H_
