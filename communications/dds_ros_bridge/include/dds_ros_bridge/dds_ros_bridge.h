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

#include <config_reader/config_reader.h>
#include <dds_ros_bridge/rapid_pub.h>
#include <dds_ros_bridge/rapid_sub_ros_pub.h>
#include <dds_ros_bridge/ros_sub_rapid_pub.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

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
  int BuildAccessControlStateToRapid(const std::string& subTopic,
                                     const std::string& pubTopic,
                                     const std::string& name);
  int BuildAckToRapid(const std::string& subTopic,
                      const std::string& pubTopic,
                      const std::string& name);
  int BuildAgentStateToRapid(const std::string& subTopic,
                             const std::string& pubTopic,
                             const std::string& name);
  int BuildArmJointSampleToRapid(const std::string& subTopic,
                                 const std::string& pubTopic,
                                 const std::string& name);
  int BuildArmStateToRapid(const std::string& subTopic,
                           const std::string& pubTopic,
                           const std::string& name);
  int BuildBatteryStateToRapid(const std::string& subTopicBatteryStateTL,
                               const std::string& subTopicBatteryStateTR,
                               const std::string& subTopicBatteryStateBL,
                               const std::string& subTopicBatteryStateBR,
                               const std::string& subTopicBatteryTempTL,
                               const std::string& subTopicBatteryTempTR,
                               const std::string& subTopicBatteryTempBL,
                               const std::string& subTopicBatteryTempBR,
                               const std::string& pubTopic,
                               const std::string& name);
  int BuildCompressedImageToImage(const std::string& subTopic,
                                  const std::string& pubTopic,
                                  const std::string& name);
  int BuildCompressedFileToRapid(const std::string& subTopic,
                                  const std::string& pubTopic,
                                  const std::string& name);
  int BuildCompressedFileAckToRapid(const std::string& subTopic,
                                    const std::string& pubTopic,
                                    const std::string& name);
  int BuildCpuStateToRapid(const std::string& subTopic,
                           const std::string& pubTopic,
                           const std::string& name);
  int BuildDiskStateToRapid(const std::string& subTopic,
                            const std::string& pubTopic,
                            const std::string& name);
  int BuildFaultConfigToRapid(const std::string& subTopic,
                              const std::string& pubTopic,
                              const std::string& name);
  int BuildFaultStateToRapid(const std::string& subTopic,
                             const std::string& pubTopic,
                             const std::string& name);
  int BuildGncFamCmdStateToRapid(const std::string& subTopic,
                                 const std::string& pubTopic,
                                 const std::string& name);
  int BuildGncControlStateToRapid(const std::string& subTopic,
                                  const std::string& pubTopic,
                                  const std::string& name);
  int BuildGuestScienceToRapid(const std::string& stateSubTopic,
                               const std::string& configSubTopic,
                               const std::string& dataSubTopic,
                               const std::string& pubTopic,
                               const std::string& name);
  int BuildOdomToPosition(const std::string& subTopic,
                          const std::string& pubTopic,
                          const std::string& name);
  int BuildPlanStatusToPlanStatus(const std::string& subTopic,
                                  const std::string& pubTopic,
                                  const std::string& name);
  int BuildStringToTextMessage(const std::string& subTopic,
                               const std::string& pubTopic,
                               const std::string& name);
  int BuildTelemetryToRapid(const std::string& subTopic,
                            const std::string& pubTopic,
                            const std::string& name);

  /**
   * Build Rapid subscribers to Ros publishers
   * @param subscribeTopic topix suffix for RAPID ex : "-debug"
   * @param pubTopic topic to publish to
   *
   * @return number of RapidPubRosSubs
   */
  int BuildCommandToCommand(const std::string& subTopic,
                            const std::string& pubTopic,
                            const std::string& name);
  int BuildCompressedFileToCompressedFile(const std::string& subTopic,
                                          const std::string& pubTopic,
                                          const std::string& name);


  /**
   * Build Rapid publisher for one time message
   */
  int BuildCommandConfigToCommandConfig(const std::string& pubTopic,
                                        const std::string& name);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();

 private:
  config_reader::ConfigReader config_params_;

  int components_;

  ros::NodeHandle nh_;

  std::map<std::string, ff::RosSubRapidPubPtr> m_rosSubRapidPubs_;
  std::shared_ptr<kn::DdsEntitiesFactorySvc> m_ddsEntitiesFactory_;
  std::string agent_name_, participant_name_;
  std::vector<ff::RapidPubPtr> m_rapidPubs_;
  std::vector<ff::RapidSubRosPubPtr> m_rapidSubRosPubs_;
};

}  // end namespace dds_ros_bridge

#endif  // DDS_ROS_BRIDGE_DDS_ROS_BRIDGE_H_
