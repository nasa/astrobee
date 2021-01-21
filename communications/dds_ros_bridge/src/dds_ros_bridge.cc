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

#include "dds_ros_bridge/dds_ros_bridge.h"

namespace {

typedef std::vector<kn::DdsNodeParameters> NodeVector;

void SubstituteROBOT_NAME(kn::DdsEntitiesFactorySvcParameters * params) {
  {
    NodeVector::iterator first, last = params->publishers.end();
    for (first = params->publishers.begin(); first != last; ++first) {
      if (first->name == "<ROBOTNAME>") {
        first->name = Miro::RobotParameters::instance()->name.c_str();
      }
      if (first->partition == "<ROBOTNAME>") {
        first->partition = Miro::RobotParameters::instance()->name.c_str();
      }
    }
  }

  {
    NodeVector::iterator first, last = params->subscribers.end();
    for (first = params->subscribers.begin(); first != last; ++first) {
      if (first->name == "<ROBOTNAME>") {
        first->name = Miro::RobotParameters::instance()->name.c_str();
      }
      if (first->partition == "<ROBOTNAME>") {
        first->partition = Miro::RobotParameters::instance()->name.c_str();
      }
    }
  }
}

}  // namespace

namespace dds_ros_bridge {
DdsRosBridge::DdsRosBridge() :
  ff_util::FreeFlyerNodelet(NODE_DDS_ROS_BRIDGE, true),
  components_(0),
  agent_name_("Bumble") {
}

DdsRosBridge::~DdsRosBridge() {
}

bool DdsRosBridge::BuildAccessControlStateToRapid(const std::string& sub_topic,
                                                  const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr acs_to_acs(
                                new ff::RosAccessControlStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = acs_to_acs;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildAckToRapid(const std::string& sub_topic,
                                   const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr ack_to_ack(new ff::RosAckToRapid(sub_topic,
                                                             pub_topic,
                                                             nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = ack_to_ack;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildAgentStateToRapid(const std::string& sub_topic,
                                          const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr agent_to_agent(
                                        new ff::RosAgentStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = agent_to_agent;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildArmJointSampleToRapid(const std::string& sub_topic,
                                              const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr joint_to_joint(
                                    new ff::RosArmJointSampleToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = joint_to_joint;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildArmStateToRapid(const std::string& sub_topic,
                                        const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr arm_to_arm(new ff::RosArmStateToRapid(sub_topic,
                                                                  pub_topic,
                                                                  nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = arm_to_arm;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildBatteryStateToRapid(
                                  const std::string& sub_topic_battery_state_TL,
                                  const std::string& sub_topic_battery_state_TR,
                                  const std::string& sub_topic_battery_state_BL,
                                  const std::string& sub_topic_battery_state_BR,
                                  const std::string& sub_topic_battery_temp_TL,
                                  const std::string& sub_topic_battery_temp_TR,
                                  const std::string& sub_topic_battery_temp_BL,
                                  const std::string& sub_topic_battery_temp_BR,
                                  const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr battery_to_battery(
                      new ff::RosBatteryStateToRapid(sub_topic_battery_state_TL,
                                                     sub_topic_battery_state_TR,
                                                     sub_topic_battery_state_BL,
                                                     sub_topic_battery_state_BR,
                                                     sub_topic_battery_temp_TL,
                                                     sub_topic_battery_temp_TR,
                                                     sub_topic_battery_temp_BL,
                                                     sub_topic_battery_temp_BR,
                                                     pub_topic,
                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = battery_to_battery;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCommandToRapid(const std::string& sub_topic,
                                       const std::string& ac_sub_topic,
                                       const std::string& failed_cmd_sub_topic,
                                       const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr command_to_command(
                                new ff::RosCommandToRapid(sub_topic,
                                                          ac_sub_topic,
                                                          failed_cmd_sub_topic,
                                                          pub_topic,
                                                          nh_,
                                                          agent_name_));
      components_++;
      ros_sub_rapid_pubs_[name] = command_to_command;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCompressedFileToRapid(const std::string& sub_topic,
                                              const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr compressed_file_to_rapid(
                                    new ff::RosCompressedFileToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = compressed_file_to_rapid;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCompressedFileAckToRapid(const std::string& sub_topic,
                                                 const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr compressed_file_ack_to_rapid(
                                  new ff::RosCompressedFileAckToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = compressed_file_ack_to_rapid;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCompressedImageToImage(const std::string& sub_topic,
                                               const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr compressed_image_to_image(
                                new ff::RosCompressedImageRapidImage(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = compressed_image_to_image;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCpuStateToRapid(const std::string& sub_topic,
                                        const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr cpu_to_cpu(new ff::RosCpuStateToRapid(sub_topic,
                                                                  pub_topic,
                                                                  nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = cpu_to_cpu;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildDataToDiskStateToRapid(
                                            const std::string& state_sub_topic,
                                            const std::string& topics_sub_topic,
                                            const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr data_to_disk_to_data_to_disk(
                                  new ff::RosDataToDiskToRapid(state_sub_topic,
                                                               topics_sub_topic,
                                                               pub_topic,
                                                               nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = data_to_disk_to_data_to_disk;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildDiskStateToRapid(const std::string& sub_topic,
                                         const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr disk_to_disk(new ff::RosDiskStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = disk_to_disk;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildFaultConfigToRapid(const std::string& sub_topic,
                                           const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr fault_to_fault(
                                        new ff::RosFaultConfigToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = fault_to_fault;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildFaultStateToRapid(const std::string& sub_topic,
                                          const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr fault_to_fault(
                                        new ff::RosFaultStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
    components_++;
    ros_sub_rapid_pubs_[name] = fault_to_fault;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildGncFamCmdStateToRapid(const std::string& sub_topic,
                                              const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr gnc_fam_cmd_to_gnc_fam_cmd(
                                    new ff::RosGncFamCmdStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = gnc_fam_cmd_to_gnc_fam_cmd;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildGncControlStateToRapid(const std::string& sub_topic,
                                               const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr gnc_control_to_gnc_control(
                                    new ff::RosGncControlStateToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = gnc_control_to_gnc_control;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildGuestScienceToRapid(const std::string& state_sub_topic,
                                            const std::string& config_sub_topic,
                                            const std::string& data_sub_topic,
                                            const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr guest_science_to_guest_science(
                                new ff::RosGuestScienceToRapid(state_sub_topic,
                                                               config_sub_topic,
                                                               data_sub_topic,
                                                               pub_topic,
                                                               nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = guest_science_to_guest_science;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildInertiaDataToRapid(const std::string& sub_topic,
                                           const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr inertia_to_inertia(
                                            new ff::RosInertiaToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = inertia_to_inertia;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildLogSampleToRapid(const std::string& sub_topic,
                                         const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr log_to_log(new ff::RosLogSampleToRapid(sub_topic,
                                                                   pub_topic,
                                                                   nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = log_to_log;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildOdomToPosition(const std::string& sub_topic,
                                       const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr odom_to_position(
                                        new ff::RosOdomRapidPosition(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = odom_to_position;
    }
  } else {
    return false;
  }

  return true;
}


bool DdsRosBridge::BuildPayloadStateToPayloadState(const std::string& sub_topic,
                                                   const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr payload_state_to_payload_state(
                                      new ff::RosPayloadStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = payload_state_to_payload_state;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildPlanStatusToPlanStatus(const std::string& sub_topic,
                                               const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr plan_status_to_plan_status(
                                new ff::RosPlanStatusRapidPlanStatus(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = plan_status_to_plan_status;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildPmcCmdStateToRapid(const std::string& sub_topic,
                                           const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr pmc_command_to_pmc_command(
                                        new ff::RosPmcCmdStateToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = pmc_command_to_pmc_command;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildStringToTextMessage(const std::string& name) {
  std::string pub_topic, sub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use) &&
      ReadTopicInfo(name, "sub", sub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr str_to_text(
                                    new ff::RosStringRapidTextMessage(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
      components_++;
      ros_sub_rapid_pubs_[name] = str_to_text;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildTelemetryToRapid(const std::string& sub_topic,
                                         const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr telemetry_to_telemetry(
                            new ff::RosTelemetryRapidTelemetry(sub_topic,
                                                               pub_topic,
                                                               nh_,
                                                               config_params_));
      components_++;
      ros_sub_rapid_pubs_[name] = telemetry_to_telemetry;
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCommandToCommand(const std::string& pub_topic,
                                         const std::string& name) {
  std::string sub_topic;
  bool use;

  if (ReadTopicInfo(name, "sub", sub_topic, use)) {
    if (use) {
      ff::RapidSubRosPubPtr command_to_command(
                                      new ff::RapidCommandRosCommand(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      rapid_sub_ros_pubs_.push_back(command_to_command);
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCompressedFileToCompressedFile(
                                                  const std::string& pub_topic,
                                                  const std::string& name) {
  std::string sub_topic;
  bool use;

  if (ReadTopicInfo(name, "sub", sub_topic, use)) {
    if (use) {
      ff::RapidSubRosPubPtr compressed_file_to_compressed_file(
                        new ff::RapidCompressedFileRosCompressedFile(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      rapid_sub_ros_pubs_.push_back(compressed_file_to_compressed_file);
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::BuildCommandConfigToCommandConfig(const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      // keep in scope to retain reliable durable
      ff::RapidPubPtr command_config_to_command_config(
                    new ff::RosCommandConfigRapidCommandConfig(pub_topic,
                                                               nh_,
                                                               config_params_));
      components_++;
      rapid_pubs_.push_back(command_config_to_command_config);
    }
  } else {
    return false;
  }

  return true;
}

bool DdsRosBridge::SetTelem(float rate, std::string &err_msg, RateType type) {
  if (ros_sub_rapid_pubs_.count("RTRT") == 0) {
    err_msg = "DDS Bridge: Telemetry message stuff not added but it is needed.";
    return false;
  }

  ff::RosTelemetryRapidTelemetry *RTRT =
                                  static_cast<ff::RosTelemetryRapidTelemetry *>
                                  (ros_sub_rapid_pubs_["RTRT"].get());

  switch (type) {
    case COMM_STATUS:
      RTRT->SetCommStatusRate(rate);
      break;
    case CPU_STATE:
      RTRT->SetCpuStateRate(rate);
      break;
    case DISK_STATE:
      RTRT->SetDiskStateRate(rate);
      break;
    case EKF_STATE:
      RTRT->SetEkfStateRate(rate);
      break;
    case GNC_STATE:
      RTRT->SetGncStateRate(rate);
      break;
    case PMC_CMD_STATE:
      RTRT->SetPmcCmdStateRate(rate);
      break;
    case POSITION:
      RTRT->SetPositionRate(rate);
      break;
    default:
      err_msg = "DDS Bridge: Unknown type when setting telemetry rate!";
      return false;
  }

  return true;
}

bool DdsRosBridge::SetCommStatusRate(float rate, std::string &err_msg) {
  // TODO(Katie) Add comm stuff when you add comm message to bridge
  if (!SetTelem(rate, err_msg, COMM_STATUS)) {
    return false;
  }

  return true;
}

bool DdsRosBridge::SetCpuStateRate(float rate, std::string &err_msg) {
  if (ros_sub_rapid_pubs_.count("RCSRCS") == 0) {
    err_msg = "DDS Bridge: Cpu state message not added but it is needed.";
    return false;
  }

  if (!SetTelem(rate, err_msg, CPU_STATE)) {
    return false;
  }

  ff::RosCpuStateToRapid *RCSR = static_cast<ff::RosCpuStateToRapid *>
                                          (ros_sub_rapid_pubs_["RCSRCS"].get());

  RCSR->SetPublishRate(rate);
  return true;
}

bool DdsRosBridge::SetDiskStateRate(float rate, std::string &err_msg) {
  if (ros_sub_rapid_pubs_.count("RDSRDS") == 0) {
    err_msg = "DDS Bridge: Disk state message not added but it is needed.";
    return false;
  }

  if (!SetTelem(rate, err_msg, DISK_STATE)) {
    return false;
  }

  ff::RosDiskStateToRapid *RDSR = static_cast<ff::RosDiskStateToRapid *>
                                          (ros_sub_rapid_pubs_["RDSRDS"].get());
  RDSR->SetPublishRate(rate);
  return true;
}

bool DdsRosBridge::SetEkfPositionRate(float rate,
                                      std::string &err_msg,
                                      RateType type) {
  if (ros_sub_rapid_pubs_.count("RORP") == 0) {
    err_msg = "DDS Bridge: Ekf state message not added but it is needed.";
    return false;
  }

  if (!SetTelem(rate, err_msg, type)) {
    return false;
  }

  ff::RosOdomRapidPosition *RORP = static_cast<ff::RosOdomRapidPosition *>
                                            (ros_sub_rapid_pubs_["RORP"].get());

  if (type == EKF_STATE) {
    RORP->SetEkfPublishRate(rate);
  } else if (type == POSITION) {
    RORP->SetPositionPublishRate(rate);
  } else {
      err_msg = "DDS Bridge: Unknown type when setting ekf or position rate!";
      return false;
  }

  return true;
}

bool DdsRosBridge::SetGncStateRate(float rate, std::string &err_msg) {
  if (ros_sub_rapid_pubs_.count("RGCCRGCC") == 0) {
    err_msg = "DDS Bridge: Gnc control command not added but it is needed.";
    return false;
  }

  if (ros_sub_rapid_pubs_.count("RGCSRGCS") == 0) {
    err_msg = "DDS Bridge: Gnc control shaper not added but it is needed.";
    return false;
  }

  if (ros_sub_rapid_pubs_.count("RGCTRGCT") == 0) {
    err_msg = "DDS Bridge: Gnc control trajectory not added but it is needed.";
    return false;
  }

  if (!SetTelem(rate, err_msg, GNC_STATE)) {
    return false;
  }

  ff::RosGncFamCmdStateToRapid *RGCC =
                                    static_cast<ff::RosGncFamCmdStateToRapid *>
                                    (ros_sub_rapid_pubs_["RGCCRGCC"].get());

  ff::RosGncControlStateToRapid *RGCS =
                                    static_cast<ff::RosGncControlStateToRapid *>
                                    (ros_sub_rapid_pubs_["RGCSRGCS"].get());

  ff::RosGncControlStateToRapid *RGCT =
                                    static_cast<ff::RosGncControlStateToRapid *>
                                    (ros_sub_rapid_pubs_["RGCTRGCT"].get());

  RGCC->SetGncPublishRate(rate);
  RGCS->SetGncPublishRate(rate);
  RGCT->SetGncPublishRate(rate);
  return true;
}

bool DdsRosBridge::SetPmcStateRate(float rate, std::string &err_msg) {
  if (ros_sub_rapid_pubs_.count("RPCSRPCS") == 0) {
    err_msg = "DDS Bridge: Pmc cmd state message not added but it is needed.";
    return false;
  }

  if (!SetTelem(rate, err_msg, PMC_CMD_STATE)) {
    return false;
  }

  ff::RosPmcCmdStateToRapid *RPCS = static_cast<ff::RosPmcCmdStateToRapid *>
                                        (ros_sub_rapid_pubs_["RPCSRPCS"].get());

  RPCS->SetPmcPublishRate(rate);
  return true;
}

bool DdsRosBridge::SetTelemRateCallback(ff_msgs::SetRate::Request &req,
                                        ff_msgs::SetRate::Response &res) {
  std::string err_msg;
  switch (req.which) {
    case ff_msgs::SetRate::Request::COMM_STATUS:
      res.success = SetCommStatusRate(req.rate, err_msg);
      break;
    case ff_msgs::SetRate::Request::CPU_STATE:
      res.success = SetCpuStateRate(req.rate, err_msg);
      break;
    case ff_msgs::SetRate::Request::DISK_STATE:
      res.success = SetDiskStateRate(req.rate, err_msg);
      break;
    case ff_msgs::SetRate::Request::EKF_STATE:
      res.success = SetEkfPositionRate(req.rate, err_msg, EKF_STATE);
      break;
    case ff_msgs::SetRate::Request::GNC_STATE:
      res.success = SetGncStateRate(req.rate, err_msg);
      break;
    case ff_msgs::SetRate::Request::PMC_CMD_STATE:
      res.success = SetPmcStateRate(req.rate, err_msg);
      break;
    case ff_msgs::SetRate::Request::POSITION:
      res.success = SetEkfPositionRate(req.rate, err_msg, POSITION);
      break;
    default:
      res.success = false;
      err_msg = "DDS Bridge: Unknown which in set telemetry rate service.";
  }

  res.status = err_msg;
  return res.success;
}

void DdsRosBridge::Initialize(ros::NodeHandle *nh) {
  // Need to get robot name which is in the lua config files, add files, read
  // files, and get robot name  but don't get the rest of the config parameters
  // since these params are used to create the classes that use rapid dds. Need
  // to set up Miro/DDs before reading the parameters.
  config_params_.AddFile("commands.config");
  config_params_.AddFile("communications/dds_ros_bridge.config");

  if (!config_params_.ReadFiles()) {
    ROS_FATAL("DDS Bridge: Error reading config files.");
    exit(EXIT_FAILURE);
    return;
  }

  if (!config_params_.GetStr("agent_name", &agent_name_)) {
    ROS_FATAL("DDS Bridge: Could not read robot name.");
    exit(EXIT_FAILURE);
    return;
  }

  nh_ = *nh;

  // In simulation, the namespace is usually set to the robot name so we need to
  // check if we are in simulation and get the right name
  if (agent_name_ == "sim" || agent_name_ == "simulator") {
    // The platform name should be the simulated robot name
    agent_name_ = GetPlatform();

    // If there is not robot name, set it to a default name so that we can
    // connect to the bridge
    if (agent_name_ == "") {
      agent_name_ = "Bumble";
    } else {
      // Make sure that first letter of robot name is capitialized. GDS only
      // recognizes capitialized robot names.
      agent_name_[0] = toupper(agent_name_[0]);
    }
  }

  // Publish agent name so that the android apks have an easy way to get the
  // robot name. Topic is latched.
  robot_name_pub_ = nh_.advertise<std_msgs::String>(TOPIC_ROBOT_NAME,
                                                    10,
                                                    true);
  std_msgs::String robot_name_msg;
  robot_name_msg.data = agent_name_;
  robot_name_pub_.publish(robot_name_msg);

  int fake_argc = 1;

  // TODO(tfmorse): make hardcoded values configurable

  // Make path to QOS and NDDS files
  std::string config_path = ff_common::GetConfigDir();
  config_path += "/communications/dds/";

  // Create fake argv containing only the particaptant name
  // Participant name needs to uniue so combine robot name with timestamp
  ros::Time time = ros::Time::now();
  participant_name_ = agent_name_ + std::to_string(time.sec);
  char **fake_argv = new char*[1];
  fake_argv[0] = new char[(participant_name_.size() + 1)];
  std::strcpy(fake_argv[0], participant_name_.c_str());  // NOLINT

  /* fake miro log into thinking we have no arguments */
  Miro::Log::init(fake_argc, fake_argv);
  Miro::Log::level(9);

  /* fake miro configuration into thinking we have no arguments */
  Miro::Configuration::init(fake_argc, fake_argv);

  Miro::RobotParameters *robot_params = Miro::RobotParameters::instance();
  kn::DdsEntitiesFactorySvcParameters *dds_params =
      kn::DdsEntitiesFactorySvcParameters::instance();

  /* get the defaults for *all the things!* */
  Miro::ConfigDocument *config = Miro::Configuration::document();
  config->setSection("Robot");
  config->getParameters("Miro::RobotParameters", *robot_params);
  config->getParameters("kn::DdsEntitiesFactorySvcParameters", *dds_params);

  robot_params->name = agent_name_;
  robot_params->namingContextName = robot_params->name;

  SubstituteROBOT_NAME(dds_params);

  // Clear config files so that dds only looks for the files we add
  dds_params->participants[0].discoveryPeersFiles.clear();
  dds_params->configFiles.clear();

  dds_params->participants[0].participantName = participant_name_;
  dds_params->participants[0].domainId = 37;
  dds_params->participants[0].discoveryPeersFiles.push_back(
      (config_path + "NDDS_DISCOVERY_PEERS"));
  dds_params->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));

  /**
   * Facade to initialize basic system
   * get instance to RobotParameters
   * init Miro::Log
   * init Miro::Configuration
   *    load doc given my -MCF  MiroConfigFile
   * get reference to Miro::Configuration::document()
   * set RobotParameters from ConfigDocument
   *
   * parse options and set RobotParameters
   *    -MRN  MiroRobotName
   *    -MNC  MiroNamingContext
   *    -MNT  MiroNamingTimeout
   *    -MDR  MiroDataRoot
   *    -MEC  MiroEventChannel
   *    -MNN  MiroNoNaming
   *    -?    MiroHelp
   Miro::Robot::init(argc, argv);
   */

  /**
   * Facade to initialize basic dds-related parameters
   * get instance to DdsEntitiesFactorySvcParameters
   * get reference to Miro::Configuration::document()
   * set DdsEntitiesFactorySvcParameters from ConfigDocument
   *
   * parse options and set DdsEntitiesFactorySvcParameters
   *    -MDC  MiroDdsConfig
   *    -MDI  MiroDdsDomainId
   *    -MDP  MiroDiscoverPeers
   *    -MDM  MiroDdsMonitor
   *    -MDPN MiroDdsParticipantName
   *    -MDDL MiroDdsDefaultLibrary
   *    -MDED MiroDdsEndpointDiscover
   kn::DdsSupport::init(argc, argv);
   */

  /**
   * get instance to DdsEntitiesFactorySvcParameters
   *
   * the following are default values in knDdsParameters.xml
   * configFiles.push_back(RAPID_QOS_PROFILES.xml)
   * defaultLibrary = "RapidQosLibrary"
   * defaultProfile = "RapidDefaultQos"
   */

  /**
   * Hardcode participant name
   entityParams->participants[0].participantName = argv[0];
   */

  /**
   * Use DdsEntitiesFactorySvc to create a new DdsEntitiesFactory
   * which will create all objects:
   *    Participants   DdsDomainParticpantRepository::instance()
   *    Publishers     DdsPublisherRespoitory::instance()
   *    Subscribers    DdsSubscriberRepository::instance()
   *    Topics
   * and store in relevant repository
   * based on DdsEntitesFactoryParameters
   */
  dds_entities_factory_.reset(new kn::DdsEntitiesFactorySvc());
  dds_entities_factory_->init(dds_params);

  if (!ReadParams()) {
    exit(EXIT_FAILURE);
    return;
  }

  srv_set_telem_rate_ = nh_.advertiseService(
                                    SERVICE_COMMUNICATIONS_DDS_SET_TELEM_RATES,
                                    &DdsRosBridge::SetTelemRateCallback,
                                    this);
}

bool DdsRosBridge::ReadTopicInfo(const std::string& topic_abbr,
                                 const std::string& sub_or_pub,
                                 std::string& topic,
                                 bool& use) {
  std::string use_exp = "use_" + topic_abbr;
  std::string topic_exp = sub_or_pub + "_topic_" + topic_abbr;

  if (!config_params_.GetBool(use_exp.c_str(), &use)) {
    ROS_FATAL("DDS Bridge: use %s not specified!", topic_abbr.c_str());
    return false;
  }

  if (use) {
    if (!config_params_.GetStr(topic_exp.c_str(), &topic)) {
      ROS_FATAL("DDS Bridge: sub topic %s not specified!", topic_abbr.c_str());
      return false;
    }
  }

  return true;
}

bool DdsRosBridge::ReadParams() {
  std::string sub_topic;
  std::string pub_topic;
  components_ = 0;

  // rapid_command_ros_command => RCRC
  if (!BuildCommandToCommand(TOPIC_COMMUNICATIONS_DDS_COMMAND, "RCRC")) {
    return false;
  }

  // rapid_compressed_file_data_ros_compressed_file_data => RCFDRCFD
  if (!BuildCompressedFileToCompressedFile(TOPIC_COMMUNICATIONS_DDS_DATA,
                                           "RCFDRCFD")) {
    return false;
  }

  // rapid_compressed_file_plan_ros_compressed_file_plan => RCFPRCFP
  if (!BuildCompressedFileToCompressedFile(TOPIC_COMMUNICATIONS_DDS_PLAN,
                                           "RCFPRCFP")) {
    return false;
  }

  // rapid_compressed_file_zones_ros_compressed_file_zones => RCFZRCFZ
  if (!BuildCompressedFileToCompressedFile(TOPIC_COMMUNICATIONS_DDS_ZONES,
                                           "RCFZRCFZ")) {
    return false;
  }

  // ros_access_control_rapid_state => RACS
  if (!BuildAccessControlStateToRapid(TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE,
                                      "RACS")) {
    return false;
  }

  // ros_ack_rapid_ack => RARA
  if (!BuildAckToRapid(TOPIC_MANAGEMENT_ACK, "RARA")) {
    return false;
  }

  // ros_agent_state_rapid_agent_state => RASRAS
  if (!BuildAgentStateToRapid(TOPIC_MANAGEMENT_EXEC_AGENT_STATE, "RASRAS")) {
    return false;
  }

  // ros_arm_joint_sample_rapid_arm_joint_sample => RAJSRAJS
  if (!BuildArmJointSampleToRapid(TOPIC_BEHAVIORS_ARM_JOINT_SAMPLE,
                                  "RAJSRAJS")) {
    return false;
  }

  // ros_arm_state_rapid_arm_state => RARMRARM
  if (!BuildArmStateToRapid(TOPIC_BEHAVIORS_ARM_ARM_STATE, "RARMRARM")) {
    return false;
  }

  // ros_battery_state_rapid_battery_config => RBSRBS
  if (!BuildBatteryStateToRapid(TOPIC_HARDWARE_EPS_BATTERY_STATE_TL,
                                TOPIC_HARDWARE_EPS_BATTERY_STATE_TR,
                                TOPIC_HARDWARE_EPS_BATTERY_STATE_BL,
                                TOPIC_HARDWARE_EPS_BATTERY_STATE_BR,
                                TOPIC_HARDWARE_EPS_BATTERY_TEMP_TL,
                                TOPIC_HARDWARE_EPS_BATTERY_TEMP_TR,
                                TOPIC_HARDWARE_EPS_BATTERY_TEMP_BL,
                                TOPIC_HARDWARE_EPS_BATTERY_TEMP_BR,
                                "RBSRBS")) {
    return false;
  }

  // ros_command_rapid_command => RosCRapC
  if (!BuildCommandToRapid(TOPIC_COMMAND,
                           TOPIC_COMMUNICATIONS_DDS_COMMAND,
                           TOPIC_MANAGEMENT_ACCESS_CONTROL_CMD,
                           "ROSCRAPC")) {
    return false;
  }

  // ros_command_config_rapid_command_config => RCCRCC
  if (!BuildCommandConfigToCommandConfig("RCCRCC")) {
    return false;
  }

  // ros_compressed_file_rapid_compressed_file => RosCFRapCF
  if (!BuildCompressedFileToRapid(TOPIC_MANAGEMENT_EXEC_PLAN,
                                  "ROSCFRAPCF")) {
    return false;
  }

  // ros_compressed_file_ack_rapid_compressed_file_ack => RCFARCFA
  if (!BuildCompressedFileAckToRapid(TOPIC_MANAGEMENT_EXEC_CF_ACK,
                                     "RCFARCFA")) {
    return false;
  }

  // ros_compressed_dock_cam_image_rapid_image => RCDCIRI
  if (!BuildCompressedImageToImage(TOPIC_MANAGEMENT_IMG_SAMPLER_DOCK_CAM_STREAM,
                                   "RCDCIRI")) {
    return false;
  }

  // ros_compressed_haz_cam_image_rapid_image => RCHCIRI
  if (!BuildCompressedImageToImage(TOPIC_MANAGEMENT_IMG_SAMPLER_HAZ_CAM_STREAM,
                                   "RCHCIRI")) {
    return false;
  }

  // ros_compressed_nav_cam_image_rapid_image => RCNCIRI
  if (!BuildCompressedImageToImage(TOPIC_MANAGEMENT_IMG_SAMPLER_NAV_CAM_STREAM,
                                   "RCNCIRI")) {
    return false;
  }

  // ros_compressed_perch_cam_image_rapid_image => RCPCIRI
  if (!BuildCompressedImageToImage(
                                  TOPIC_MANAGEMENT_IMG_SAMPLER_PERCH_CAM_STREAM,
                                  "RCPCIRI")) {
    return false;
  }

  // ros_compressed_science_cam_image_rapid_image => RCSCIRI
  if (!BuildCompressedImageToImage(TOPIC_HARDWARE_SCI_CAM, "RCSCIRI")) {
    return false;
  }

  // ros_cpu_state_rapid_cpu_state => RCSRCS
  if (!BuildCpuStateToRapid(TOPIC_MANAGEMENT_CPU_MONITOR_STATE, "RCSRCS")) {
    return false;
  }

  // ros_data_to_disk_rapid_data_to_disk => RDTDRDTD
  if (!BuildDataToDiskStateToRapid(TOPIC_MANAGEMENT_DATA_BAGGER_STATE,
                                   TOPIC_MANAGEMENT_DATA_BAGGER_TOPICS,
                                   "RDTDRDTD")) {
    return false;
  }

  // ros_disk_state_rapid_disk_state => RDSRDS
  if (!BuildDiskStateToRapid(TOPIC_MANAGEMENT_DISK_MONITOR_STATE, "RDSRDS")) {
    return false;
  }

  // ros_fault_config_rapid_fault_config => RFCRFC
  if (!BuildFaultConfigToRapid(TOPIC_MANAGEMENT_SYS_MONITOR_CONFIG, "RFCRFC")) {
    return false;
  }

  // ros_fault_state_rapid_fault_state => RFSRFS
  if (!BuildFaultStateToRapid(TOPIC_MANAGEMENT_SYS_MONITOR_STATE, "RFSRFS")) {
    return false;
  }

  // ros_gnc_control_command_rapid_gnc_control_command => RGCCRGCC
  if (!BuildGncFamCmdStateToRapid(TOPIC_GNC_CTL_COMMAND, "RGCCRGCC")) {
    return false;
  }

  // ros_gnc_control_shaper_rapid_gnc_control_shaper => RGCSRGCS
  if (!BuildGncControlStateToRapid(TOPIC_GNC_CTL_SHAPER, "RGCSRGCS")) {
    return false;
  }

  // ros_gnc_control_trajectory_rapid_gnc_control_trajectory => RGCTRGCT
  if (!BuildGncControlStateToRapid(TOPIC_GNC_CTL_TRAJ, "RGCTRGCT")) {
    return false;
  }

  // ros_guest_science_rapid_guest_science => RGSRGS
  if (!BuildGuestScienceToRapid(TOPIC_GUEST_SCIENCE_MANAGER_STATE,
                                TOPIC_GUEST_SCIENCE_MANAGER_CONFIG,
                                TOPIC_GUEST_SCIENCE_DATA,
                                "RGSRGS")) {
    return false;
  }

  // ros_inertia_rapid_inertia => RIRI
  if (!BuildInertiaDataToRapid(TOPIC_MOBILITY_INERTIA, "RIRI")) {
    return false;
  }

  // ros_log_rapid_log_sample => RLRLS
  // TODO(rgarciar): Change topic name for a constant
  if (!BuildLogSampleToRapid("/rosout", "RLRLS")) {
    return false;
  }

  // ros_odom_rapid_position => RORP
  if (!BuildOdomToPosition(TOPIC_GNC_EKF, "RORP")) {
    return false;
  }

  // ros_payload_power_state_rapid_payload_power_state => RPPSRPPS
  if (!BuildPayloadStateToPayloadState(TOPIC_HARDWARE_EPS_POWER_STATE,
                                       "RPPSRPPS")) {
    return false;
  }

  // ros_plan_status_rapid_plan_status => RPSRPS
  if (!BuildPlanStatusToPlanStatus(TOPIC_MANAGEMENT_EXEC_PLAN_STATUS,
                                   "RPSRPS")) {
    return false;
  }

  // ros_pmc_cmd_state_rapid_pmc_cmd_state => RPCSRPCS
  if (!BuildPmcCmdStateToRapid(TOPIC_HARDWARE_PMC_COMMAND, "RPCSRPCS")) {
    return false;
  }

  // ros_string_rapid_text_message => RSRTM
  if (!BuildStringToTextMessage("RSRTM")) {
    return false;
  }

  // ros_telemetry_rapid_telemetry => RTRT
  if (!BuildTelemetryToRapid(TOPIC_MANAGEMENT_CAMERA_STATE, "RTRT")) {
    return false;
  }

  // Read and set the publishing rates
  if (!ReadRateParams()) {
    exit(EXIT_FAILURE);
    return false;
  }

  // Read in the multiple that the battery estimated time remaining needs to be
  // rounded to.
  int multiple;
  if (!config_params_.GetInt("battery_time_round_to_multiple", &multiple)) {
    ROS_FATAL("DDS Bridge: Battery time round to multiple not specified!");
    return false;
  }

  if (ros_sub_rapid_pubs_.count("RBSRBS") == 0) {
    ROS_ERROR("DDS Bridge: Battery msg stuff not added and it is needed!");
    return false;
  }

  ff::RosBatteryStateToRapid *RBSRBS = static_cast<ff::RosBatteryStateToRapid *>
                                          (ros_sub_rapid_pubs_["RBSRBS"].get());

  RBSRBS->SetBatteryTimeMultiple(multiple);

  return true;
}

bool DdsRosBridge::ReadRateParams() {
  // Read in the telemetery rates so the bridge knows how often to publish
  // the telemetry messages
  float temp_rate;
  std::string err_msg;

  if (!config_params_.GetReal("comm_status_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: comm state rate not specified!");
    return false;
  }

  if (!SetCommStatusRate(temp_rate, err_msg)) {
    ROS_FATAL("%s", err_msg.c_str());
    return false;
  }

  if (!config_params_.GetReal("cpu_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: cpu state rate not specified!");
    return false;
  }

  if (!SetCpuStateRate(temp_rate, err_msg)) {
    ROS_FATAL("%s", err_msg.c_str());
    return false;
  }

  if (!config_params_.GetReal("disk_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: disk state rate not specified!");
    return false;
  }

  if (!SetDiskStateRate(temp_rate, err_msg)) {
    ROS_FATAL("%s", err_msg.c_str());
    return false;
  }

  if (!config_params_.GetReal("ekf_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: ekf state rate not specified!");
    return false;
  }

  if (!SetEkfPositionRate(temp_rate, err_msg, EKF_STATE)) {
    ROS_FATAL("%s", err_msg.c_str());
    return false;
  }

  if (!config_params_.GetReal("gnc_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: gnc state rate not specified!");
    return false;
  }

  if (!SetGncStateRate(temp_rate, err_msg)) {
    ROS_FATAL("%s", err_msg.c_str());
    return false;
  }

  if (!config_params_.GetReal("pmc_command_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: pmc command rate not specified!");
    return false;
  }

  if (!SetPmcStateRate(temp_rate, err_msg)) {
    ROS_FATAL("%s", err_msg.c_str());
    return false;
  }

  if (!config_params_.GetReal("position_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: position rate not specified!");
    return false;
  }

  if (!SetEkfPositionRate(temp_rate, err_msg, POSITION)) {
    ROS_FATAL("%s", err_msg.c_str());
    return false;
  }

  return true;
}

}   // end namespace dds_ros_bridge

PLUGINLIB_EXPORT_CLASS(dds_ros_bridge::DdsRosBridge, nodelet::Nodelet)
