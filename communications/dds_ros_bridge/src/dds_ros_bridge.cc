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

int DdsRosBridge::BuildAccessControlStateToRapid(const std::string& sub_topic,
                                                 const std::string& pub_topic,
                                                 const std::string& name) {
  ff::RosSubRapidPubPtr acs_to_acs(
                                new ff::RosAccessControlStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  ros_sub_rapid_pubs_[name] = acs_to_acs;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildAckToRapid(const std::string& sub_topic,
                                  const std::string& pub_topic,
                                  const std::string& name) {
  ff::RosSubRapidPubPtr ack_to_ack(new ff::RosAckToRapid(sub_topic,
                                                         pub_topic,
                                                         nh_));
  ros_sub_rapid_pubs_[name] = ack_to_ack;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildAgentStateToRapid(const std::string& sub_topic,
                                         const std::string& pub_topic,
                                         const std::string& name) {
  ff::RosSubRapidPubPtr agent_to_agent(new ff::RosAgentStateToRapid(sub_topic,
                                                                    pub_topic,
                                                                    nh_));
  ros_sub_rapid_pubs_[name] = agent_to_agent;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildArmJointSampleToRapid(const std::string& sub_topic,
                                             const std::string& pub_topic,
                                             const std::string& name) {
  ff::RosSubRapidPubPtr joint_to_joint(
                                    new ff::RosArmJointSampleToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  ros_sub_rapid_pubs_[name] = joint_to_joint;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildArmStateToRapid(const std::string& sub_topic,
                                       const std::string& pub_topic,
                                       const std::string& name) {
  ff::RosSubRapidPubPtr arm_to_arm(new ff::RosArmStateToRapid(sub_topic,
                                                              pub_topic,
                                                              nh_));
  ros_sub_rapid_pubs_[name] = arm_to_arm;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildBatteryStateToRapid(
                                  const std::string& sub_topic_battery_state_TL,
                                  const std::string& sub_topic_battery_state_TR,
                                  const std::string& sub_topic_battery_state_BL,
                                  const std::string& sub_topic_battery_state_BR,
                                  const std::string& sub_topic_battery_temp_TL,
                                  const std::string& sub_topic_battery_temp_TR,
                                  const std::string& sub_topic_battery_temp_BL,
                                  const std::string& sub_topic_battery_temp_BR,
                                  const std::string& pub_topic,
                                  const std::string& name) {
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
  ros_sub_rapid_pubs_[name] = battery_to_battery;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildCommandToRapid(const std::string& sub_topic,
                                      const std::string& ac_sub_topic,
                                      const std::string& failed_cmd_sub_topic,
                                      const std::string& pub_topic,
                                      const std::string& name) {
  ff::RosSubRapidPubPtr command_to_command(new ff::RosCommandToRapid(
                                                          sub_topic,
                                                          ac_sub_topic,
                                                          failed_cmd_sub_topic,
                                                          pub_topic,
                                                          nh_,
                                                          agent_name_));
  ros_sub_rapid_pubs_[name] = command_to_command;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildCompressedImageToImage(const std::string& sub_topic,
                                              const std::string& pub_topic,
                                              const std::string& name) {
  ff::RosSubRapidPubPtr compressed_image_to_image(
                                new ff::RosCompressedImageRapidImage(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  ros_sub_rapid_pubs_[name] = compressed_image_to_image;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildCpuStateToRapid(const std::string& sub_topic,
                                       const std::string& pub_topic,
                                       const std::string& name) {
  ff::RosSubRapidPubPtr cpu_to_cpu(new ff::RosCpuStateToRapid(sub_topic,
                                                              pub_topic,
                                                              nh_));
  ros_sub_rapid_pubs_[name] = cpu_to_cpu;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildDataToDiskStateToRapid(
                                            const std::string& state_sub_topic,
                                            const std::string& topics_sub_topic,
                                            const std::string& pub_topic,
                                            const std::string& name) {
  ff::RosSubRapidPubPtr data_to_disk_to_data_to_disk(
                                  new ff::RosDataToDiskToRapid(state_sub_topic,
                                                               topics_sub_topic,
                                                               pub_topic,
                                                               nh_));
  ros_sub_rapid_pubs_[name] = data_to_disk_to_data_to_disk;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildDiskStateToRapid(const std::string& sub_topic,
                                        const std::string& pub_topic,
                                        const std::string& name) {
  ff::RosSubRapidPubPtr disk_to_disk(new ff::RosDiskStateToRapid(sub_topic,
                                                                 pub_topic,
                                                                 nh_));
  ros_sub_rapid_pubs_[name] = disk_to_disk;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildFaultConfigToRapid(const std::string& sub_topic,
                                          const std::string& pub_topic,
                                          const std::string& name) {
  ff::RosSubRapidPubPtr fault_to_fault(new ff::RosFaultConfigToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  ros_sub_rapid_pubs_[name] = fault_to_fault;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildFaultStateToRapid(const std::string& sub_topic,
                                         const std::string& pub_topic,
                                         const std::string& name) {
  ff::RosSubRapidPubPtr fault_to_fault(new ff::RosFaultStateToRapid(sub_topic,
                                                                    pub_topic,
                                                                    nh_));
  ros_sub_rapid_pubs_[name] = fault_to_fault;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildGncFamCmdStateToRapid(const std::string& sub_topic,
                                             const std::string& pub_topic,
                                             const std::string& name) {
  ff::RosSubRapidPubPtr gnc_fam_cmd_to_gnc_fam_cmd(
                                    new ff::RosGncFamCmdStateToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  ros_sub_rapid_pubs_[name] = gnc_fam_cmd_to_gnc_fam_cmd;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildGncControlStateToRapid(const std::string& sub_topic,
                                              const std::string& pub_topic,
                                              const std::string& name) {
  ff::RosSubRapidPubPtr gnc_control_to_gnc_control(
                                    new ff::RosGncControlStateToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
  ros_sub_rapid_pubs_[name] = gnc_control_to_gnc_control;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildGuestScienceToRapid(const std::string& state_sub_topic,
                                           const std::string& config_sub_topic,
                                           const std::string& data_sub_topic,
                                           const std::string& pub_topic,
                                           const std::string& name) {
  ff::RosSubRapidPubPtr guest_science_to_guest_science(
                                new ff::RosGuestScienceToRapid(state_sub_topic,
                                                               config_sub_topic,
                                                               data_sub_topic,
                                                               pub_topic,
                                                               nh_));
  ros_sub_rapid_pubs_[name] = guest_science_to_guest_science;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildInertiaDataToRapid(const std::string& sub_topic,
                                          const std::string& pub_topic,
                                          const std::string& name) {
  ff::RosSubRapidPubPtr inertia_to_inertia(new ff::RosInertiaToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));

  ros_sub_rapid_pubs_[name] = inertia_to_inertia;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildLogSampleToRapid(const std::string& sub_topic,
                                         const std::string& pub_topic,
                                         const std::string& name) {
  ff::RosSubRapidPubPtr log_to_log(new ff::RosLogSampleToRapid(sub_topic,
                                                               pub_topic,
                                                               nh_));
  ros_sub_rapid_pubs_[name] = log_to_log;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildOdomToPosition(const std::string& sub_topic,
                                      const std::string& pub_topic,
                                      const std::string& name) {
  ff::RosSubRapidPubPtr odom_to_position(new ff::RosOdomRapidPosition(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
  ros_sub_rapid_pubs_[name] = odom_to_position;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildPlanStatusToPlanStatus(const std::string& sub_topic,
                                              const std::string& pub_topic,
                                              const std::string& name) {
  ff::RosSubRapidPubPtr plan_status_to_plan_status(
                                new ff::RosPlanStatusRapidPlanStatus(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  ros_sub_rapid_pubs_[name] = plan_status_to_plan_status;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildPmcCmdStateToRapid(const std::string& sub_topic,
                                              const std::string& pub_topic,
                                              const std::string& name) {
  ff::RosSubRapidPubPtr pmc_command_to_pmc_command(
                                    new ff::RosPmcCmdStateToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
  ros_sub_rapid_pubs_[name] = pmc_command_to_pmc_command;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildStringToTextMessage(const std::string& sub_topic,
                                           const std::string& pub_topic,
                                           const std::string& name) {
  ff::RosSubRapidPubPtr str_to_text(new ff::RosStringRapidTextMessage(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
  ros_sub_rapid_pubs_[name] = str_to_text;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildTelemetryToRapid(const std::string& sub_topic,
                                        const std::string& pub_topic,
                                        const std::string& name) {
  ff::RosSubRapidPubPtr telemetry_to_telemetry(
                            new ff::RosTelemetryRapidTelemetry(sub_topic,
                                                               pub_topic,
                                                               nh_,
                                                               config_params_));
  ros_sub_rapid_pubs_[name] = telemetry_to_telemetry;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildCommandToCommand(const std::string& sub_topic,
                                        const std::string& pub_topic,
                                        const std::string& name) {
  ff::RapidSubRosPubPtr command_to_command(
                                      new ff::RapidCommandRosCommand(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  rapid_sub_ros_pubs_.push_back(command_to_command);
  return rapid_sub_ros_pubs_.size();
}

int DdsRosBridge::BuildCompressedFileToCompressedFile(
                                                  const std::string& sub_topic,
                                                  const std::string& pub_topic,
                                                  const std::string& name) {
  ff::RapidSubRosPubPtr compressed_file_to_compressed_file(
                        new ff::RapidCompressedFileRosCompressedFile(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  rapid_sub_ros_pubs_.push_back(compressed_file_to_compressed_file);
  return rapid_sub_ros_pubs_.size();
}

int DdsRosBridge::BuildCompressedFileToRapid(const std::string& sub_topic,
                                             const std::string& pub_topic,
                                             const std::string& name) {
  ff::RosSubRapidPubPtr compressed_file_to_rapid(
                                    new ff::RosCompressedFileToRapid(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
  ros_sub_rapid_pubs_[name] = compressed_file_to_rapid;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildCompressedFileAckToRapid(const std::string& sub_topic,
                                                const std::string& pub_topic,
                                                const std::string& name) {
  ff::RosSubRapidPubPtr compressed_file_ack_to_rapid(
                                  new ff::RosCompressedFileAckToRapid(sub_topic,
                                                                      pub_topic,
                                                                      nh_));
  ros_sub_rapid_pubs_[name] = compressed_file_ack_to_rapid;
  return ros_sub_rapid_pubs_.size();
}

int DdsRosBridge::BuildCommandConfigToCommandConfig(
                                                  const std::string& pub_topic,
                                                  const std::string& name) {
  // keep in scope to retain reliable durable
  ff::RapidPubPtr command_config_to_command_config(
                    new ff::RosCommandConfigRapidCommandConfig(pub_topic,
                                                               nh_,
                                                               config_params_));
  rapid_pubs_.push_back(command_config_to_command_config);
  return rapid_pubs_.size();
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
  std::string config_path = common::GetConfigDir();
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

bool DdsRosBridge::ReadParams() {
  std::string sub_topic, sub_topic2;
  std::string pub_topic;
  bool use;
  components_ = 0;

  // rapid_command_ros_command => RCRC
  if (!config_params_.GetBool("use_RCRC", &use)) {
    ROS_FATAL("DDS Bridge: use RCRC not specified");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("sub_topic_RCRC", &sub_topic)) {
      ROS_FATAL("DDS Bridge: sub topic RCRC not specified!");
      return false;
    }

    BuildCommandToCommand(sub_topic, TOPIC_COMMUNICATIONS_DDS_COMMAND, "RCRC");
    components_++;
  }

  // rapid_compressed_file_plan_ros_compressed_file_plan => RCFPRCFP
  if (!config_params_.GetBool("use_RCFPRCFP", &use)) {
    ROS_FATAL("DDS Bridge: use RCFPRCFP not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("sub_topic_RCFPRCFP", &sub_topic)) {
      ROS_FATAL("DDS Bridge: sub topic RCFPRCFP not specified!");
      return false;
    }

    BuildCompressedFileToCompressedFile(sub_topic,
                                        TOPIC_COMMUNICATIONS_DDS_PLAN,
                                        "RCFPRCFP");
    components_++;
  } else {
    ROS_INFO("Not bridging plan compressed files");
  }

  // rapid_compressed_file_zones_ros_compressed_file_zones => RCFZRCFZ
  if (!config_params_.GetBool("use_RCFZRCFZ", &use)) {
    ROS_FATAL("DDS Bridge: use RCFZRCF not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("sub_topic_RCFZRCFZ", &sub_topic)) {
      ROS_FATAL("DDS Bridge: sub topic RCFZRCFZ not specified!");
      return false;
    }

    BuildCompressedFileToCompressedFile(sub_topic,
                                        TOPIC_COMMUNICATIONS_DDS_ZONES,
                                        "RCFZRCFZ");
    components_++;
  } else {
    ROS_INFO("Not bridging zone compressed files!");
  }

  // ros_access_control_rapid_state => RACS
  if (!config_params_.GetBool("use_RACS", &use)) {
    ROS_FATAL("DDS Bridge: use RACS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RACS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RACS not specified!");
      return false;
    }

    BuildAccessControlStateToRapid(TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE,
                                   pub_topic,
                                   "RACS");
    components_++;
  }

  // ros_ack_rapid_ack => RARA
  if (!config_params_.GetBool("use_RARA", &use)) {
    ROS_FATAL("DDS Bridge: use RARA not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RARA", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RARA not specified!");
      return false;
    }

    BuildAckToRapid(TOPIC_MANAGEMENT_ACK, pub_topic, "RARA");
    components_++;
  }

  // ros_agent_state_rapid_agent_state => RASRAS
  if (!config_params_.GetBool("use_RASRAS", &use)) {
    ROS_FATAL("DDS Bridge: use RASRAS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RASRAS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RASRAS not specified!");
      return false;
    }

    BuildAgentStateToRapid(TOPIC_MANAGEMENT_EXEC_AGENT_STATE,
                           pub_topic,
                           "RASRAS");
    components_++;
  }

  // ros_arm_joint_sample_rapid_arm_joint_sample => RAJSRAJS
  if (!config_params_.GetBool("use_RAJSRAJS", &use)) {
    ROS_FATAL("DDS Bridge: use RAJSRAJS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RAJSRAJS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RAJSRAJS not specified!");
      return false;
    }

    BuildArmJointSampleToRapid(TOPIC_BEHAVIORS_ARM_JOINT_SAMPLE,
                               pub_topic,
                               "RAJSRAJS");
    components_++;
  }

  // ros_arm_state_rapid_arm_state => RARMRARM
  if (!config_params_.GetBool("use_RARMRARM", &use)) {
    ROS_FATAL("DDS Bridge: use RARMRARM not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RARMRARM", &pub_topic)) {
      ROS_FATAL("DDS_Bridge: pub topic RARMRARM not specified!");
      return false;
    }

    BuildArmStateToRapid(TOPIC_BEHAVIORS_ARM_ARM_STATE,
                         pub_topic,
                         "RARMRARM");
    components_++;
  }

  // ros_battery_state_rapid_battery_config => RBSRBS
  if (!config_params_.GetBool("use_RBSRBS", &use)) {
    ROS_FATAL("DDS Bridge: pub topic RBSRBS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RBSRBS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RBSRBS not specified!");
      return false;
    }

    BuildBatteryStateToRapid(TOPIC_HARDWARE_EPS_BATTERY_STATE_TL,
                             TOPIC_HARDWARE_EPS_BATTERY_STATE_TR,
                             TOPIC_HARDWARE_EPS_BATTERY_STATE_BL,
                             TOPIC_HARDWARE_EPS_BATTERY_STATE_BR,
                             TOPIC_HARDWARE_EPS_BATTERY_TEMP_TL,
                             TOPIC_HARDWARE_EPS_BATTERY_TEMP_TR,
                             TOPIC_HARDWARE_EPS_BATTERY_TEMP_BL,
                             TOPIC_HARDWARE_EPS_BATTERY_TEMP_BR,
                             pub_topic,
                             "RBSRBS");
    components_++;
  }

  // ros_command_rapid_command => RosCRapC
  if (!config_params_.GetBool("use_ROSCRAPC", &use)) {
    ROS_FATAL("DDS Bridge: use ROSCRAPC not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_ROSCRAPC", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic ROSCRAPC not specified!");
      return false;
    }

    BuildCommandToRapid(TOPIC_COMMAND,
                        TOPIC_COMMUNICATIONS_DDS_COMMAND,
                        TOPIC_MANAGEMENT_ACCESS_CONTROL_CMD,
                        pub_topic,
                        "ROSCRAPC");
    components_++;
  }

  // ros_command_config_rapid_command_config => RCCRCC
  if (!config_params_.GetBool("use_RCCRCC", &use)) {
    ROS_FATAL("DDS Bridge: use RCCRCC not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCCRCC", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RCCRCC not specified!");
      return false;
    }

    BuildCommandConfigToCommandConfig(pub_topic, "RCCRCC");
    components_++;
  }

  // ros_compressed_file_rapid_compressed_file => RosCFRapCF
  if (!config_params_.GetBool("use_ROSCFRAPCF", &use)) {
    ROS_FATAL("DDS Bridge: use ROSCFRAPCF not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_ROSCFRAPCF", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic ROSCFRAPCF not specified!");
      return false;
    }

    BuildCompressedFileToRapid(TOPIC_MANAGEMENT_EXEC_PLAN,
                               pub_topic,
                               "ROSCFRAPCF");
    components_++;
  }

  // ros_compressed_file_ack_rapid_compressed_file_ack => RCFARCFA
  if (!config_params_.GetBool("use_RCFARCFA", &use)) {
    ROS_FATAL("DDS Bridge: use RCFARCFA not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCFARCFA", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RCFARCFA not specified!");
      return false;
    }

    BuildCompressedFileAckToRapid(TOPIC_MANAGEMENT_EXEC_CF_ACK,
                                  pub_topic,
                                  "RCFARCFA");
    components_++;
  }

  // ros_compressed_dock_cam_image_rapid_image => RCDCIRI
  if (!config_params_.GetBool("use_RCDCIRI", &use)) {
    ROS_FATAL("DDS Bridge: use RCDCIRI not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCDCIRI", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RCDCIRI not specified!");
      return false;
    }

    BuildCompressedImageToImage(TOPIC_MANAGEMENT_IMG_SAMPLER_DOCK_CAM_STREAM,
                                pub_topic,
                                "RCDCIRI");
    components_++;
  }

  // ros_compressed_nav_cam_image_rapid_image => RCNCIRI
  if (!config_params_.GetBool("use_RCNCIRI", &use)) {
    ROS_FATAL("DDS Bridge: use RCNCIRI not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCNCIRI", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RCNCIRI not specified!");
      return false;
    }

    BuildCompressedImageToImage(TOPIC_MANAGEMENT_IMG_SAMPLER_NAV_CAM_STREAM,
                                pub_topic,
                                "RCNCIRI");
    components_++;
  }

  // ros_cpu_state_rapid_cpu_state => RCSRCS
  if (!config_params_.GetBool("use_RCSRCS", &use)) {
    ROS_FATAL("DDS Bridge: use RCSRCS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCSRCS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RCSRCS not specified!");
      return false;
    }

    BuildCpuStateToRapid(TOPIC_MANAGEMENT_CPU_MONITOR_STATE,
                         pub_topic,
                         "RCSRCS");
    components_++;
  }

  // ros_data_to_disk_rapid_data_to_disk => RDTDRDTD
  if (!config_params_.GetBool("use_RDTDRDTD", &use)) {
    ROS_FATAL("DDS Bridge: use RDTDRDTD not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RDTDRDTD", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RDTDRDTD not specified!");
      return false;
    }

    BuildDataToDiskStateToRapid(TOPIC_MANAGEMENT_DATA_BAGGER_STATE,
                                TOPIC_MANAGEMENT_DATA_BAGGER_TOPICS,
                                pub_topic,
                                "RDTDRDTD");

    components_++;
  }

  // ros_disk_state_rapid_disk_state => RDSRDS
  if (!config_params_.GetBool("use_RDSRDS", &use)) {
    ROS_FATAL("DDS Bridge: use RDSRDS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RDSRDS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RDSRDS not specified!");
      return false;
    }

    BuildDiskStateToRapid(TOPIC_MANAGEMENT_DISK_MONITOR_STATE,
                          pub_topic,
                          "RDSRDS");
    components_++;
  }

  // ros_fault_config_rapid_fault_config => RFCRFC
  if (!config_params_.GetBool("use_RFCRFC", &use)) {
    ROS_FATAL("DDS Bridge: use RFCRFC not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RFCRFC", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RFCRFC not specified!");
      return false;
    }

    BuildFaultConfigToRapid(TOPIC_MANAGEMENT_SYS_MONITOR_CONFIG,
                            pub_topic,
                            "RFCRFC");
    components_++;
  }

  // ros_fault_state_rapid_fault_state => RFSRFS
  if (!config_params_.GetBool("use_RFSRFS", &use)) {
    ROS_FATAL("DDS Bridge: use RFSRFS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RFSRFS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RFSRFS not specified!");
      return false;
    }

    BuildFaultStateToRapid(TOPIC_MANAGEMENT_SYS_MONITOR_STATE,
                           pub_topic,
                           "RFSRFS");
    components_++;
  }

  // ros_gnc_control_command_rapid_gnc_control_command => RGCCRGCC
  if (!config_params_.GetBool("use_RGCCRGCC", &use)) {
    ROS_FATAL("DDS Bridge: use RGCCRGCC not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGCCRGCC", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RGCCRGCC not specified!");
      return false;
    }

    BuildGncFamCmdStateToRapid(TOPIC_GNC_CTL_COMMAND, pub_topic, "RGCCRGCC");
    components_++;
  }

  // ros_gnc_control_shaper_rapid_gnc_control_shaper => RGCSRGCS
  if (!config_params_.GetBool("use_RGCSRGCS", &use)) {
    ROS_FATAL("DDS Bridge: use RGCSRGCS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGCSRGCS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RGCSRGCS not specified!");
      return false;
    }

    BuildGncControlStateToRapid(TOPIC_GNC_CTL_SHAPER, pub_topic, "RGCSRGCS");
    components_++;
  }

  // ros_gnc_control_trajectory_rapid_gnc_control_trajectory => RGCTRGCT
  if (!config_params_.GetBool("use_RGCTRGCT", &use)) {
    ROS_FATAL("DDS Bridge: use RGCTRGCT not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGCTRGCT", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RGCTRGCT not specified!");
      return false;
    }

    BuildGncControlStateToRapid(TOPIC_GNC_CTL_TRAJ, pub_topic, "RGCTRGCT");
    components_++;
  }

  // ros_guest_science_rapid_guest_science => RGSRGS
  if (!config_params_.GetBool("use_RGSRGS", &use)) {
    ROS_FATAL("DDS Bridge: use RGSRGS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGSRGS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RGSRGS not specified!");
      return false;
    }

    BuildGuestScienceToRapid(TOPIC_GUEST_SCIENCE_MANAGER_STATE,
                             TOPIC_GUEST_SCIENCE_MANAGER_CONFIG,
                             TOPIC_GUEST_SCIENCE_DATA,
                             pub_topic,
                             "RGSRGS");
    components_++;
  }

  // ros_inertia_rapid_inertia => RIRI
  if (!config_params_.GetBool("use_RIRI", &use)) {
    ROS_FATAL("DDS Bridge: use RIRI not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RIRI", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RIRI not specified!");
      return false;
    }

    BuildInertiaDataToRapid(TOPIC_MOBILITY_INERTIA, pub_topic, "RIRI");

    components_++;
  }

  // ros_log_rapid_log_sample => RLRLS
  if (!config_params_.GetBool("use_RLRLS", &use)) {
    ROS_FATAL("DDS Bridge: use RLRLS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RLRLS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RLSLS not specified!");
      return false;
    }

    // TODO(rgarciar): Change topic name for a constant
    BuildLogSampleToRapid("/rosout",
                           pub_topic,
                           "RLRLS");
    components_++;
  }

  // ros_odom_rapid_position => RORP
  if (!config_params_.GetBool("use_RORP", &use)) {
    ROS_FATAL("DDS Bridge: use RORP not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RORP", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RORP not specified!");
      return false;
    }

    BuildOdomToPosition(TOPIC_GNC_EKF, pub_topic, "RORP");
    components_++;
  }

  // ros_plan_status_rapid_plan_status => RPSRPS
  if (!config_params_.GetBool("use_RPSRPS", &use)) {
    ROS_FATAL("DDS Bridge: use RPSRPS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RPSRPS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RPSRPS not specified!");
      return false;
    }

    BuildPlanStatusToPlanStatus(TOPIC_MANAGEMENT_EXEC_PLAN_STATUS,
                                pub_topic,
                                "RPSRPS");
    components_++;
  }

  // ros_pmc_cmd_state_rapid_pmc_cmd_state => RPCSRPCS
  if (!config_params_.GetBool("use_RPCSRPCS", &use)) {
    ROS_FATAL("DDS Bridge: use RPCSRPCS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RPCSRPCS", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RPCSRPCS not specified!");
      return false;
    }

    BuildPmcCmdStateToRapid(TOPIC_HARDWARE_PMC_COMMAND, pub_topic, "RPCSRPCS");
    components_++;
  }

  // ros_string_rapid_text_message => RSRTM
  if (!config_params_.GetBool("use_RSRTM", &use)) {
    ROS_FATAL("DDS Bridge: use RSRTM not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("sub_topic_RSRTM", &sub_topic)) {
      ROS_FATAL("DDS Bridge: sub topic RSRTM not specified!");
      return false;
    }

    if (!config_params_.GetStr("pub_topic_RSRTM", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RSRTM not specified!");
      return false;
    }

    BuildStringToTextMessage(sub_topic, pub_topic, "RSRTM");
    components_++;
  }

  // ros_telemetry_rapid_telemetry => RTRT
  if (!config_params_.GetBool("use_RTRT", &use)) {
    ROS_FATAL("DDS Bridge: use RTRT not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RTRT", &pub_topic)) {
      ROS_FATAL("DDS Bridge: pub topic RTRT not specified!");
      return false;
    }

    BuildTelemetryToRapid(TOPIC_MANAGEMENT_CAMERA_STATE, pub_topic, "RTRT");
    components_++;
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
