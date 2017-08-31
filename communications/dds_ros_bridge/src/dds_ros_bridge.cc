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

#include <string>
#include <vector>

#include "dds_ros_bridge/dds_ros_bridge.h"
#include "dds_ros_bridge/rapid_command_ros_command_plan.h"
#include "dds_ros_bridge/rapid_compressed_file_ros_compressed_file.h"
#include "dds_ros_bridge/ros_access_control.h"
#include "dds_ros_bridge/ros_ack.h"
#include "dds_ros_bridge/ros_agent_state.h"
#include "dds_ros_bridge/ros_arm_joint_sample.h"
#include "dds_ros_bridge/ros_arm_state.h"
#include "dds_ros_bridge/ros_battery_state.h"
#include "dds_ros_bridge/ros_command_config_rapid_command_config.h"
#include "dds_ros_bridge/ros_compressed_file_rapid_compressed_file.h"
#include "dds_ros_bridge/ros_compressed_file_ack.h"
#include "dds_ros_bridge/ros_compressed_image_rapid_image.h"
#include "dds_ros_bridge/ros_cpu_state.h"
#include "dds_ros_bridge/ros_disk_state.h"
#include "dds_ros_bridge/ros_fault_state.h"
#include "dds_ros_bridge/ros_fault_config.h"
#include "dds_ros_bridge/ros_gnc_fam_cmd_state.h"
#include "dds_ros_bridge/ros_gnc_control_state.h"
#include "dds_ros_bridge/ros_guest_science.h"
#include "dds_ros_bridge/ros_odom_rapid_position.h"
#include "dds_ros_bridge/ros_plan_status_rapid_plan_status.h"
#include "dds_ros_bridge/ros_string_rapid_text_message.h"
#include "dds_ros_bridge/ros_telemetry_rapid_telemetry.h"

/*miro includes*/
#include "miro/Configuration.h"
#include "miro/Robot.h"
#include "miro/Log.h"

/*SoraCore Includes*/
#include "knDds/DdsSupport.h"
#include "knDds/DdsEntitiesFactory.h"
#include "knDds/DdsEntitiesFactorySvc.h"

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
  robot_name_("Bumble") {
}

DdsRosBridge::~DdsRosBridge() {
}

int DdsRosBridge::BuildAccessControlStateToRapid(const std::string& subTopic,
                                                 const std::string& pubTopic,
                                                 const std::string& name) {
  ff::RosSubRapidPubPtr acsToAcs(new ff::RosAccessControlStateToRapid(subTopic,
                                                                      pubTopic,
                                                                      nh_));
  m_rosSubRapidPubs_[name] = acsToAcs;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildAckToRapid(const std::string& subTopic,
                                  const std::string& pubTopic,
                                  const std::string& name) {
  ff::RosSubRapidPubPtr ackToAck(new ff::RosAckToRapid(subTopic,
                                                       pubTopic,
                                                       nh_));
  m_rosSubRapidPubs_[name] = ackToAck;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildAgentStateToRapid(const std::string& subTopic,
                                         const std::string& pubTopic,
                                         const std::string& name) {
  ff::RosSubRapidPubPtr agentToAgent(new ff::RosAgentStateToRapid(subTopic,
                                                                  pubTopic,
                                                                  nh_));
  m_rosSubRapidPubs_[name] = agentToAgent;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildArmJointSampleToRapid(const std::string& subTopic,
                                             const std::string& pubTopic,
                                             const std::string& name) {
  ff::RosSubRapidPubPtr jointToJoint(new ff::RosArmJointSampleToRapid(subTopic,
                                                                      pubTopic,
                                                                      nh_));
  m_rosSubRapidPubs_[name] = jointToJoint;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildArmStateToRapid(const std::string& subTopic,
                                       const std::string& pubTopic,
                                       const std::string& name) {
  ff::RosSubRapidPubPtr armToArm(new ff::RosArmStateToRapid(subTopic,
                                                            pubTopic,
                                                            nh_));
  m_rosSubRapidPubs_[name] = armToArm;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildBatteryStateToRapid(
                                      const std::string& subTopicBatteryStateTL,
                                      const std::string& subTopicBatteryStateTR,
                                      const std::string& subTopicBatteryStateBL,
                                      const std::string& subTopicBatteryStateBR,
                                      const std::string& subTopicBatteryTempTL,
                                      const std::string& subTopicBatteryTempTR,
                                      const std::string& subTopicBatteryTempBL,
                                      const std::string& subTopicBatteryTempBR,
                                      const std::string& pubTopic,
                                      const std::string& name) {
  ff::RosSubRapidPubPtr batteryToBattery(
                          new ff::RosBatteryStateToRapid(subTopicBatteryStateTL,
                                                         subTopicBatteryStateTR,
                                                         subTopicBatteryStateBL,
                                                         subTopicBatteryStateBR,
                                                         subTopicBatteryTempTL,
                                                         subTopicBatteryTempTR,
                                                         subTopicBatteryTempBL,
                                                         subTopicBatteryTempBR,
                                                         pubTopic, nh_));
  m_rosSubRapidPubs_[name] = batteryToBattery;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildCompressedImageToImage(const std::string& subTopic,
                                              const std::string& pubTopic,
                                              const std::string& name) {
  ff::RosSubRapidPubPtr compressedImageToImage(
                new ff::RosCompressedImageRapidImage(subTopic, pubTopic, nh_));
  m_rosSubRapidPubs_[name] = compressedImageToImage;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildCpuStateToRapid(const std::string& subTopic,
                                       const std::string& pubTopic,
                                       const std::string& name) {
  ff::RosSubRapidPubPtr cpuToCpu(new ff::RosCpuStateToRapid(subTopic,
                                                            pubTopic,
                                                            nh_));
  m_rosSubRapidPubs_[name] = cpuToCpu;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildDiskStateToRapid(const std::string& subTopic,
                                        const std::string& pubTopic,
                                        const std::string& name) {
  ff::RosSubRapidPubPtr diskToDisk(new ff::RosDiskStateToRapid(subTopic,
                                                               pubTopic,
                                                               nh_));
  m_rosSubRapidPubs_[name] = diskToDisk;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildFaultConfigToRapid(const std::string& subTopic,
                                          const std::string& pubTopic,
                                          const std::string& name) {
  ff::RosSubRapidPubPtr faultToFault(new ff::RosFaultConfigToRapid(subTopic,
                                                                   pubTopic,
                                                                   nh_));
  m_rosSubRapidPubs_[name] = faultToFault;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildFaultStateToRapid(const std::string& subTopic,
                                         const std::string& pubTopic,
                                         const std::string& name) {
  ff::RosSubRapidPubPtr faultToFault(new ff::RosFaultStateToRapid(subTopic,
                                                                  pubTopic,
                                                                  nh_));
  m_rosSubRapidPubs_[name] = faultToFault;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildGncFamCmdStateToRapid(const std::string& subTopic,
                                             const std::string& pubTopic,
                                             const std::string& name) {
  ff::RosSubRapidPubPtr gncFamCmdToGncFamCmd(new ff::RosGncFamCmdStateToRapid(
                                                                      subTopic,
                                                                      pubTopic,
                                                                      nh_));
  m_rosSubRapidPubs_[name] = gncFamCmdToGncFamCmd;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildGncControlStateToRapid(const std::string& subTopic,
                                              const std::string& pubTopic,
                                              const std::string& name) {
  ff::RosSubRapidPubPtr gncControlToGncControl(
                                    new ff::RosGncControlStateToRapid(subTopic,
                                                                      pubTopic,
                                                                      nh_));
  m_rosSubRapidPubs_[name] = gncControlToGncControl;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildGuestScienceToRapid(const std::string& stateSubTopic,
                                           const std::string& configSubTopic,
                                           const std::string& dataSubTopic,
                                           const std::string& pubTopic,
                                           const std::string& name) {
  ff::RosSubRapidPubPtr guestScienceToGuestScience(
                            new ff::RosGuestScienceToRapid(stateSubTopic,
                                                           configSubTopic,
                                                           dataSubTopic,
                                                           pubTopic,
                                                           nh_));
  m_rosSubRapidPubs_[name] = guestScienceToGuestScience;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildOdomToPosition(const std::string& subTopic,
                                      const std::string& pubTopic,
                                      const std::string& name) {
  ff::RosSubRapidPubPtr odomToPosition(new ff::RosOdomRapidPosition(subTopic,
                                                                    pubTopic,
                                                                    nh_));
  m_rosSubRapidPubs_[name] = odomToPosition;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildPlanStatusToPlanStatus(const std::string& subTopic,
                                              const std::string& pubTopic,
                                              const std::string& name) {
  ff::RosSubRapidPubPtr planStatusToPlanStatus(
                new ff::RosPlanStatusRapidPlanStatus(subTopic, pubTopic, nh_));
  m_rosSubRapidPubs_[name] = planStatusToPlanStatus;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildStringToTextMessage(const std::string& subTopic,
                                           const std::string& pubTopic,
                                           const std::string& name) {
  ff::RosSubRapidPubPtr strToText(new ff::RosStringRapidTextMessage(subTopic,
                                                                    pubTopic,
                                                                    nh_));
  m_rosSubRapidPubs_[name] = strToText;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildTelemetryToRapid(const std::string& subTopic,
                                        const std::string& pubTopic,
                                        const std::string& name) {
  ff::RosSubRapidPubPtr telemetryToTelemetry(
    new ff::RosTelemetryRapidTelemetry(subTopic,
                                       pubTopic,
                                       nh_,
                                       config_params_));
  m_rosSubRapidPubs_[name] = telemetryToTelemetry;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildCommandToCommand(const std::string& subTopic,
                                        const std::string& pubTopic,
                                        const std::string& name) {
  ff::RapidSubRosPubPtr commandToCommand(
                       new ff::RapidCommandRosCommand(subTopic, pubTopic, nh_));
  m_rapidSubRosPubs_.push_back(commandToCommand);
  return m_rapidSubRosPubs_.size();
}

int DdsRosBridge::BuildCompressedFileToCompressedFile(
                                                    const std::string& subTopic,
                                                    const std::string& pubTopic,
                                                    const std::string& name) {
  ff::RapidSubRosPubPtr compressedFileToCompressedFile(
        new ff::RapidCompressedFileRosCompressedFile(subTopic, pubTopic, nh_));
  m_rapidSubRosPubs_.push_back(compressedFileToCompressedFile);
  return m_rapidSubRosPubs_.size();
}

int DdsRosBridge::BuildCompressedFileToRapid(const std::string& subTopic,
                                             const std::string& pubTopic,
                                             const std::string& name) {
  ff::RosSubRapidPubPtr compressedFileToRapid(
          new ff::RosCompressedFileToRapid(subTopic, pubTopic, nh_));
  m_rosSubRapidPubs_[name] = compressedFileToRapid;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildCompressedFileAckToRapid(const std::string& subTopic,
                                                const std::string& pubTopic,
                                                const std::string& name) {
  ff::RosSubRapidPubPtr compressedFileAckToRapid(
          new ff::RosCompressedFileAckToRapid(subTopic, pubTopic, nh_));
  m_rosSubRapidPubs_[name] = compressedFileAckToRapid;
  return m_rosSubRapidPubs_.size();
}

int DdsRosBridge::BuildCommandConfigToCommandConfig(const std::string& pubTopic,
                                                    const std::string& name) {
  // keep in scope to retain reliable durable
  ff::RapidPubPtr commandConfigToCommandConfig(
    new ff::RosCommandConfigRapidCommandConfig(pubTopic, nh_, config_params_));
  m_rapidPubs_.push_back(commandConfigToCommandConfig);
  return m_rapidPubs_.size();
}

void DdsRosBridge::Initialize(ros::NodeHandle *nh) {
  // Need to get robot name which is in the lua config files, add files, read
  // files, and get robot name  but don't get the rest of the config parameters
  // since these params are used to create the classes that use rapid dds. Need
  // to set up Miro/DDs before reading the parameters.
  config_params_.AddFile("commands.config");
  config_params_.AddFile("context.config");
  config_params_.AddFile("communications/dds_ros_bridge.config");

  if (!config_params_.ReadFiles()) {
    ROS_FATAL("DDS Bridge: Error reading config files.");
    exit(EXIT_FAILURE);
    return;
  }

  if (!config_params_.GetStr("robot_name", &robot_name_)) {
    ROS_FATAL("DDS Bridge: Could not read robot name.");
    exit(EXIT_FAILURE);
    return;
  }

  // In simulation, the namespace is usually set to the robot name so we need to
  // check if we are in simulation and get the right name
  if (robot_name_ == "sim" || robot_name_ == "simulator") {
    // The platform name should be the simulated robot name
    robot_name_ = GetPlatform();

    // If there is not robot name, set it to a default name so that we can
    // connect to the bridge
    if (robot_name_ == "") {
      robot_name_ = "Bumble";
    }
  }

  // Make sure that first letter of robot name is capitialized. GDS only
  // recognizes capitialized robot names.
  robot_name_[0] = toupper(robot_name_[0]);

  nh_ = *nh;

  int fakeArgc = 1;

  // TODO(tfmorse): make hardcoded values configurable

  // Make path to QOS and NDDS files
  std::string config_path = common::GetConfigDir();
  config_path += "/communications/dds/";

  // Create fake argv containing only the particaptant name
  // Participant name needs to uniue so combine robot name with timestamp
  ros::Time time = ros::Time::now();
  participant_name_ = robot_name_ + std::to_string(time.sec);
  char **fakeArgv = new char*[1];
  fakeArgv[0] = new char[(participant_name_.size() + 1)];
  std::strcpy(fakeArgv[0], participant_name_.c_str());  // NOLINT

  /* fake miro log into thinking we have no arguments */
  Miro::Log::init(fakeArgc, fakeArgv);
  Miro::Log::level(9);

  /* fake miro configuration into thinking we have no arguments */
  Miro::Configuration::init(fakeArgc, fakeArgv);

  Miro::RobotParameters *robotParams = Miro::RobotParameters::instance();
  kn::DdsEntitiesFactorySvcParameters *ddsParams =
      kn::DdsEntitiesFactorySvcParameters::instance();

  /* get the defaults for *all the things!* */
  Miro::ConfigDocument *config = Miro::Configuration::document();
  config->setSection("Robot");
  config->getParameters("Miro::RobotParameters", *robotParams);
  config->getParameters("kn::DdsEntitiesFactorySvcParameters", *ddsParams);

  robotParams->name = robot_name_;
  robotParams->namingContextName = robotParams->name;

  SubstituteROBOT_NAME(ddsParams);

  // Clear config files so that dds only looks for the files we add
  ddsParams->participants[0].discoveryPeersFiles.clear();
  ddsParams->configFiles.clear();

  ddsParams->participants[0].participantName = participant_name_;
  ddsParams->participants[0].domainId = 37;
  ddsParams->participants[0].discoveryPeersFiles.push_back(
      (config_path + "NDDS_DISCOVERY_PEERS"));
  ddsParams->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));

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
  m_ddsEntitiesFactory_.reset(new kn::DdsEntitiesFactorySvc());
  m_ddsEntitiesFactory_->init(ddsParams);

  if (!ReadParams()) {
    exit(EXIT_FAILURE);
    return;
  }
}

bool DdsRosBridge::ReadParams() {
  std::string subTopic, subTopic2;
  std::string pubTopic;
  bool use;

  components_ = 0;

  // rapid_command_ros_command => RCRC
  if (!config_params_.GetBool("use_RCRC", &use)) {
    ROS_FATAL("DDS Bridge: use RCRC not specified");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("sub_topic_RCRC", &subTopic)) {
      ROS_FATAL("DDS Bridge: sub topic RCRC not specified!");
      return false;
    }

    BuildCommandToCommand(subTopic, TOPIC_COMMUNICATIONS_DDS_COMMAND, "RCRC");
    components_++;
  }

  // rapid_compressed_file_plan_ros_compressed_file_plan => RCFPRCFP
  if (!config_params_.GetBool("use_RCFPRCFP", &use)) {
    ROS_FATAL("DDS Bridge: use RCFPRCFP not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("sub_topic_RCFPRCFP", &subTopic)) {
      ROS_FATAL("DDS Bridge: sub topic RCFPRCFP not specified!");
      return false;
    }

    BuildCompressedFileToCompressedFile(subTopic, TOPIC_COMMUNICATIONS_DDS_PLAN,
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
    if (!config_params_.GetStr("sub_topic_RCFZRCFZ", &subTopic)) {
      ROS_FATAL("DDS Bridge: sub topic RCFZRCFZ not specified!");
      return false;
    }

    BuildCompressedFileToCompressedFile(subTopic,
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
    if (!config_params_.GetStr("pub_topic_RACS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RACS not specified!");
      return false;
    }

    BuildAccessControlStateToRapid(TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE,
                                   pubTopic, "RACS");
    components_++;
  }

  // ros_ack_rapid_ack => RARA
  if (!config_params_.GetBool("use_RARA", &use)) {
    ROS_FATAL("DDS Bridge: use RARA not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RARA", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RARA not specified!");
      return false;
    }

    BuildAckToRapid(TOPIC_MANAGEMENT_ACK, pubTopic, "RARA");
    components_++;
  }

  // ros_agent_state_rapid_agent_state => RASRAS
  if (!config_params_.GetBool("use_RASRAS", &use)) {
    ROS_FATAL("DDS Bridge: use RASRAS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RASRAS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RASRAS not specified!");
      return false;
    }

    BuildAgentStateToRapid(TOPIC_MANAGEMENT_EXEC_AGENT_STATE, pubTopic,
                           "RASRAS");
    components_++;
  }

  // ros_arm_joint_sample_rapid_arm_joint_sample => RAJSRAJS
  if (!config_params_.GetBool("use_RAJSRAJS", &use)) {
    ROS_FATAL("DDS Bridge: use RAJSRAJS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RAJSRAJS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RAJSRAJS not specified!");
      return false;
    }

    BuildArmJointSampleToRapid(TOPIC_HARDWARE_PERCHING_ARM_JOINT_SAMPLE,
                                                          pubTopic, "RAJSRAJS");
    components_++;
  }

  // ros_arm_state_rapid_arm_state => RARMRARM
  if (!config_params_.GetBool("use_RARMRARM", &use)) {
    ROS_FATAL("DDS Bridge: use RARMRARM not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RARMRARM", &pubTopic)) {
      ROS_FATAL("DDS_Bridge: pub topic RARMRARM not specified!");
      return false;
    }

    BuildArmStateToRapid(TOPIC_HARDWARE_PERCHING_ARM_STATE, pubTopic,
                                                                    "RARMRARM");
    components_++;
  }

  // ros_battery_state_rapid_battery_config => RBSRBS
  if (!config_params_.GetBool("use_RBSRBS", &use)) {
    ROS_FATAL("DDS Bridge: pub topic RBSRBS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RBSRBS", &pubTopic)) {
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
                             pubTopic,
                             "RBSRBS");
    components_++;
  }

  // ros_command_config_rapid_command_config => RCCRCC
  if (!config_params_.GetBool("use_RCCRCC", &use)) {
    ROS_FATAL("DDS Bridge: use RCCRCC not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCCRCC", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RCCRCC not specified!");
      return false;
    }

    BuildCommandConfigToCommandConfig(pubTopic, "RCCRCC");
    components_++;
  }

  // ros_compressed_file_rapid_compressed_file => RoCFRaCF
  if (!config_params_.GetBool("use_ROSCFRAPCF", &use)) {
    ROS_FATAL("DDS Bridge: use ROSCFRAPCF not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_ROSCFRAPCF", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic ROSCFRAPCF not specified!");
      return false;
    }

    BuildCompressedFileToRapid(TOPIC_MANAGEMENT_EXEC_PLAN, pubTopic,
                               "ROSCFRAPCF");
    components_++;
  }

  // ros_compressed_file_ack_rapid_compressed_file_ack => RCFARCFA
  if (!config_params_.GetBool("use_RCFARCFA", &use)) {
    ROS_FATAL("DDS Bridge: use RCFARCFA not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCFARCFA", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RCFARCFA not specified!");
      return false;
    }

    BuildCompressedFileAckToRapid(TOPIC_MANAGEMENT_EXEC_CF_ACK, pubTopic,
                                  "RCFARCFA");
    components_++;
  }

  // ros_compressed_dock_cam_image_rapid_image => RCDCIRI
  if (!config_params_.GetBool("use_RCDCIRI", &use)) {
    ROS_FATAL("DDS Bridge: use RCDCIRI not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCDCIRI", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RCDCIRI not specified!");
      return false;
    }

    BuildCompressedImageToImage(TOPIC_MANAGEMENT_IMG_SAMPLER_DOCK_CAM_STREAM,
                                pubTopic, "RCDCIRI");
    components_++;
  }

  // ros_compressed_nav_cam_image_rapid_image => RCNCIRI
  if (!config_params_.GetBool("use_RCNCIRI", &use)) {
    ROS_FATAL("DDS Bridge: use RCNCIRI not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCNCIRI", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RCNCIRI not specified!");
      return false;
    }

    BuildCompressedImageToImage(TOPIC_MANAGEMENT_IMG_SAMPLER_NAV_CAM_STREAM,
                                pubTopic, "RCNCIRI");
    components_++;
  }

  // ros_cpu_state_rapid_cpu_state => RCSRCS
  if (!config_params_.GetBool("use_RCSRCS", &use)) {
    ROS_FATAL("DDS Bridge: use RCSRCS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RCSRCS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RCSRCS not specified!");
      return false;
    }

    BuildCpuStateToRapid(TOPIC_MANAGEMENT_CPU_MONITOR_STATE, pubTopic,
                                                                      "RCSRCS");
    components_++;
  }

  // ros_disk_state_rapid_disk_state => RDSRDS
  if (!config_params_.GetBool("use_RDSRDS", &use)) {
    ROS_FATAL("DDS Bridge: use RDSRDS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RDSRDS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RDSRDS not specified!");
      return false;
    }

    BuildDiskStateToRapid(TOPIC_MANAGEMENT_DISK_MONITOR_STATE, pubTopic,
                          "RDSRDS");
    components_++;
  }

  // ros_fault_config_rapid_fault_config => RFCRFC
  if (!config_params_.GetBool("use_RFCRFC", &use)) {
    ROS_FATAL("DDS Bridge: use RFCRFC not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RFCRFC", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RFCRFC not specified!");
      return false;
    }

    BuildFaultConfigToRapid(TOPIC_MANAGEMENT_SYS_MONITOR_CONFIG, pubTopic,
                            "RFCRFC");
    components_++;
  }

  // ros_fault_state_rapid_fault_state => RFSRFS
  if (!config_params_.GetBool("use_RFSRFS", &use)) {
    ROS_FATAL("DDS Bridge: use RFSRFS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RFSRFS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RFSRFS not specified!");
      return false;
    }

    BuildFaultStateToRapid(TOPIC_MANAGEMENT_SYS_MONITOR_STATE, pubTopic,
                           "RFSRFS");
    components_++;
  }

  // ros_gnc_control_command_rapid_gnc_control_command => RGCCRGCC
  if (!config_params_.GetBool("use_RGCCRGCC", &use)) {
    ROS_FATAL("DDS Bridge: use RGCCRGCC not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGCCRGCC", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RGCCRGCC not specified!");
      return false;
    }

    BuildGncFamCmdStateToRapid(TOPIC_GNC_CTL_COMMAND, pubTopic, "RGCCRGCC");
    components_++;
  }

  // ros_gnc_control_shaper_rapid_gnc_control_shaper => RGCSRGCS
  if (!config_params_.GetBool("use_RGCSRGCS", &use)) {
    ROS_FATAL("DDS Bridge: use RGCSRGCS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGCSRGCS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RGCSRGCS not specified!");
      return false;
    }

    BuildGncControlStateToRapid(TOPIC_GNC_CTL_SHAPER, pubTopic, "RGCSRGCS");
    components_++;
  }

  // ros_gnc_control_trajectory_rapid_gnc_control_trajectory => RGCTRGCT
  if (!config_params_.GetBool("use_RGCTRGCT", &use)) {
    ROS_FATAL("DDS Bridge: use RGCTRGCT not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGCTRGCT", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RGCTRGCT not specified!");
      return false;
    }

    BuildGncControlStateToRapid(TOPIC_GNC_CTL_TRAJ, pubTopic, "RGCTRGCT");
    components_++;
  }

  // ros_guest_science_rapid_guest_science => RGSRGS
  if (!config_params_.GetBool("use_RGSRGS", &use)) {
    ROS_FATAL("DDS Bridge: use RGSRGS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RGSRGS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RGSRGS not specified!");
      return false;
    }

    BuildGuestScienceToRapid(TOPIC_GUEST_SCIENCE_MANAGER_STATE,
                             TOPIC_GUEST_SCIENCE_MANAGER_CONFIG,
                             TOPIC_GUEST_SCIENCE_DATA,
                             pubTopic,
                             "RGSRGS");
    components_++;
  }

  // ros_odom_rapid_position => RORP
  if (!config_params_.GetBool("use_RORP", &use)) {
    ROS_FATAL("DDS Bridge: use RORP not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RORP", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RORP not specified!");
      return false;
    }

    BuildOdomToPosition(TOPIC_GNC_EKF, pubTopic, "RORP");
    components_++;
  }

  // ros_plan_status_rapid_plan_status => RPSRPS
  if (!config_params_.GetBool("use_RPSRPS", &use)) {
    ROS_FATAL("DDS Bridge: use RPSRPS not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RPSRPS", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RPSRPS not specified!");
      return false;
    }

    BuildPlanStatusToPlanStatus(TOPIC_MANAGEMENT_EXEC_PLAN_STATUS, pubTopic,
                                "RPSRPS");
    components_++;
  }

  // ros_string_rapid_text_message => RSRTM
  if (!config_params_.GetBool("use_RSRTM", &use)) {
    ROS_FATAL("DDS Bridge: use RSRTM not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("sub_topic_RSRTM", &subTopic)) {
      ROS_FATAL("DDS Bridge: sub topic RSRTM not specified!");
      return false;
    }

    if (!config_params_.GetStr("pub_topic_RSRTM", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RSRTM not specified!");
      return false;
    }

    BuildStringToTextMessage(subTopic, pubTopic, "RSRTM");
    components_++;
  }

  // ros_telemetry_rapid_telemetry => RTRT
  if (!config_params_.GetBool("use_RTRT", &use)) {
    ROS_FATAL("DDS Bridge: use RTRT not specified!");
    return false;
  }

  if (use) {
    if (!config_params_.GetStr("pub_topic_RTRT", &pubTopic)) {
      ROS_FATAL("DDS Bridge: pub topic RTRT not specified!");
      return false;
    }

    BuildTelemetryToRapid(TOPIC_MANAGEMENT_CAMERA_STATE, pubTopic, "RTRT");
    components_++;
  }

  // Read in the telemetery rates so the bridge knows how often to publish
  // the telemmetry messages
  float temp_rate;
  if (m_rosSubRapidPubs_.count("RTRT") == 0) {
    ROS_ERROR("DDS Bridge: Telemetry msg stuff not added and it is needed!");
    return false;
  }
  ff::RosTelemetryRapidTelemetry *RTRT =
                                  static_cast<ff::RosTelemetryRapidTelemetry *>
                                  (m_rosSubRapidPubs_["RTRT"].get());

  if (m_rosSubRapidPubs_.count("RCSRCS") == 0) {
    ROS_ERROR("DDS Bridge: Cpu state stuff not added and it is needed!");
    return false;
  }
  ff::RosCpuStateToRapid *RCSR = static_cast<ff::RosCpuStateToRapid *>
                                          (m_rosSubRapidPubs_["RCSRCS"].get());

  if (m_rosSubRapidPubs_.count("RDSRDS") == 0) {
    ROS_ERROR("DDS Bridge: Disk state stuff not added and it is needed!");
    return false;
  }
  ff::RosDiskStateToRapid *RDSR = static_cast<ff::RosDiskStateToRapid *>
                                          (m_rosSubRapidPubs_["RDSRDS"].get());

  if (m_rosSubRapidPubs_.count("RGCCRGCC") == 0) {
    ROS_ERROR("DDS Bridge: GNC control command stuff not added and is needed!");
    return false;
  }
  ff::RosGncFamCmdStateToRapid *RGCC =
                                    static_cast<ff::RosGncFamCmdStateToRapid *>
                                    (m_rosSubRapidPubs_["RGCCRGCC"].get());

  if (m_rosSubRapidPubs_.count("RGCSRGCS") == 0) {
    ROS_ERROR("DDS Bridge: GNC control shaper stuff not added and is needed!");
    return false;
  }
  ff::RosGncControlStateToRapid *RGCS =
                                    static_cast<ff::RosGncControlStateToRapid *>
                                    (m_rosSubRapidPubs_["RGCSRGCS"].get());

  if (m_rosSubRapidPubs_.count("RGCTRGCT") == 0) {
    ROS_ERROR("DDS Bridge: GNC control trajectory not added and it is needed!");
    return false;
  }
  ff::RosGncControlStateToRapid *RGCT =
                                    static_cast<ff::RosGncControlStateToRapid *>
                                    (m_rosSubRapidPubs_["RGCTRGCT"].get());

  if (m_rosSubRapidPubs_.count("RORP") == 0) {
    ROS_ERROR("DDS Bridge: Odometry stuff not added and it is needed!");
    return false;
  }
  ff::RosOdomRapidPosition *RORP = static_cast<ff::RosOdomRapidPosition *>
                                            (m_rosSubRapidPubs_["RORP"].get());

  if (!config_params_.GetReal("comm_status_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: comm state rate not specified!");
    return false;
  }
  RTRT->SetCommStatusRate(temp_rate);

  if (!config_params_.GetReal("cpu_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: cpu state rate not specified!");
    return false;
  }
  RTRT->SetCpuStateRate(temp_rate);
  RCSR->SetPublishRate(temp_rate);

  if (!config_params_.GetReal("disk_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: disk state rate not specified!");
    return false;
  }
  RTRT->SetDiskStateRate(temp_rate);
  RDSR->SetPublishRate(temp_rate);

  if (!config_params_.GetReal("ekf_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: ekf state rate not specified!");
    return false;
  }
  RTRT->SetEkfStateRate(temp_rate);
  RORP->SetEkfPublishRate(temp_rate);

  if (!config_params_.GetReal("gnc_state_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: gnc state rate not specified!");
    return false;
  }
  RTRT->SetGncStateRate(temp_rate);
  RGCC->SetGncPublishRate(temp_rate);
  RGCS->SetGncPublishRate(temp_rate);
  RGCT->SetGncPublishRate(temp_rate);

  if (!config_params_.GetReal("position_rate", &temp_rate)) {
    ROS_FATAL("DDS Bridge: position rate not specified!");
    return false;
  }
  RTRT->SetPositionRate(temp_rate);
  RORP->SetPositionPublishRate(temp_rate);

  // Read in the multiple that the battery estimated time remaining needs to be
  // rounded to.
  int multiple;
  if (!config_params_.GetInt("battery_time_round_to_multiple", &multiple)) {
    ROS_FATAL("DDS Bridge: Battery time round to multiple not specified!");
    return false;
  }

  if (m_rosSubRapidPubs_.count("RBSRBS") == 0) {
    ROS_ERROR("DDS Bridge: Battery msg stuff not added and it is needed!");
    return false;
  }

  ff::RosBatteryStateToRapid *RBSRBS = static_cast<ff::RosBatteryStateToRapid *>
                                          (m_rosSubRapidPubs_["RBSRBS"].get());

  RBSRBS->SetBatteryTimeMultiple(multiple);

  return true;
}

}   // end namespace dds_ros_bridge

PLUGINLIB_EXPORT_CLASS(dds_ros_bridge::DdsRosBridge, nodelet::Nodelet)
