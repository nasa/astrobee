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

#include "ground_dds_ros_bridge/ground_dds_ros_bridge.h"

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

namespace ground_dds_ros_bridge {

GroundDdsRosBridge::GroundDdsRosBridge() {
  components_ = 0;

  agent_name_ = "ISAAC Ground Bridge";
}

GroundDdsRosBridge::~GroundDdsRosBridge() {
}

bool GroundDdsRosBridge::BuildSensorImageToCompressedImage(
                                                  const std::string& pub_topic,
                                                  const std::string& name) {
  std::string sub_topic;
  bool use;

  if (ReadTopicInfo(name, "sub", sub_topic, use)) {
    if (use) {
      ff::RapidSubRosPubPtr sensor_image_to_compressed_image(
                                new ff::RapidImageRosCompressedImage(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      rapid_sub_ros_pubs_.push_back(sensor_image_to_compressed_image);
    }
  } else {
    return false;
  }

  return true;
}

bool GroundDdsRosBridge::Initialize(ros::NodeHandle *nh) {
  config_params_.AddFile("communications/ground_dds_ros_bridge.config");

  if (!config_params_.ReadFiles()) {
    ROS_FATAL("DDS Bridge: Error reading config files.");
    return false;
  }

  nh_ = *nh;

  int fake_argc = 1;

  // TODO(kmhamil1): make hardcoded values configurable

  // Make path to QOS and NDDS files
  std::string config_path = ff_common::GetConfigDir();
  config_path += "/communications/dds/";

  // Create fake argv containing only the particaptant name
  // Participant name needs to unique so combine bridge name with timestamp
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

  Miro::RobotParameters *ground_params = Miro::RobotParameters::instance();
  kn::DdsEntitiesFactorySvcParameters *dds_params =
      kn::DdsEntitiesFactorySvcParameters::instance();

  /* get the defaults for *all the things!* */
  Miro::ConfigDocument *config = Miro::Configuration::document();
  config->setSection("Robot");
  config->getParameters("Miro::RobotParameters", *ground_params);
  config->getParameters("kn::DdsEntitiesFactorySvcParameters", *dds_params);

  ground_params->name = agent_name_;
  ground_params->namingContextName = ground_params->name;

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
    return false;
  }
}

bool GroundDdsRosBridge::ReadTopicInfo(const std::string& topic_abbr,
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

bool GroundDdsRosBridge::ReadParams() {
  components_ = 0;

  // rapid_image_ros_compressed_science_cam_image => RIRCSCI
  if (!BuildCompressedImageToImage(TOPIC_HARDWARE_SCI_CAM, "RIRCSCI")) {
    return false;
  }

  return true;
}

}   // end namespace ground_dds_ros_bridge
