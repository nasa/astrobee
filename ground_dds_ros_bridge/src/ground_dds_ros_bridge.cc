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

  connecting_robot_ = "";
}

GroundDdsRosBridge::~GroundDdsRosBridge() {
}

/********************** Rapid Sub Ros Pub Functions ***************************/
bool GroundDdsRosBridge::BuildAccessControlStateToRos(
                                                  const std::string& pub_topic,
                                                  const std::string& name) {
  std::string sub_topic;
  bool use;

  if (ReadTopicInfo(name, "sub", sub_topic, use)) {
    if (use) {
      ff::RapidSubRosPubPtr access_control_state_to_access_control_state(
                                new ff::RapidAccessControlStateToRos(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      rapid_sub_ros_pubs_.push_back(access_control_state_to_access_control_state);
    }
  } else {
    return false;
  }

  return true;
}

bool GroundDdsRosBridge::BuildGuestScienceDataToRos(
                                                  const std::string& pub_topic,
                                                  const std::string& name) {
  std::string sub_topic;
  bool use;

  if (ReadTopicInfo(name, "sub", sub_topic, use)) {
    if (use) {
      ff::RapidSubRosPubPtr guest_science_data_to_guest_science_data(
                                  new ff::RapidGuestScienceDataToRos(sub_topic,
                                                                     pub_topic,
                                                                     nh_));
      components_++;
      rapid_sub_ros_pubs_.push_back(guest_science_data_to_guest_science_data);
    }
  } else {
    return false;
  }

  return true;
}


bool GroundDdsRosBridge::BuildSensorImageToRos(const std::string& pub_topic,
                                               const std::string& name) {
  std::string sub_topic;
  bool use;

  if (ReadTopicInfo(name, "sub", sub_topic, use)) {
    if (use) {
      ff::RapidSubRosPubPtr sensor_image_to_compressed_image(
                                             new ff::RapidImageToRos(sub_topic,
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

/********************* Ros Sub Rapid Pub Functions ****************************/
bool GroundDdsRosBridge::BuildCommandToRapid(const std::string& sub_topic,
                                             const std::string& name) {
  std::string pub_topic;
  bool use;

  if (ReadTopicInfo(name, "pub", pub_topic, use)) {
    if (use) {
      ff::RosSubRapidPubPtr command_to_command(
                                    new ff::RosCommandToRapid(sub_topic,
                                                              pub_topic,
                                                              connecting_robot_,
                                                              nh_));
      components_++;
      ros_sub_rapid_pubs_.push_back(command_to_command);
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

  // Need to get the connecting robot name and domain id first for the dds setup
  // All other configuration parameters will be read in after dds is setup.
  // Get the name of the robot the ground bridge is trying to connect to.
  if (!config_params_.GetStr("connecting_robot", &connecting_robot_)) {
    ROS_FATAL("Ground bridge: connecting robot not specified!");
    return false;
  }

  // Get the domain id which will be different in space than on the ground
  if (!config_params_.GetInt("domain_id", &domain_id_)) {
    ROS_FATAL("Ground bridge: domain id not specified!");
    return false;
  }

  nh_ = *nh;

  int fake_argc = 1;

  // TODO(kmhamil1): change this to not use freeflyer tools

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

  // Add robot that we need to listen to
  ground_params->teamMembers.push_back(connecting_robot_);

  SubstituteROBOT_NAME(dds_params);
  
  // Set values for default publisher and subscriber
/*  dds_params->publishers[0].name = agent_name_;
  dds_params->publishers[0].partition = agent_name_;
  dds_params->publishers[0].participant = participant_name_;

  dds_params->subscribers[0].name = connecting_robot_;
  dds_params->subscribers[0].partition = connecting_robot_;
  dds_params->subscribers[0].participant = participant_name_;


  kn::DdsNodeParameters *subscriber = new kn::DdsNodeParameters();
  subscriber->name = connecting_robot_;
  subscriber->partition = connecting_robot_;
  subscriber->participant = participant_name_;
  dds_params->subscribers.push_back(*subscriber);
*/

  // Clear config files so that dds only looks for the files we add
  dds_params->participants[0].discoveryPeersFiles.clear();
  dds_params->configFiles.clear();

  dds_params->participants[0].participantName = participant_name_;
  dds_params->participants[0].domainId = domain_id_;
  dds_params->participants[0].discoveryPeersFiles.push_back(
      (config_path + "NDDS_DISCOVERY_PEERS"));
  dds_params->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));

  dds_params->subscribers[0].partition = connecting_robot_;
  dds_params->publishers[0].partition = connecting_robot_;

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

  return true;
}

bool GroundDdsRosBridge::ReadTopicInfo(const std::string& topic_abbr,
                                 const std::string& sub_or_pub,
                                 std::string& topic,
                                 bool& use) {
  std::string use_exp = "use_" + topic_abbr;
  std::string topic_exp = sub_or_pub + "_topic_" + topic_abbr;

  if (!config_params_.GetBool(use_exp.c_str(), &use)) {
    ROS_FATAL("Ground bridge: use %s not specified!", topic_abbr.c_str());
    return false;
  }

  if (use) {
    if (!config_params_.GetStr(topic_exp.c_str(), &topic)) {
      ROS_FATAL("Ground bridge: sub topic %s not specified!",
                                                            topic_abbr.c_str());
      return false;
    }
  }

  return true;
}

bool GroundDdsRosBridge::ReadParams() {
  std::string ns = "/";
  bool use_namespace = false;
  components_ = 0;

  // Get boolean that signifies if the namespace needs to be the robot name
  config_params_.GetBool("use_namespace", &use_namespace);
  if (use_namespace) {
    // First character needs to be uppercase for dds but lower case for fsw
    std::string lowercase_robot = connecting_robot_;
    lowercase_robot[0] = std::tolower(lowercase_robot[0]);
    ns += lowercase_robot + "/";
  }

  // rapid_access_control_state_ros_access_control_state = RACSRACS
  if (!BuildAccessControlStateToRos(
                                  (ns + TOPIC_MANAGEMENT_ACCESS_CONTROL_STATE),
                                  "RACSRACS")) {
    return false;
  }

  // rapid_guest_science_data_ros_guest_science_data => RGSDRGSD
  if (!BuildGuestScienceDataToRos((ns + TOPIC_GUEST_SCIENCE_DATA),
                                  "RGSDRGSD")) {
    return false;
  }

  // rapid_image_ros_compressed_science_cam_image => RIRCSCI
  if (!BuildSensorImageToRos((ns + TOPIC_HARDWARE_SCI_CAM), "RIRCSCI")) {
    return false;
  }

  // rapid_image_ros_compressed_nav_cam_image => RIRCNCI
  if (!BuildSensorImageToRos((ns + TOPIC_HARDWARE_NAV_CAM), "RIRCNCI")) {
    return false;
  }

  // rapid_image_ros_compressed_dock_cam_image => RIRCDCI
  if (!BuildSensorImageToRos((ns + TOPIC_HARDWARE_DOCK_CAM), "RIRCDCI")) {
    return false;
  }

  // ros_command_rapid_command => RCRC
  if (!BuildCommandToRapid((ns + TOPIC_COMMAND), "RCRC")) {
    return false;
  }

  return true;
}

}   // end namespace ground_dds_ros_bridge
