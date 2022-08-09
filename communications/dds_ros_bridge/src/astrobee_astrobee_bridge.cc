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

#include "dds_ros_bridge/astrobee_astrobee_bridge.h"

namespace dds_ros_bridge {
AstrobeeAstrobeeBridge::AstrobeeAstrobeeBridge() :
  ff_util::FreeFlyerNodelet(NODE_ASTROBEE_ASTROBEE_BRIDGE, true),
  components_(0),
  started_(false),
  agent_name_("Bumble"),
  read_params_error_("") {
}

AstrobeeAstrobeeBridge::~AstrobeeAstrobeeBridge() {
}

void AstrobeeAstrobeeBridge::Initialize(ros::NodeHandle *nh) {
  // Need to get robot name which is in the lua config files, add files, read
  // files, and get robot name  but don't get the rest of the config parameters
  // since these params are used to create the classes that use rapid dds. Need
  // to set up Miro/DDs before reading the parameters.

  config_params_.AddFile("communications/astrobee_astrobee_bridge.config");

  if (!config_params_.ReadFiles()) {
    ROS_FATAL("AstrobeeAstrobeeBridge: Error reading config files.");
    exit(EXIT_FAILURE);
    return;
  }

  if (!config_params_.GetBool("started", &run_on_start_)) {
    ROS_FATAL("AstrobeeAstrobeeBridge: Could not read started value.");
    exit(EXIT_FAILURE);
    return;
  }

  if (!config_params_.GetStr("agent_name", &agent_name_)) {
    ROS_FATAL("AstrobeeAstrobeeBridge: Could not read robot name.");
    exit(EXIT_FAILURE);
    return;
  }

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

  nh_ = *nh;

  int fake_argc = 1;

  // TODO(tfmorse): make hardcoded values configurable

  // Make path to QOS and NDDS files
  std::string config_path = ff_common::GetConfigDir();
  config_path += "/communications/dds_intercomms/";

  // Create fake argv containing only the particaptant name
  // Participant name needs to uniue so combine robot name with timestamp
  ros::Time time = ros::Time::now();
  participant_name_ = agent_name_ + std::to_string(time.sec)
     + std::string("-multi-bridge");
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

  // Set values for default publisher and susbcriber
  dds_params->publishers[0].name = agent_name_;
  dds_params->publishers[0].partition = agent_name_;
  dds_params->publishers[0].participant = participant_name_;
  dds_params->subscribers[0].participant = participant_name_;

  // Clear config files so that dds only looks for the files we add
  dds_params->participants[0].discoveryPeersFiles.clear();
  dds_params->configFiles.clear();

  dds_params->participants[0].name = participant_name_;
  dds_params->participants[0].participantName = participant_name_;
  dds_params->participants[0].domainId = 38;
  dds_params->participants[0].discoveryPeersFiles.push_back(
      (config_path + "NDDS_DISCOVERY_PEERS"));
  dds_params->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));

  std::string local_subscriber = Miro::RobotParameters::instance()->name.c_str();

  // Read subscribers from config file
  std::vector<std::string> default_subscribers;
  config_reader::ConfigReader::Table rapid_sub_names;
  std::string rapid_sub;
  if (!config_params_.GetTable("rapid_sub_names", &rapid_sub_names)) {
    ROS_FATAL("Multi-Astrobee Bridge: rapid_sub_names not specified!");
    exit(EXIT_FAILURE);
    return;
  }
  for (int i = 1; i <= rapid_sub_names.GetSize(); i++) {
    rapid_sub_names.GetStr(i, &rapid_sub);
    default_subscribers.push_back(rapid_sub);
  }

  // Register the subscribers into the parameters so they can be used later.
  for (unsigned int i = 0; i < default_subscribers.size(); i++) {
    if (local_subscriber != default_subscribers[i]) {
      // TODO(rgarciar): Use () init instead of new
      kn::DdsNodeParameters *subscriber = new kn::DdsNodeParameters();
      subscriber->name = default_subscribers[i];
      subscriber->partition = default_subscribers[i];
      subscriber->participant = participant_name_;
      dds_params->subscribers.push_back(*subscriber);
    }
  }

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

  trigger_srv_ = nh->advertiseService(
                              SERVICE_COMMUNICATIONS_ENABLE_ASTROBEE_INTERCOMMS,
                              &AstrobeeAstrobeeBridge::Start,
                              this);

  if (run_on_start_) {
    if (!Run()) {
      ROS_ERROR_STREAM(read_params_error_);
    }
  }
}

bool AstrobeeAstrobeeBridge::Start(ff_msgs::ResponseOnly::Request& req,
                                   ff_msgs::ResponseOnly::Response & res) {
  if (!Run()) {
    ROS_ERROR_STREAM(read_params_error_);
    res.success = false;
    res.status = read_params_error_;
  } else {
    res.success = true;
  }

  return true;
}

bool AstrobeeAstrobeeBridge::Run() {
  if (!started_ && !ReadParams()) {
    return false;
  }
  started_ = true;
  return true;
}

bool AstrobeeAstrobeeBridge::ReadParams() {
  config_reader::ConfigReader::Table share_topics, share_topic_groups, agents;
  config_reader::ConfigReader::Table item_conf, agent, topics;
  std::string item_name, topic_name, name, external_name;
  bool enable, topic_enable;
  float rate;

  components_ = 0;

  // Reload files. This allows the config file to be fixed if something is wrong
  // during run time
  if (!config_params_.ReadFiles()) {
    read_params_error_ = "AstrobeeAstrobeeBridge: Error reading config files.";
    return false;
  }
  // ROS -> RAPID
  // -----------------------------
  // Load individual shared topics
  if (!config_params_.GetTable("share_topics", &share_topics)) {
    read_params_error_ =
                  "AstrobeeAstrobeeBridge: share_topics table not specified!";
    return false;
  }

  for (int i = 1; i <= share_topics.GetSize(); i++) {
    share_topics.GetTable(i, &item_conf);
    if (!ReadSharedItemConf(item_conf, item_name, enable, rate)) {
      return false;
    }

    // GNC_EKF configuration
    // ---------------------
    if (item_name == "TOPIC_GNC_EKF" && enable) {
      BuildRosToRapid<ff::RosEkfToRapid>(item_name, TOPIC_GNC_EKF,
                            "", "", nh_);

      if (ros_sub_rapid_pubs_.count(item_name) == 0) {
        read_params_error_ = "AstrobeeAstrobeeBridge: ";
        read_params_error_ += item_name.c_str();
        read_params_error_ += " not added and it is needed.";
        return false;
      }

      if (rate < 0) {
        read_params_error_ = "AstrobeeAstrobeeBridge: ";
        read_params_error_ += item_name.c_str();
        read_params_error_ += " requires a non-negative rate.";
        return false;
      }

      ff::RosEkfToRapid *ekf_to_rapid = static_cast<ff::RosEkfToRapid *>
                                        (ros_sub_rapid_pubs_[item_name].get());
      ekf_to_rapid->SetEkfPublishRate(rate);
    }

    // Additional topics here...
    // -------------------------
  }

  // Load shared topic groups
  if (!config_params_.GetTable("share_topic_groups", &share_topic_groups)) {
    read_params_error_ =
              "AstrobeeAstrobeeBridge: share_topic_groups table not specified!";
    return false;
  }

  for (int i = 1; i <= share_topic_groups.GetSize(); i++) {
    share_topic_groups.GetTable(i, &item_conf);
    if (!ReadSharedItemConf(item_conf, item_name, enable, rate)) {
      return false;
    }

    // Guest Sciece GROUP
    // ------------------
    if (item_name == "TOPIC_GROUP_GS" && enable) {
      BuildRosToRapid<ff::RosGuestScienceToRapid>(item_name,
                                            TOPIC_GUEST_SCIENCE_MANAGER_STATE,
                                            TOPIC_GUEST_SCIENCE_MANAGER_CONFIG,
                                            TOPIC_GUEST_SCIENCE_DATA, "", nh_);
    }

    // Additional topics here...
    // -------------------------
  }

  // RAPID -> ROS
  // ----------------------
  // Read listeners options
  if (!config_params_.GetTable("agents", &agents)) {
    read_params_error_ = "AstrobeeAstrobeeBridge: agents not specified!";
    return false;
  }

  for (int i = 1; i <= agents.GetSize(); i++) {
    // Agent configuration
    agents.GetTable(i, &item_conf);
    if (!item_conf.GetStr("name", &item_name)) {
      read_params_error_ = "AstrobeeAstrobeeBridge: agent name not specified!";
      return false;
    }
    if (!item_conf.GetBool("enable", &enable)) {
      read_params_error_ =
                          "AstrobeeAstrobeeBridge: agent enable not specified!";
      return false;
    }
    if (!item_conf.GetTable("topics", &topics)) {
      read_params_error_ =
                          "AstrobeeAstrobeeBridge: agent topics not specified!";
      return false;
    }

    // Skip if the configuration is for this agent.
    // We don't want to translate topics coming from this robot already
    // Also ignore if the agent is disabled
    if (agent_name_ == item_name || !enable) {
      continue;
    }

    // Lower case the external agent name to use it like a namespace
    external_name = item_name;
    external_name[0] = tolower(external_name[0]);

    for (int j = 1; j <= topics.GetSize(); j++) {
      // Agent topic configuration
      topics.GetTable(j, &item_conf);
      if (!item_conf.GetStr("name", &topic_name)) {
        read_params_error_ =
                      "AstrobeeAstrobeeBridge: agent topic name not specified!";
        return false;
      }
      if (!item_conf.GetBool("enable", &topic_enable)) {
        read_params_error_ =
                    "AstrobeeAstrobeeBridge: agent topic enable not specified!";
        return false;
      }

      if (topic_name == "TOPIC_GNC_EKF" && topic_enable) {
        BuildRapidToRos<ff::RapidEkfToRos>(
                            rapid::ext::astrobee::EKF_STATE_TOPIC,
                            item_name,
                            external_name + '/' + std::string(TOPIC_GNC_EKF),
                            nh_);
      }

      if (topic_name == "TOPIC_GUEST_SCIENCE_DATA" && topic_enable) {
        BuildRapidToRos<ff::RapidGuestScienceDataToRos>(
                    rapid::ext::astrobee::GUEST_SCIENCE_DATA_TOPIC,
                    item_name,
                    external_name + '/' + std::string(TOPIC_GUEST_SCIENCE_DATA),
                    nh_);
      }

      // Additional topics here...
    }
  }
  return true;
}

bool AstrobeeAstrobeeBridge::ReadSharedItemConf(
                          config_reader::ConfigReader::Table &conf,
                          std::string &topic_name, bool &enable, float &rate) {
  if (!conf.GetStr("name", &topic_name)) {
    read_params_error_ =
                      "AstrobeeAstrobeeBridge: share topic name not specified!";
    return false;
  }
  if (!conf.GetBool("enable", &enable)) {
    read_params_error_ =
                    "AstrobeeAstrobeeBridge: share topic enable not specified!";
    return false;
  }
  if (!conf.GetReal("rate", &rate)) {
    read_params_error_ =
                      "AstrobeeAstrobeeBridge: ekf state rate not specified!";
    return false;
  }

  return true;
}

template<typename T, typename... Args>
int AstrobeeAstrobeeBridge::BuildRosToRapid(const std::string& name,
                                                              Args&&... args) {  // NOLINT
  ff::RosSubRapidPubPtr topic_to_rapid(new T(std::forward<Args>(args)...));  // NOLINT
  components_++;
  ros_sub_rapid_pubs_[name] = topic_to_rapid;
  return ros_sub_rapid_pubs_.size();
}

template<typename T, typename ... Args>
int AstrobeeAstrobeeBridge::BuildRapidToRos(Args&& ... args) {  // NOLINT
  ff::RapidSubRosPubPtr topic_to_ros(new T(std::forward<Args>(args)...));  // NOLINT
  components_++;
  rapid_sub_ros_pubs_.push_back(topic_to_ros);
  return rapid_sub_ros_pubs_.size();
}

}   // end namespace dds_ros_bridge

PLUGINLIB_EXPORT_CLASS(dds_ros_bridge::AstrobeeAstrobeeBridge, nodelet::Nodelet)
