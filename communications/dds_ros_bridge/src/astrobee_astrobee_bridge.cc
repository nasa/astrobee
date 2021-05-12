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
  agent_name_("Bumble") {
}

AstrobeeAstrobeeBridge::~AstrobeeAstrobeeBridge() {
}


int AstrobeeAstrobeeBridge::BuildEkfToRapid(const std::string& sub_topic,
                                      const std::string& pub_topic,
                                      const std::string& rapid_pub_name,
                                      const std::string& name) {
  ff::RosSubRapidPubPtr ekf_to_rapid(new ff::RosEkfToRapid(sub_topic,
                                                           pub_topic,
                                                           rapid_pub_name,
                                                           nh_));
  ros_sub_rapid_pubs_[name] = ekf_to_rapid;
  return ros_sub_rapid_pubs_.size();
}

int AstrobeeAstrobeeBridge::BuildEkfToRos(const std::string& sub_topic,
                    const std::string& sub_partition,
                    const std::string& pub_topic,
                    const std::string& name) {
  ff::RapidSubRosPubPtr rapid_ekf_to_ros(new ff::RapidEkfToRos(sub_topic,
                                                                        sub_partition,
                                                                        pub_topic,
                                                                        nh_));

  rapid_sub_ros_pubs_.push_back(rapid_ekf_to_ros);
  return rapid_sub_ros_pubs_.size();
}

void AstrobeeAstrobeeBridge::Initialize(ros::NodeHandle *nh) {
  // Need to get robot name which is in the lua config files, add files, read
  // files, and get robot name  but don't get the rest of the config parameters
  // since these params are used to create the classes that use rapid dds. Need
  // to set up Miro/DDs before reading the parameters.

  config_params_.AddFile("communications/astrobee_astrobee_bridge.config");

  if (!config_params_.ReadFiles()) {
    ROS_FATAL("Multi-Astrobee Bridge: Error reading config files.");
    exit(EXIT_FAILURE);
    return;
  }

  if (!config_params_.GetStr("agent_name", &agent_name_)) {
    ROS_FATAL("Multi-Astrobee Bridge: Could not read robot name.");
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
  config_path += "/communications/dds/";

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
  dds_params->publishers[0].name = agent_name_ + std::string("multi-bridge");
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

  if (!ReadParams()) {
    exit(EXIT_FAILURE);
    return;
  }
}

bool AstrobeeAstrobeeBridge::ReadParams() {
  config_reader::ConfigReader::Table share_topics, agents, agent, topics, topic;
  std::string name, foreign_name, topic_name;
  bool enable, topic_enable;
  float rate;

  components_ = 0;

  if (!config_params_.GetTable("share_topics", &share_topics)) {
    ROS_FATAL("Multi-Astrobee Bridge: share topics not specified!");
    return false;
  }

  for (int i = 1; i <= share_topics.GetSize(); i++) {
    share_topics.GetTable(i, &topic);
    if (!topic.GetStr("name", &topic_name)) {
      ROS_FATAL("Multi-Astrobee Bridge: share topic name not specified!");
      return false;
    }
    if (!topic.GetBool("enable", &enable)) {
      ROS_FATAL("Multi-Astrobee Bridge: share topic enable not specified!");
      return false;
    }
    if (!topic.GetReal("rate", &rate)) {
      ROS_FATAL("Multi-Astrobee Bridge: ekf state rate not specified!");
      return false;
    }

    if (enable) {
      BuildEkfToRapid(TOPIC_GNC_EKF, "", agent_name_
        + std::string("multi-bridge"), "GNC_EKF");
      components_++;

      if (ros_sub_rapid_pubs_.count("GNC_EKF") == 0) {
        ROS_ERROR("Multi-Astrobee Bridge: Ekf stuff not added and it is needed!");
        return false;
      }
      ff::RosEkfToRapid *RERE = static_cast<ff::RosEkfToRapid *>
                                        (ros_sub_rapid_pubs_["GNC_EKF"].get());

      RERE->SetEkfPublishRate(rate);

      // Add additional topics here...
    }
  }

  // Read listeners options
  if (!config_params_.GetTable("agents", &agents)) {
    ROS_FATAL("Multi-Astrobee Bridge: agents not specified!");
    return false;
  }

  for (int i = 1; i <= agents.GetSize(); i++) {
    agents.GetTable(i, &agent);

    if (!agent.GetStr("name", &name)) {
      ROS_FATAL("Multi-Astrobee Bridge: agent name not specified!");
      return false;
    }
    if (!agent.GetBool("enable", &enable)) {
      ROS_FATAL("Multi-Astrobee Bridge: agent enable not specified!");
      return false;
    }
    if (!agent.GetTable("topics", &topics)) {
      ROS_FATAL("Multi-Astrobee Bridge: agent topics not specified!");
      return false;
    }
    if (enable) {
      for (int j = 1; j <= topics.GetSize(); j++) {
        topics.GetTable(j, &topic);
        if (!topic.GetStr("name", &topic_name)) {
          ROS_FATAL("Multi-Astrobee Bridge: agent topic name not specified!");
          return false;
        }
        if (!topic.GetBool("enable", &topic_enable)) {
          ROS_FATAL("Multi-Astrobee Bridge: agent topic enable not specified!");
          return false;
        }

        foreign_name = name;
        foreign_name[0] = tolower(foreign_name[0]);

        if (topic_name == "GNC_EKF" && topic_enable && agent_name_ != name) {
          BuildEkfToRos(rapid::ext::astrobee::EKF_STATE_TOPIC,
                                            name,
                                            foreign_name + '/' + std::string(TOPIC_GNC_EKF),
                                            foreign_name + std::string("_GNC_EKF"));
        }
        components_++;

        // Add additional topics here...
      }
    }
  }
  return true;
}

}   // end namespace dds_ros_bridge

PLUGINLIB_EXPORT_CLASS(dds_ros_bridge::AstrobeeAstrobeeBridge, nodelet::Nodelet)
