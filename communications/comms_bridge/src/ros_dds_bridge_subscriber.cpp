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

#include "comms_bridge/ros_dds_bridge_subscriber.h"

#include <string>

ROSDDSBridgeSubscriber::ROSDDSBridgeSubscriber(std::string agent_name) {
  agent_name_ = agent_name;


  int fake_argc = 1;
  // Create fake argv containing only the participant name
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

  // Miro::RobotParameters *robot_params = Miro::RobotParameters::instance();
  // kn::DdsEntitiesFactorySvcParameters *dds_params =
  //     kn::DdsEntitiesFactorySvcParameters::instance();

  // /* get the defaults for *all the things!* */
  // Miro::ConfigDocument *config = Miro::Configuration::document();
  // config->setSection("Robot");
  // config->getParameters("Miro::RobotParameters", *robot_params);
  // config->getParameters("kn::DdsEntitiesFactorySvcParameters", *dds_params);

  // robot_params->name = agent_name_;
  // robot_params->namingContextName = robot_params->name;

  // // Set values for default publisher and susbcriber
  // dds_params->publishers[0].name = agent_name_;
  // dds_params->publishers[0].partition = agent_name_;
  // dds_params->publishers[0].participant = participant_name_;
  // dds_params->subscribers[0].participant = participant_name_;

  // // Clear config files so that dds only looks for the files we add
  // dds_params->participants[0].discoveryPeersFiles.clear();
  // dds_params->configFiles.clear();

  // dds_params->participants[0].name = participant_name_;
  // dds_params->participants[0].participantName = participant_name_;
  // dds_params->participants[0].domainId = 38;
  // dds_params->participants[0].discoveryPeersFiles.push_back(
  //     (config_path + "NDDS_DISCOVERY_PEERS"));
  // dds_params->configFiles.push_back((config_path + "RAPID_QOS_PROFILES.xml"));


  // /**
  //  * Use DdsEntitiesFactorySvc to create a new DdsEntitiesFactory
  //  * which will create all objects:
  //  *    Participants   DdsDomainParticpantRepository::instance()
  //  *    Publishers     DdsPublisherRespoitory::instance()
  //  *    Subscribers    DdsSubscriberRepository::instance()
  //  *    Topics
  //  * and store in relevant repository
  //  * based on DdsEntitesFactoryParameters
  //  */
  // dds_entities_factory_.reset(new kn::DdsEntitiesFactorySvc());
  // dds_entities_factory_->init(dds_params);

  // trigger_srv_ = nh->advertiseService(
  //                             SERVICE_COMMUNICATIONS_ENABLE_BRIDGE_SUBSCRIBER,
  //                             &AstrobeeAstrobeeBridge::Start,
  //                             this);
}

ROSDDSBridgeSubscriber::~ROSDDSBridgeSubscriber() {
  // FIXME: any needed cleanup
}

// Called with the mutex held
void ROSDDSBridgeSubscriber::subscribeTopic(std::string const& in_topic, const RelayTopicInfo& info) {
  // FIXME: DDS stuff here
}

// Called with the mutex held
void ROSDDSBridgeSubscriber::advertiseTopic(const RelayTopicInfo& info) {
  // FIXME: DDS stuff here
}

// Called with the mutex held
void ROSDDSBridgeSubscriber::relayMessage(const RelayTopicInfo& topic_info, ContentInfo const& content_info) {
  // FIXME: DDS stuff here
}
