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

// Standard ROS includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// FSW shared libraries
#include <config_reader/config_reader.h>

// FSW nodelet
#include <ff_common/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <stdio.h>
#include <getopt.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "comms_bridge/generic_rapid_msg_ros_pub.h"
#include "comms_bridge/generic_rapid_sub.h"
#include "comms_bridge/generic_ros_sub_rapid_pub.h"
#include "comms_bridge/rapid_sub_advertisement_info.h"
#include "comms_bridge/rapid_sub_content.h"
#include "comms_bridge/rapid_sub_request.h"

#include "dds_msgs/AstrobeeConstants.h"

#include "ff_msgs/ResponseOnly.h"

// SoraCore
#include "knDds/DdsSupport.h"
#include "knDds/DdsEntitiesFactory.h"
#include "knDds/DdsEntitiesFactorySvc.h"
#include "knDds/DdsTypedSupplier.h"

// miro
#include "miro/Configuration.h"
#include "miro/Robot.h"
#include "miro/Log.h"

namespace kn {
  class DdsEntitiesFactorySvc;
}  // end namespace kn

namespace comms_bridge {

class CommsBridgeNodelet : public ff_util::FreeFlyerNodelet {
 public:
  CommsBridgeNodelet() : ff_util::FreeFlyerNodelet("comms_bridge"),
                         dds_initialized_(false),
                         initialize_dds_on_start_(false),
                         enable_advertisement_info_request_(false) {}

  virtual ~CommsBridgeNodelet() {}

 protected:
  virtual void Initialize(ros::NodeHandle* nh) {
    // Need to get robot name which is in the lua config files, add files, read
    // files, and get robot name  but don't get the rest of the config parameters
    // since these params are used to create the classes that use rapid dds. Need
    // to set up Miro/DDs before reading the parameters.

    config_params_.AddFile("communications/comms_bridge.config");

    if (!config_params_.ReadFiles()) {
      ROS_FATAL("BridgeSubscriberNodelet: Error reading config files.");
      exit(EXIT_FAILURE);
      return;
    }

    if (!config_params_.GetStr("agent_name", &agent_name_)) {
      ROS_FATAL("BridgeSubscriberNodelet: Could not read robot name.");
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
    ROS_INFO_STREAM("Comms Bridge Nodelet: agent name " << agent_name_);

    ros_sub_ = std::make_shared<ff::GenericROSSubRapidPub>();

    int fake_argc = 1;

    // Make path to QOS and NDDS files
    std::string config_path = ff_common::GetConfigDir();
    config_path += "/communications/dds_generic_comms/";

    // Create fake argv containing only the participant name
    // Participant name needs to be unique so combine robot name with timestamp
    ros::Time time = ros::Time::now();
    participant_name_ = agent_name_ + std::to_string(time.sec) + std::string("-comms-bridge");
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

    ROS_INFO("Agent name %s and participant name %s\n", agent_name_.c_str(), participant_name_.c_str());

    // Set values for default punlisher and subscriber
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

    if (!ReadParams()) {
      exit(EXIT_FAILURE);
      return;
    }

    // Register the connections into the parameters so they can be used later
    for (int i = 0; i < rapid_connections_.size(); i++) {
      // This shouldn't be needed but check just in case
      if (local_subscriber != rapid_connections_[i]) {
        kn::DdsNodeParameters subscriber;
        subscriber.name = rapid_connections_[i];
        subscriber.partition = rapid_connections_[i];
        subscriber.participant = participant_name_;
        dds_params->subscribers.push_back(subscriber);
      }
    }

    /**
     * Use DdsEntitiesFactorySvc to create a new DdsEntitiesFactory
     * which will create all objects:
     *    Participants   DdsDomainParticipantRepository::instance()
     *    Publishers     DdsPublisherRespository::instance()
     *    Subscribers    DdsSubscriberRepository::instance()
     *    Topics
     * and store in relevant repository
     * based on DdsEntitiesFactoryParameters
     */
    dds_entities_factory_.reset(new kn::DdsEntitiesFactorySvc());
    dds_entities_factory_->init(dds_params);

    dds_initialize_srv_ = nh->advertiseService(
                              SERVICE_COMMUNICATIONS_ENABLE_ASTROBEE_INTERCOMMS,
                              &CommsBridgeNodelet::StartDDS,
                              this);

    if (initialize_dds_on_start_) {
      InitializeDDS();
    }
  }

  bool StartDDS(ff_msgs::ResponseOnly::Request& req,
                ff_msgs::ResponseOnly::Response& res) {
    if (!dds_initialized_) {
      InitializeDDS();
    }
    res.success = true;
    return true;
  }

  void InitializeDDS() {
    std::string connection, dds_topic_name;
    ff::GenericRapidSubPtr rapid_sub;
    ros_sub_->InitializeDDS(rapid_connections_);
    if (enable_advertisement_info_request_) {
      ros_pub_->InitializeDDS(rapid_connections_,
                              enable_advertisement_info_request_);
    }
    for (size_t i = 0; i < rapid_connections_.size(); i++) {
      // Lower case the external agent name to use it like a namespace
      connection = rapid_connections_[i];
      dds_topic_name = agent_name_ + "-" +
          rapid::ext::astrobee::GENERIC_COMMS_ADVERTISEMENT_INFO_TOPIC;
      ROS_INFO("Comms Bridge: DDS Sub DDS advertisement info topic name: %s\n",
               dds_topic_name.c_str());
      rapid_sub = std::make_shared<ff::RapidSubAdvertisementInfo>(
                                "AstrobeeGenericCommsAdvertisementInfoProfile",
                                dds_topic_name,
                                connection,
                                ros_pub_.get());
      rapid_subs_.push_back(rapid_sub);

      dds_topic_name = agent_name_ + "-" +
                       rapid::ext::astrobee::GENERIC_COMMS_CONTENT_TOPIC;
      ROS_INFO("Comms Bridge: DDS Sub DDS content topic name: %s\n",
               dds_topic_name.c_str());
      rapid_sub = std::make_shared<ff::RapidSubContent>(
                                          "AstrobeeGenericCommsContentProfile",
                                          dds_topic_name,
                                          connection,
                                          ros_pub_.get());
      rapid_subs_.push_back(rapid_sub);

      if (enable_advertisement_info_request_) {
        dds_topic_name = agent_name_ + "-" +
            rapid::ext::astrobee::GENERIC_COMMS_REQUEST_TOPIC;
        ROS_INFO("Comms Bridge: DDS Sub DDS request topic name: %s\n",
                 dds_topic_name.c_str());
        rapid_sub = std::make_shared<ff::RapidSubRequest>(
                                          "AstrobeeGenericCommsRequestProfile",
                                          dds_topic_name,
                                          connection,
                                          ros_sub_.get());
        rapid_subs_.push_back(rapid_sub);
      }
    }
    ros_sub_->AddTopics(link_entries_);
    dds_initialized_ = true;
  }

  bool ReadParams() {
    double ad2pub_delay = 0;
    if (!config_params_.GetReal("ad2pub_delay", &ad2pub_delay) ||
        ad2pub_delay <= 0) {
      NODELET_ERROR("Comms Bridge Nodelet: Could not read/or invalid ad2pub_delay. Setting to 3.");
      ad2pub_delay = 3;
    }
    ros_pub_ = std::make_shared<ff::GenericRapidMsgRosPub>(ad2pub_delay);

    unsigned int verbose = 2;
    if (!config_params_.GetUInt("verbose", &verbose)) {
      NODELET_ERROR("Comms Bridge Nodelet: Could not read verbosity level. Setting to 2 (info?).");
    }
    ros_sub_->setVerbosity(verbose);
    ros_pub_->setVerbosity(verbose);

    initialize_dds_on_start_ = false;
    if (!config_params_.GetBool("initialize_dds_on_start",
                                &initialize_dds_on_start_)) {
      NODELET_ERROR("Comms Bridge Nodelet: Could not read initialize dds on start. Setting to false.");
    }

    enable_advertisement_info_request_ = false;
    if (!config_params_.GetBool("enable_advertisement_info_request",
                                &enable_advertisement_info_request_)) {
      NODELET_ERROR("Comms Bridge Nodelet: Could not read enable advertisement info request. Setting to false.");
    }

    // Load shared topic groups
    config_reader::ConfigReader::Table links, link;
    if (!config_params_.GetTable("links", &links)) {
      ROS_FATAL("Comms Bridge Nodelet: Links not specified!");
      return false;
    }

    std::string ns = std::string("/") + agent_name_ + "/";
    ns[1] = std::tolower(ns[1]);  // namespaces don't start with upper case
    int num_topics = 0;
    ROS_INFO_STREAM("Read Params numebr of links: " << links.GetSize());
    for (int i = 1; i <= links.GetSize(); i++) {
      if (!links.GetTable(i, &link)) {
        NODELET_ERROR("Comms Bridge Nodelet: Could read link table row %i", i);
        continue;
      }
      std::string config_agent, connection_agent;
      num_topics = 0;
      ROS_INFO_STREAM("Link " << i << " from " << link.GetStr("from", &config_agent));
      ROS_INFO_STREAM("Link " << i << " to " << link.GetStr("to", &config_agent));
      if (link.GetStr("from", &config_agent) && config_agent == agent_name_) {
        if (AddRapidConnections(link, "to", connection_agent)) {
          AddTableToSubs(link, "relay_forward", ns, connection_agent, num_topics);
          AddTableToSubs(link, "relay_both", ns, connection_agent, num_topics);
        }
      } else if (link.GetStr("to", &config_agent) && config_agent == agent_name_) {
        if (AddRapidConnections(link, "from", connection_agent)) {
          AddTableToSubs(link, "relay_backward", ns, connection_agent, num_topics);
          AddTableToSubs(link, "relay_both", ns, connection_agent, num_topics);
        }
      }

      // Check to make sure the number of topics added doesn't exceed the
      // the number of messages dds reliably delivers
      if (num_topics > 20) {
        ROS_ERROR("Comms bridge: Num of added topics is greater than the number of topics dds will reliably deliver.");
      }
    }
    return true;
  }

  bool AddRapidConnections(config_reader::ConfigReader::Table &link_table,
                           std::string const& direction,
                           std::string &connection) {
    if (!link_table.GetStr(direction.c_str(), &connection)) {
      NODELET_ERROR("Comms Bridge Nodelet: %s not specified for one link", direction.c_str());
      return false;
    }

    // This should be very quick since we shouldn't have more than 2 connections
    bool found = false;
    for (size_t i = 0; i < rapid_connections_.size() && !found; i++) {
      if (connection == rapid_connections_[i]) {
        found = true;
      }
    }

    if (!found) {
      rapid_connections_.push_back(connection);
    }
    return true;
  }

  void AddTableToSubs(config_reader::ConfigReader::Table &link_table,
                      std::string table_name,
                      std::string const& current_robot_ns,
                      std::string const& connection_robot,
                      int &num_topics) {
    config_reader::ConfigReader::Table relay_table, relay_item;
    std::string in_topic, out_topic;
    if (link_table.GetTable(table_name.c_str(), &relay_table)) {
      num_topics += relay_table.GetSize();
      for (size_t i = 1; i <= relay_table.GetSize(); i++) {
        relay_table.GetTable(i, &relay_item);
        if (!relay_item.GetStr("in_topic", &in_topic)) {
           NODELET_ERROR("Comms Bridge Nodelet: In topic not specified!");
            continue;
        }

        if (!relay_item.GetStr("out_topic", &out_topic)) {
          out_topic = current_robot_ns + in_topic;
        }

        // Save all output topics under the same in topic since we don't want
        // to subscribe to the same topic multiple times
        link_entries_[in_topic].push_back(std::make_pair(connection_robot,
                                                         out_topic));
      }
    }
  }

 private:
  bool initialize_dds_on_start_, dds_initialized_;
  bool enable_advertisement_info_request_;
  config_reader::ConfigReader config_params_;

  std::vector<ff::GenericRapidSubPtr> rapid_subs_;
  std::vector<std::string> rapid_connections_;

  std::shared_ptr<kn::DdsEntitiesFactorySvc> dds_entities_factory_;
  std::shared_ptr<ff::GenericRapidMsgRosPub> ros_pub_;
  std::shared_ptr<ff::GenericROSSubRapidPub> ros_sub_;

  std::string agent_name_, participant_name_;
  std::map<std::string, std::vector<std::pair<std::string, std::string>>> link_entries_;
  ros::ServiceServer dds_initialize_srv_;
};

PLUGINLIB_EXPORT_CLASS(comms_bridge::CommsBridgeNodelet, nodelet::Nodelet)

}  // namespace comms_bridge
