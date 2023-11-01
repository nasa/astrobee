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

#include <string>

#include "comms_bridge/ros_dds_bridge_subscriber.h"

namespace comms_bridge {
// void usage(const char *progname)
// {
//   fprintf(stderr, "Usage: %s [-v<verbose>] [-a<anonymous node>] [-t meta_topic_prefix] <topic to subscribe to> <topic
//   to republish on remote side> [<subscribe topic> <republish topic> ...]\n", progname);
// }

class BridgeSubscriberNodelet : public ff_util::FreeFlyerNodelet {
 public:
  BridgeSubscriberNodelet() : ff_util::FreeFlyerNodelet("comms_bridge_sub") {}

  virtual ~BridgeSubscriberNodelet() {}

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

    bool verbose;
    if (!config_params_.GetBool("verbose", &verbose)) {
      ROS_FATAL("BridgeSubscriberNodelet: Could not read verbosity level.");
      exit(EXIT_FAILURE);
      return;
    }

    ROS_ERROR_STREAM("ROBOT NAME " << agent_name_);
    // Declare the ROS to DDS Subscriber class
    ROSDDSBridgeSubscriber sub(agent_name_);
    std::string ns = "/" + std::tolower(agent_name_[0]) + agent_name_.substr(1) + "/";

    // Load shared topic groups
    config_reader::ConfigReader::Table links, link, relay_forward, relay_backward, relay_both, item_conf;
    if (!config_params_.GetTable("links", &links)) {
      ROS_FATAL("BridgeSubscriberNodelet: Links not specified!");
      return;
    }

    // Need to search for the collision distance in the mapper parameters
    for (int i = 1; i <= links.GetSize(); i++) {
      if (!links.GetTable(i, &link)) {
        NODELET_ERROR("Could not read the mapper parameter table row %i", i);
        continue;
      }
      std::string config_agent, topic_name;
      if (link.GetStr("from", &config_agent) && config_agent == agent_name_) {
        if (link.GetTable("relay_forward", &relay_forward)) {
          for (int j = 1; j <= relay_forward.GetSize(); j++) {
            // Agent topic configuration
            relay_forward.GetTable(j, &item_conf);
            if (!item_conf.GetStr("name", &topic_name)) {
              NODELET_ERROR("BridgeSubscriberNodelet: agent topic name not specified!");
              return;
            }
            assert(sub.addTopic(topic_name, ns + topic_name));
          }
        }
        if (link.GetTable("relay_both", &relay_both)) {
          for (int j = 1; j <= relay_both.GetSize(); j++) {
            // Agent topic configuration
            relay_both.GetTable(j, &item_conf);
            if (!item_conf.GetStr("name", &topic_name)) {
              NODELET_ERROR("BridgeSubscriberNodelet: agent topic name not specified!");
              return;
            }
            assert(sub.addTopic(topic_name, ns + topic_name));
          }
        }
      } else if (link.GetStr("to", &config_agent) && config_agent == agent_name_) {
        if (link.GetTable("relay_backward", &relay_backward)) {
          for (int j = 1; j <= relay_backward.GetSize(); j++) {
            // Agent topic configuration
            relay_backward.GetTable(j, &item_conf);
            if (!item_conf.GetStr("name", &topic_name)) {
              NODELET_ERROR("BridgeSubscriberNodelet: agent topic name not specified!");
              return;
            }
            assert(sub.addTopic(topic_name, ns + topic_name));
          }
        }
        if (link.GetTable("relay_both", &relay_both)) {
          for (int j = 1; j <= relay_both.GetSize(); j++) {
            // Agent topic configuration
            relay_both.GetTable(j, &item_conf);
            if (!item_conf.GetStr("name", &topic_name)) {
              NODELET_ERROR("BridgeSubscriberNodelet: agent topic name not specified!");
              return;
            }
            assert(sub.addTopic(topic_name, ns + topic_name));
          }
        }
      }
    }
  }

 private:
  config_reader::ConfigReader config_params_;
  std::string agent_name_;
};

PLUGINLIB_EXPORT_CLASS(comms_bridge::BridgeSubscriberNodelet, nodelet::Nodelet)

}  // namespace comms_bridge
