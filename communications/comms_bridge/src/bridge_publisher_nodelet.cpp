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

#include "comms_bridge/dds_ros_bridge_publisher.h"


namespace comms_bridge {

class BridgePublisherNodelet : public ff_util::FreeFlyerNodelet {
 public:
  BridgePublisherNodelet() : ff_util::FreeFlyerNodelet("comms_bridge_pub") {}

  virtual ~BridgePublisherNodelet() {}

 protected:
  virtual void Initialize(ros::NodeHandle* nh) {
    config_params_.AddFile("communications/comms_bridge.config");

    if (!config_params_.ReadFiles()) {
      ROS_FATAL("BridgePublisherNodelet: Error reading config files.");
      exit(EXIT_FAILURE);
      return;
    }

    unsigned int  verbose;
    if (!config_params_.GetUInt("verbose", &verbose)) {
      ROS_FATAL("BridgePublisherNodelet: Could not read verbosity level.");
      exit(EXIT_FAILURE);
      return;
    }

    double ad2pub_delay = 3.0;
    if (!config_params_.GetReal("ad2pub_delay", &ad2pub_delay)) {
      ROS_FATAL("BridgePublisherNodelet: Could not read advertiser to publisher delay.");
      exit(EXIT_FAILURE);
      return;
    }

    DDSROSBridgePublisher pub(ad2pub_delay);
    // if (verbose > 0)
    //   pub.setVerbosity(verbose);
  }

 private:
  config_reader::ConfigReader config_params_;
};

PLUGINLIB_EXPORT_CLASS(comms_bridge::BridgePublisherNodelet, nodelet::Nodelet)

}  // namespace comms_bridge
