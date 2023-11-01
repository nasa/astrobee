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

// void usage(const char *progname)
// {
//   fprintf(stderr, "Usage: %s [-v<verbose>] [-a<anonymous node>] [-d <delay sec>] [-t meta_topic_prefix]\n",
//   progname);
// }

class BridgePublisherNodelet : public ff_util::FreeFlyerNodelet {
 public:
  BridgePublisherNodelet() : ff_util::FreeFlyerNodelet("comms_bridge_pub") {}

  virtual ~BridgePublisherNodelet() {}

 protected:
  virtual void Initialize(ros::NodeHandle* nh) {
    // bool anonymous_node = false;
    // unsigned int verbose = 0;
    // std::string meta_topic_prefix = "/polymorph_relay";
    // double ad2pub_delay = 3.0;

    // int c;
    // while ((c = getopt(argc, argv, "avd:t:")) != -1) {
    //   switch (c) {
    //     case 'a':
    //       anonymous_node = true;
    //       break;
    //     case 'v':
    //       verbose++;
    //       break;
    //     case 'd':
    //       ad2pub_delay = atof(optarg);
    //       if (ad2pub_delay < 0 || std::isnan(ad2pub_delay)) {
    //         fprintf(stderr, "Invalid advertise delay %s\n", optarg);
    //         usage(argv[0]);
    //         return 1;
    //       }
    //       break;
    //     case 't':
    //       meta_topic_prefix = optarg;
    //       break;
    //     case '?':
    //     default:
    //       usage(argv[0]);
    //       return 1;
    //       break;
    //   }
    // }

    // if (argc-optind != 0) {
    //   usage(argv[0]);
    //   return 1;
    // }

    // ros::init(argc, argv, "polymorph_relay_pub", (anonymous_node ? ros::init_options::AnonymousName : 0));
    // ros::NodeHandle nhp("~");
    // //nhp.param("verbose", verbose, 0);

    // ROSROSBridgePublisher pub(meta_topic_prefix, ad2pub_delay);
    // if (verbose > 0)
    //   pub.setVerbosity(verbose);

    // printf("Waiting for messages\n");
    // ros::spin();

    // printf("Exiting\n");

    // return 0;
  }
};

PLUGINLIB_EXPORT_CLASS(comms_bridge::BridgePublisherNodelet, nodelet::Nodelet)

}  // namespace comms_bridge
