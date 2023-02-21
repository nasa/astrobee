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

#include <ff_hw_msgs/msg/configure_led_group.hpp>
namespace ff_hw_msgs {
  typedef msg::ConfigureLEDGroup ConfigureLEDGroup;
}  // namespace ff_hw_msgs

#include <ff_common/ff_names.h>
#include <ff_common/ff_ros.h>
#include <jsoncpp/json/allocator.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <iostream>
#include "../include/light_flow.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("light_flow_publisher");
  rclcpp::Publisher<ff_hw_msgs::ConfigureLEDGroup>::SharedPtr publishLEDGroup;
  publishLEDGroup = FF_CREATE_PUBLISHER(nh, ff_hw_msgs::ConfigureLEDGroup,
      TOPIC_HARDWARE_SIGNAL_LIGHTS_CONFIG, 1000);
  Json::Value input;
  std::cin >> input;
  light_flow::compileLightFlow(input);
  light_flow::publishLightFlow(input, publishLEDGroup, false);
  return 0;
}
