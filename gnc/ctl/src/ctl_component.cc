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

#include <ctl/ctl_ros.h>

#include <ff_common/init.h>
#include <ff_util/ff_component.h>

#include <memory>

namespace ctl {

class CtlComponent : public ff_util::FreeFlyerComponent {
 public:
  explicit CtlComponent(const rclcpp::NodeOptions & options) : ff_util::FreeFlyerComponent(options, NODE_CTL) {}
  ~CtlComponent() {}
  // This is called when the nodelet is loaded into the nodelet manager
  void Initialize(NodeHandle &nh) {
    // this used to be multi-threaded, but don't think it needs to be
    ctl_.reset(new ctl::Ctl(nh, GetName()));
  }

 private:
  std::shared_ptr<ctl::Ctl> ctl_;
};

}  // end namespace ctl

#include "rclcpp_components/register_node_macro.hpp"

// Declare the plugin
RCLCPP_COMPONENTS_REGISTER_NODE(ctl::CtlComponent)
