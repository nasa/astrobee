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

#ifndef COMMS_BRIDGE_RAPID_PUB_REQUEST_H_
#define COMMS_BRIDGE_RAPID_PUB_REQUEST_H_

#include <comms_bridge/util.h>
#include <ros/ros.h>

#include <string>

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "dds_msgs/AstrobeeConstantsSupport.h"
#include "dds_msgs/GenericCommsRequestSupport.h"

namespace ff {

class RapidPubRequest {
 public:
  explicit RapidPubRequest(std::string const& robot_name);
  ~RapidPubRequest();

  void SendRequest(std::string const& output_topic);

 private:
  using RequestSupplier =
              kn::DdsTypedSupplier<rapid::ext::astrobee::GenericCommsRequest>;
  using RequestSupplierPtr = std::unique_ptr<RequestSupplier>;
  RequestSupplierPtr request_supplier_;

  unsigned int request_seq_;
};

typedef std::shared_ptr<ff::RapidPubRequest> RapidPubRequestPtr;

}  // end namespace ff

#endif  // COMMS_BRIDGE_RAPID_PUB_REQUEST_H_
