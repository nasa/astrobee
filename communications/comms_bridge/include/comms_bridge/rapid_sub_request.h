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

#ifndef COMMS_BRIDGE_RAPID_SUB_REQUEST_H_
#define COMMS_BRIDGE_RAPID_SUB_REQUEST_H_

#include <comms_bridge/generic_ros_sub_rapid_pub.h>
#include <comms_bridge/generic_rapid_sub.h>

#include <string>

#include "knDds/DdsTypedSupplier.h"

#include "dds_msgs/AstrobeeConstants.h"
#include "dds_msgs/GenericCommsRequestSupport.h"

namespace ff {

class RapidSubRequest : public GenericRapidSub {
 public:
  RapidSubRequest(const std::string& entity_name,
                  const std::string& subscribe_topic,
                  const std::string& subscriber_partition,
                  GenericROSSubRapidPub* ros_sub_rapid_pub);

  /**
   * Call back for ddsEventLoop
   */
  void operator() (rapid::ext::astrobee::GenericCommsRequest const* request);

 private:
  GenericROSSubRapidPub* ros_sub_;
};

}  // end namespace ff

#endif  // COMMS_BRIDGE_RAPID_SUB_REQUEST_H_
