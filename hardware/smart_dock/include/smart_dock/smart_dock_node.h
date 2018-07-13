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


#ifndef SMART_DOCK_SMART_DOCK_NODE_H_
#define SMART_DOCK_SMART_DOCK_NODE_H_

#include <string.h>
#include <sys/time.h>

#include <iostream>
#include <memory>
#include <string>
#include <map>

#include "AstrobeeConstants.h"

#include "common/init.h"

#include "DockStateSupport.h"

#include "knDds/DdsEntitiesFactory.h"
#include "knDds/DdsEntitiesFactorySvc.h"
#include "knDds/DdsEventLoop.h"
#include "knDds/DdsSupport.h"
#include "knDds/DdsTypedSupplier.h"

#include "miro/Configuration.h"
#include "miro/Robot.h"
#include "miro/Log.h"

#include "rapidDds/RapidConstants.h"
#include "rapidDds/AckSupport.h"
#include "rapidDds/CommandSupport.h"

#include "rapidUtil/RapidHelper.h"

#include "smart_dock/smart_dock.h"

namespace kn {
  class DdsEntitiesFactorySvc;
}  // end namespace kn

namespace smart_dock {
class SmartDockNode {
 public:
  // Contructor
  SmartDockNode(int argc, char** argv, const i2c::Device &i2c_dev,
    std::string const& name);

  // Destructor
  ~SmartDockNode();

  // Callback for the DDS event loop
  void operator() (rapid::Command const* cmd);

  // Communcation publish loop
  void ProcessComms();

  // Telemetry publish loop (latched)
  void ProcessTelem();

  // Register a device (robot) serial to ID mapping
  bool AddDevice(std::string const& serial, std::string const& name);

 protected:
  // Publish an ack
  void PublishCmdAck(std::string const& cmd_id,
                     rapid::AckStatus status = rapid::ACK_COMPLETED,
                     rapid::AckCompletedStatus completed_status =
                                                        rapid::ACK_COMPLETED_OK,
                     std::string const& message = "");

  // Make a DDS timestamp
  int64_t MakeTimestamp();

 private:
  using AckSupplier = kn::DdsTypedSupplier<rapid::Ack>;
  using DockSupplier = kn::DdsTypedSupplier<rapid::ext::astrobee::DockState>;
  // Interface to the smart dock
  SmartDock sd_;
  // Serial -> Name lookup table
  std::map<std::string, std::string> devices_;
  // Various suffixes for messages
  std::string ack_pub_suffix_, dock_state_pub_suffix_, sub_suffix_;
  //////////////////////////////////////////////////////////////////////////////
  // NB: The order of the variables below is ESSENTIAL. They are destroyed from
  // the bottom to top. It is essential that the destruction order remains sane
  // or else we will get weird hanging and segfaults on exit
  //////////////////////////////////////////////////////////////////////////////
  // Entity factory
  std::shared_ptr<kn::DdsEntitiesFactorySvc> dds_entities_factory_;
  // Dock state publisher
  DockSupplier *dock_state_supplier_;
  // Ack state publisher
  AckSupplier *ack_state_supplier_;
  // Messaging event loop
  kn::DdsEventLoop dds_event_loop_;
};

}  // end namespace smart_dock

#endif  // SMART_DOCK_SMART_DOCK_NODE_H_
