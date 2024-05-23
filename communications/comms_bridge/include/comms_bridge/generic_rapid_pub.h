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

#ifndef COMMS_BRIDGE_GENERIC_RAPID_PUB_H_
#define COMMS_BRIDGE_GENERIC_RAPID_PUB_H_

#include <comms_bridge/util.h>
#include <ros/ros.h>

#include <string>

#include "knDds/DdsTypedSupplier.h"

#include "rapidUtil/RapidHelper.h"

#include "dds_msgs/AstrobeeConstantsSupport.h"
#include "dds_msgs/GenericCommsAdvertisementInfoSupport.h"
#include "dds_msgs/GenericCommsContentSupport.h"

namespace ff {

class GenericRapidPub {
 public:
  explicit GenericRapidPub(std::string const& robot_name);
  ~GenericRapidPub();

  template <typename T>
  void CopyString(const int max_size,
                  std::string src,
                  T &dest,
                  std::string const& data_name,
                  std::string const& topic);

  void SendAdvertisementInfo(std::string const& output_topic,
                             bool latching,
                             std::string const& data_type,
                             std::string const& md5_sum,
                             std::string definition);

  void SendContent(std::string const& output_topic,
                   std::string const& md5_sum,
                   uint8_t const* data,
                   const size_t data_size,
                   const int seq_num);

 private:
  using AdvertisementInfoSupplier =
    kn::DdsTypedSupplier<rapid::ext::astrobee::GenericCommsAdvertisementInfo>;
  using AdvertisementInfoSupplierPtr =
                                    std::unique_ptr<AdvertisementInfoSupplier>;
  AdvertisementInfoSupplierPtr advertisement_info_supplier_;

  using ContentSupplier =
              kn::DdsTypedSupplier<rapid::ext::astrobee::GenericCommsContent>;
  using ContentSupplierPtr = std::unique_ptr<ContentSupplier>;
  ContentSupplierPtr content_supplier_;

  unsigned int advertisement_info_seq_;
};

typedef std::shared_ptr<ff::GenericRapidPub> GenericRapidPubPtr;

}  // end namespace ff

#endif  // COMMS_BRIDGE_GENERIC_RAPID_PUB_H_
