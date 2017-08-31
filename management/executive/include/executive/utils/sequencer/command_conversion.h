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


#ifndef EXECUTIVE_UTILS_SEQUENCER_COMMAND_CONVERSION_H_
#define EXECUTIVE_UTILS_SEQUENCER_COMMAND_CONVERSION_H_

#include <memory>
#include <string>
#include <unordered_map>

// Forward declarations
namespace jsonloader {

class Command;

}  // namespace jsonloader

namespace ff_msgs {

template<class Allocator> struct CommandStamped_;
typedef CommandStamped_<std::allocator<void> > CommandStamped;

}  // namespace ff_msgs

namespace sequencer {
namespace internal {

// a function that takes plan command and generates a DDS-based command
// if the conversion is successful, return true. otherwise... don't.
using GenerateFn = bool (*)(const jsonloader::Command * plan_cmd,
                            ff_msgs::CommandStamped * dds_cmd);

struct CommandInfo {
  std::string name;
  std::string subsystem;
  GenerateFn fn;
};

extern const std::unordered_map<std::string, CommandInfo> kCmdGenMap;

}  // namespace internal
}  // namespace sequencer

#endif  // EXECUTIVE_UTILS_SEQUENCER_COMMAND_CONVERSION_H_
