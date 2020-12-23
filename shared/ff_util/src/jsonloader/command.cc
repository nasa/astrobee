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

#include <jsonloader/command.h>
#include <jsonloader/validation.h>
#include <jsonloader/insensitive_map.h>

#include <glog/logging.h>

#include <json/json.h>

#include <string>

namespace jsonloader {

namespace internal {

// external maps defined in command_repo.cc
extern const InsensitiveMap<CommandCreateFn> kCommandMap;
extern const InsensitiveMap<std::string> kNormalizedNames;

}  // namespace internal

namespace {

// Basic fields every command has...
const Fields commandFields {
  new Field("type", Json::stringValue),
  new Field("blocking", Json::booleanValue),
};

}  // end namespace

Command * Command::Make(Json::Value const& obj) {
  using internal::kCommandMap;
  using internal::kNormalizedNames;

  if (!Validate(obj, commandFields)) {
    return nullptr;
  }

  auto it = kCommandMap.find(obj["type"].asString());
  if (it == kCommandMap.end()) {
    LOG(ERROR) << "invalid plan: unknown command: "
               << obj["type"].asString();
    return nullptr;
  }

  Command * cmd = it->second(obj);
  if (!cmd->valid()) {
    LOG(ERROR) << "invalid plan: recognized command '"
               << obj["type"].asString()
               << "' known but not valid.";
    return nullptr;
  }

  // Normalize the command name if need be.
  auto it2 = kNormalizedNames.find(it->first);
  if (it2 == kNormalizedNames.end()) {
    cmd->type_ = it->first;
  } else {
    LOG(INFO) << "normalizing " << cmd->type_ << " to " << it2->second;
    cmd->type_ = it2->second;
  }

  return cmd;
}

Command::Command(Json::Value const& obj)
  : valid_(false), blocking_(false) {
  if (!Validate(obj, commandFields))
    return;

  type_ = obj["type"].asString();
}

Command::~Command() {
}

bool Command::valid() const noexcept {
  return valid_;
}

bool Command::blocking() const noexcept {
  return blocking_;
}

std::string const& Command::type() const noexcept {
  return type_;
}

void Command::set_valid(const bool valid) noexcept {
  valid_ = valid;
}

}  // namespace jsonloader
