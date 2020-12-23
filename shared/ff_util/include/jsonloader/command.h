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

#ifndef JSONLOADER_COMMAND_H_
#define JSONLOADER_COMMAND_H_

#include <string>

namespace Json {
class Value;
}

namespace jsonloader {

class Command {
 public:
  virtual ~Command();

  virtual bool valid() const noexcept;
  bool blocking() const noexcept;
  std::string const& type() const noexcept;

  // Command Factory method to return the correct Command per the JSON object
  // - returns an empty unique_ptr on failure, which is 'false' in a
  //   boolean context.
  static Command * Make(Json::Value const& obj);

 protected:
  explicit Command(Json::Value const& obj);
  void set_valid(const bool valid) noexcept;

 private:
  bool valid_;
  bool blocking_;
  std::string type_;
};

// Fancy Command constructing function
template<typename T>
jsonloader::Command * CreateCommand(Json::Value const& obj) {
  return new T(obj);
}

using CommandCreateFn = jsonloader::Command*(*)(Json::Value const& obj);

}  // namespace jsonloader

#endif  // JSONLOADER_COMMAND_H_
