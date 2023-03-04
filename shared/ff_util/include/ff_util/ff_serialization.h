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

#ifndef FF_UTIL_FF_SERIALIZATION_H_
#define FF_UTIL_FF_SERIALIZATION_H_

// ROS includes
#include <rclcpp/serialization.hpp>

// STL includes
#include <fstream>
#include <string>

namespace ff_util {

class Serialization {
 public:
  // Write a ROS message to a file
  template < class RosMessage >
  static bool WriteFile(std::string const& file_name, RosMessage const& msg) {
    std::ofstream ofs(file_name, std::ios::out | std::ios::binary);
    if (!ofs.is_open())
      return false;
    auto serializer = rclcpp::Serialization<RosMessage>();
    rclcpp::SerializedMessage s;
    serializer.serialize_message(reinterpret_cast<const void*>(&msg), &s);
    uint32_t serial_size = s.size();
    ofs.write(reinterpret_cast < char* > (s.get_rcl_serialized_message().buffer), serial_size);
    ofs.close();
    return true;
  }
  // Read a ROS message from a file
  template < class RosMessage >
  static bool ReadFile(std::string const& file_name, RosMessage & msg) {
    std::ifstream ifs(file_name, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
      return false;
    std::string contents;
    contents.assign(std::istreambuf_iterator<char>(ifs),
                std::istreambuf_iterator<char>());
    ifs.close();
    rcl_serialized_message_t m = rmw_get_zero_initialized_serialized_message();
    m.buffer = reinterpret_cast<unsigned char*>(const_cast<char*>(contents.c_str()));
    m.buffer_length = contents.size();
    m.buffer_capacity = contents.size();
    rclcpp::SerializedMessage s(m);
    auto serializer = rclcpp::Serialization<RosMessage>();
    serializer.deserialize_message(&s, reinterpret_cast<void*>(&msg));
    return true;
  }
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_SERIALIZATION_H_
