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
#include <ros/ros.h>

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
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array < uint8_t > obuffer(new uint8_t[serial_size]);
    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, msg);
    ofs.write(reinterpret_cast < char* > (obuffer.get()), serial_size);
    ofs.close();
    return true;
  }
  // Read a ROS message from a file
  template < class RosMessage >
  static bool ReadFile(std::string const& file_name, RosMessage & msg) {
    std::ifstream ifs(file_name, std::ios::in | std::ios::binary);
    if (!ifs.is_open())
      return false;
    ifs.seekg(0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::streampos begin = ifs.tellg();
    uint32_t file_size = end - begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read(reinterpret_cast<char*> (ibuffer.get()), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, msg);
    ifs.close();
    return true;
  }
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_SERIALIZATION_H_
