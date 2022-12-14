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

#ifndef FF_UTIL_CONFIG_SERVER_H_
#define FF_UTIL_CONFIG_SERVER_H_

// ROS includes
#include <ff_common/ff_ros.h>

#include <ff_util/ff_names.h>

#include <config_reader/config_reader.h>


#if ROS1
#include <diagnostic_msgs/KeyValue.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#else
#include <diagnostic_msgs/msg/key_value.hpp>
namespace diagnostic_msgs {
typedef msg::KeyValue KeyValue;
typedef msg::DiagnosticStatus DiagnosticStatus;
typedef msg::DiagnosticArray DiagnosticArray;
}  // namespace diagnostic_msgs

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#endif

#include <climits>
#include <cfloat>
#include <string>
#include <utility>
#include <vector>
#include <map>

namespace ff_util {

typedef boost::function<bool(dynamic_reconfigure::Config &request)> ConfigCallback;

class ConfigServer {
 public:
  // Constructor opens the ASTROBEE_CONFIG_DIR/subsystem/node.config file by default
  ConfigServer();
  virtual ~ConfigServer();
  // Initialize the system
  void Initialize(ros::NodeHandle *private_nh, const std::string &fname);
  // Manually set the path to the config file
  void SetPath(const std::string &pname);
  // Add another (optional) LUA config file to the configuration
  void AddFile(const std::string &fname);
  // Start listening for updates
  bool Listen();
  bool Listen(ConfigCallback f);
  // Get and Set after initialization
  template<typename T> bool Get(const std::string &name, T &value);         // Get error-aware
  template<typename T> bool Set(const std::string &name, const T &value);   // Set
  template<typename T> T Get(const std::string &name) {
    T tmp;
    if (!Get(name, tmp)) {
      ROS_ERROR_STREAM("Cannot query parameter " << name);
    }
    return tmp;
  }
  // Lim the values of a given type a runtime (helps prevent code from getting out of sync)
  template<typename T> bool Lim(const std::string &name, const std::map<T, std::string> &vals);
  // Dump all parameters into a KeyValue message
  std::vector<diagnostic_msgs::KeyValue> Dump();
  // Get the ConfigReader pointer for advanced usage
  config_reader::ConfigReader* GetConfigReader();
  // Default reconfigure callback
  bool Reconfigure(dynamic_reconfigure::Config &request);

 private:
  // Internal function to send dynamic_reconfigure updates
  dynamic_reconfigure::ConfigDescription UpdateDescription();
  dynamic_reconfigure::Config UpdateValues();
  // Assemble an edit method
  std::string EditMethod(const std::map<std::string, std::string> &values,
                         const std::string &ctype, const std::string &cpptype, const std::string &cconsttype);
  // Internal callback for configuration
  bool ReconfigureImpl(dynamic_reconfigure::Reconfigure::Request &req, dynamic_reconfigure::Reconfigure::Response &res);
  // Update function performs the paramater update
  static bool Update(const std::string &ns, const dynamic_reconfigure::ReconfigureRequest &request);
  bool initialized_;
  bool listening_;
  ros::NodeHandle nh_;
  ros::Publisher pub_d_;
  ros::Publisher pub_u_;
  ros::ServiceServer srv_r_;
  config_reader::ConfigReader config_params_;
  ConfigCallback callback_;
  std::map<std::string, bool> bools_;
  std::map<std::string, int> ints_;
  std::map<std::string, double> doubles_;
  std::map<std::string, std::string> strs_;
  dynamic_reconfigure::ConfigDescription description_;
};

template<> bool ConfigServer::Get<int>(const std::string &name, int &value);
template<> bool ConfigServer::Get<double>(const std::string &name, double &value);
template<> bool ConfigServer::Get<bool>(const std::string &name, bool &value);
template<> bool ConfigServer::Get<std::string>(const std::string &name, std::string &value);

template<> bool ConfigServer::Set<int>(const std::string &name, const int &value);
template<> bool ConfigServer::Set<double>(const std::string &name, const double &value);
template<> bool ConfigServer::Set<bool>(const std::string &name, const bool &value);
template<> bool ConfigServer::Set<std::string>(const std::string &name, const std::string &value);

template<> bool ConfigServer::Lim<int>(const std::string &name, const std::map<int, std::string> &vals);
template<> bool ConfigServer::Lim<double>(const std::string &name, const std::map<double, std::string> &vals);
template<> bool ConfigServer::Lim<bool>(const std::string &name, const std::map<bool, std::string> &vals);
template<> bool ConfigServer::Lim<std::string>(const std::string &name, const std::map<std::string, std::string> &vals);


}  // namespace ff_util

#endif  // FF_UTIL_CONFIG_SERVER_H_
