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


#ifndef EXECUTIVE_EXECUTIVE_ACTION_CLIENT_H_
#define EXECUTIVE_EXECUTIVE_ACTION_CLIENT_H_

#include <actionlib/client/simple_action_client.h>
#include <ff_util/ff_action.h>
#include <string>

namespace executive {

enum Action {
  NONE,
  ARM,
  DOCK,
  EXECUTE,
  IDLE,
  MOVE,
  PERCH,
  STOP,
  SWITCH,
  UNDOCK,
  UNPERCH
};

template <class T>
class ExecutiveActionClient : public ff_util::FreeFlyerActionClient<T> {
 public:
  Action action();
  void action(Action const& action);
  std::string cmd_id();
  void cmd_id(std::string const& cmd_id);
  std::string cmd_origin();
  void cmd_origin(std::string const& cmd_origin);
  void SetCmdInfo(Action const& action,
                  std::string const& cmd_id,
                  std::string const& cmd_origin);
 private:
  std::string cmd_id_;
  std::string cmd_origin_;
  Action action_;
};

template <class T>
Action ExecutiveActionClient<T>::action() {
  return action_;
}

template <class T>
void ExecutiveActionClient<T>::action(Action const& action) {
  action_ = action;
}

template <class T>
std::string ExecutiveActionClient<T>::cmd_id() {
  return cmd_id_;
}

template <class T>
void ExecutiveActionClient<T>::cmd_id(std::string const& cmd_id) {
  cmd_id_ = cmd_id;
}

template <class T>
std::string ExecutiveActionClient<T>::cmd_origin() {
  return cmd_origin_;
}

template <class T>
void ExecutiveActionClient<T>::cmd_origin(std::string const& cmd_origin) {
  cmd_origin_ = cmd_origin;
}

template <class T>
void ExecutiveActionClient<T>::SetCmdInfo(Action const& action,
                                          std::string const& cmd_id,
                                          std::string const& cmd_origin) {
  action_ = action;
  cmd_id_ = cmd_id;
  cmd_origin_ = cmd_origin;
}

}  // namespace executive

#endif  // EXECUTIVE_EXECUTIVE_ACTION_CLIENT_H_
