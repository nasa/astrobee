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

#ifndef LOCALIZATION_COMMON_AVERAGER_H_
#define LOCALIZATION_COMMON_AVERAGER_H_

#include <string>

namespace localization_common {
class Averager {
 public:
  explicit Averager(const std::string& averager_name = "");
  void Update(const double value);
  double average() const;
  void Log();
  void UpdateAndLog(const double value);
  void Vlog(const int level = 2);

 private:
  std::string name_;
  double last_value_;
  double average_;
  int num_events_;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_AVERAGER_H_
