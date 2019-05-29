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

#ifndef JSONLOADER_PLANIO_H_
#define JSONLOADER_PLANIO_H_

#include <jsonloader/plan.h>

#include <glog/logging.h>

#include <string>
#include <vector>

namespace Json {
  class Value;
}  // end namespace Json

namespace jsonloader {
  Plan LoadPlan(std::string const& data);
  bool LoadData(std::string const& data, Json::Value *json);

  // Utilities for writing a plan
  void WritePlanHeader(std::ofstream & ofs, double vel, double accel, double omega, double alpha,
                       std::string creator);
  void WriteStation(std::ofstream & ofs, Eigen::VectorXd Pose, double tol, int id);
  void WriteSegment(std::ofstream & ofs, std::vector<Eigen::VectorXd> const& SegVec,
                    double vel, double accel, double omega, double alpha, int id);
  void WritePlanFooter(std::ofstream & ofs, std::string const& plan_name, int id);
  void WritePlanFooter(std::ofstream & ofs, std::string const& plan_name, int id);

}  // end namespace jsonloader

#endif  // JSONLOADER_PLANIO_H_
