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

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <fstream>
#include <string>

namespace localization_common {
class Averager {
 public:
  explicit Averager(const std::string& averager_name = "", const std::string& type_name = "",
                    const std::string& units = "", const bool log_on_destruction = false);
  ~Averager();
  void Update(const double value);
  double average() const;
  int count() const;
  void Log() const;
  void LogToFile(std::ofstream& ofstream) const;
  void LogEveryN(const int num_events_per_log) const;
  void UpdateAndLog(const double value);
  void UpdateAndLogEveryN(const double value, const int num_events_per_log);
  void Vlog(const int level = 2) const;
  void VlogEveryN(const int num_events_per_log, const int level) const;

 private:
  std::string StatsString() const;
  std::string LastValueString() const;

  std::string name_;
  std::string type_name_;
  std::string units_;
  std::string spacer_;
  double last_value_;
  bool log_on_destruction_;
  boost::accumulators::accumulator_set<
    double, boost::accumulators::stats<boost::accumulators::tag::mean(boost::accumulators::immediate),
                                       boost::accumulators::tag::variance(boost::accumulators::immediate),
                                       boost::accumulators::tag::min, boost::accumulators::tag::max,
                                       boost::accumulators::tag::count>>
    accumulator_;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_AVERAGER_H_
