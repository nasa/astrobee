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

#ifndef JSONLOADER_PLAN_H_
#define JSONLOADER_PLAN_H_

#include <stdint.h>

#include <jsonloader/command.h>

#include <Eigen/Dense>

#include <array>
#include <memory>
#include <vector>
#include <string>

// Forward declaration, don't pull in Json if we don't have to
namespace Json {
class Value;
}  // end namespace Json

namespace jsonloader {

class Milestone {
 public:
  virtual bool IsSegment() const noexcept = 0;
  virtual bool IsStation() const noexcept = 0;
};

class Station : public Milestone {
 public:
  using CommandPtr = std::unique_ptr<Command>;
  using CommandSeq = std::vector<CommandPtr>;

  explicit Station(Json::Value const& obj);
  Station(Station && other);  // move constructor

  virtual bool IsSegment() const noexcept { return false; };
  virtual bool IsStation() const noexcept { return true; };

  bool valid() const noexcept;

  bool stop_on_arrival() const noexcept;
  float tolerance() const noexcept;

  Eigen::Vector3f const& position() const noexcept;
  Eigen::Vector3f const& orientation() const noexcept;

  CommandSeq const& commands() const noexcept;

 private:
  bool valid_;

  float tolerance_;
  bool stop_on_arrival_;

  // Possibly change these?
  Eigen::Vector3f position_;  // position as x,y,z
  Eigen::Vector3f orientation_;  // orientation as r,p,y

  CommandSeq commands_;
};

// Decouple from ros::Time
class Time {
 public:
  explicit Time(const uint32_t sec = 0, const uint32_t nsec = 0);
  explicit Time(const double time);

  void Add(Time const& other);

  void Set(const double time);
  void Set(const uint32_t sec, const uint32_t nsec);

  uint32_t sec() const noexcept;
  uint32_t nsec() const noexcept;

 private:
  void Normalize();

  uint32_t sec_;
  uint32_t nsec_;
};

// Storing time in a float is not going to work. Thus, we have to split it out
// into seconds and nanoseconds from the original double we receive.
//
// XXX(all): *IF* the waypoint format ever changes such that the first double
// value is not time, we are, as the french say: [explitive]ed
class Waypoint {
 public:
  explicit Waypoint(const std::size_t size = 0);
  Waypoint(const std::size_t size, const double time);
  Waypoint(const std::size_t size, Time const& time);

  Time& time() noexcept;
  Time const& ctime() const noexcept;

  Eigen::VectorXf& waypoint() noexcept;
  Eigen::VectorXf const& cwaypoint() const noexcept;

 private:
  Time time_;
  Eigen::VectorXf waypoint_;
};

class Segment : public Milestone {
 public:
  using WaypointSeq = std::vector<Waypoint>;

  explicit Segment(Json::Value const& obj);
  Segment();

  virtual bool IsSegment() const noexcept { return true; };
  virtual bool IsStation() const noexcept { return false; };

  bool valid() const noexcept;

  bool stop_at_end() const noexcept;
  bool face_forward() const noexcept;
  float speed() const noexcept;
  float tolerance() const noexcept;
  std::string const& waypoint_type() const noexcept;
  WaypointSeq const& waypoints() const noexcept;

 private:
  bool valid_;

  bool stop_at_end_;
  bool face_forward_;
  float speed_;
  float tolerance_;
  std::string waypoint_type_;
  WaypointSeq waypoints_;
};

class InertiaConfiguration {
 public:
  explicit InertiaConfiguration(Json::Value const& obj);
  InertiaConfiguration();

  bool valid() const noexcept;

  std::string const& name() const noexcept;
  float mass() const noexcept;
  std::array<float, 9> const& matrix() const noexcept;

 private:
  bool valid_;

  std::string name_;
  float mass_;
  std::array<float, 9> matrix_;
};

class OperatingLimits {
 public:
  explicit OperatingLimits(Json::Value const& obj);
  OperatingLimits();

  bool valid() const noexcept;

  std::string const& flight_mode() const noexcept;
  std::string const& profile_name() const noexcept;

  float collision_distance() const noexcept;
  float angular_accel() const noexcept;
  float angular_velocity() const noexcept;
  float linear_accel() const noexcept;
  float linear_velocity() const noexcept;

 private:
  bool valid_;

  std::string flight_mode_;
  std::string profile_;

  float collision_distance_;
  float angular_accel_;
  float angular_vel_;
  float linear_accel_;
  float linear_vel_;
};

class Plan {
 public:
  explicit Plan(Json::Value const& obj);
  Plan(Plan const& other) = delete;
  Plan(Plan &&other) = default;
  Plan();

  Plan & operator=(Plan &&other) = default;

  std::size_t NumMilestones() const noexcept;
  Milestone const& GetMilestone(std::size_t index) const;

  bool valid() const noexcept;

  std::string const& name() const noexcept;
  float default_speed() const noexcept;
  float default_tolerance() const noexcept;
  InertiaConfiguration const& inertia_configuration() const noexcept;
  OperatingLimits const& operating_limits() const noexcept;
  std::vector<Station> const& stations() const noexcept;
  std::vector<Segment> const& segments() const noexcept;

  std::string const& waypoint_type() const noexcept;

 private:
  bool valid_;

  std::string name_;

  float default_speed_;
  float default_tolerance_;

  std::string waypoint_type_;

  InertiaConfiguration inertia_config_;
  OperatingLimits operating_limits_;

  std::vector<Station> stations_;
  std::vector<Segment> segments_;
};

}  // end namespace jsonloader

#endif  // JSONLOADER_PLAN_H_
