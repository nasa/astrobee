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

#include <jsonloader/plan.h>

#include <jsonloader/validation.h>
#include <jsonloader/command.h>

#include <glog/logging.h>

#include <json/json.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

using jsonloader::Fields;
using jsonloader::Field;
using jsonloader::StringField;
using jsonloader::BoolField;
using jsonloader::RangeFieldF;
using jsonloader::ObjectField;
using jsonloader::EnumField;
using jsonloader::Validate;

namespace {

const Fields inertiaConfFields {
  new Field("name", Json::stringValue),
  new Field("matrix", Json::arrayValue),
  new Field("mass", Json::realValue),
};

// TODO(tfmorse): narrow down flightMode
const Fields opLimitFields {
  new Field("flightMode", Json::stringValue),
  new Field("profileName", Json::stringValue),
  new Field("collisionDistance", Json::realValue),
  new Field("targetAngularAccel", Json::realValue),
  new Field("targetLinearAccel", Json::realValue),
  new Field("targetAngularVelocity", Json::realValue),
  new Field("targetLinearVelocity", Json::realValue),
};

const Fields planFields {
  new Field("name", Json::stringValue),
  new EnumField("type", { "FreeFlyerPlan", "ModuleBayPlan" }),
  new BoolField("valid", true),
  new ObjectField("inertiaConfiguration", inertiaConfFields, false),
  new ObjectField("operatingLimits", opLimitFields, false),
  new ObjectField("site", {
    new StringField("type", "Site"),
    new StringField("id", "iss")
  }),
  new Field("valid", Json::booleanValue),
  new RangeFieldF("defaultTolerance", 0.0f, 2.0f),
  new RangeFieldF("defaultSpeed", 0.0f, 2.0f),
  new Field("sequence", Json::arrayValue),
};

const Fields stationFields = {
  new EnumField("type", { "Station", "ModuleBayStation" }),
  new ObjectField("coordinate", {
    new EnumField("type", { "Point6Dof", "ModuleBayPoint" }),
    new Field("x", Json::realValue),
    new Field("y", Json::realValue),
    new Field("z", Json::realValue),
    new Field("roll", Json::realValue),
    new Field("pitch", Json::realValue),
    new Field("yaw", Json::realValue)
  }),
  new Field("stopOnArrival", Json::booleanValue),
  new Field("tolerance", Json::realValue),
  new Field("sequence", Json::arrayValue)
};

// TODO(tfmorse): Are speed/tolerance required?
const Fields segmentFields = {
  new StringField("type", "Segment"),
  new RangeFieldF("speed", 0.0f, 1.0f),
  new RangeFieldF("tolerance", 0.0f, 1.0f),
  new Field("faceForward", Json::booleanValue),
  new EnumField("waypointType", {
    "ControlValues20",
    "PoseVelAccel",
    "NotUsed"
  }),
  new Field("waypoints", Json::arrayValue)
};

}  // end namespace

jsonloader::Plan::Plan()
  : valid_(false), name_(""), default_speed_(0.0), default_tolerance_(0.0),
    waypoint_type_("") {
}

jsonloader::Plan::Plan(Json::Value const& obj)
  : valid_(false), name_(""), default_speed_(0.0), default_tolerance_(0.0),
    waypoint_type_("") {
  if (!Validate(obj, planFields)) {
    LOG(ERROR) << "invalid plan.";
    return;
  }

  name_ = obj["name"].asString();
  default_speed_ = obj["defaultSpeed"].asFloat();
  default_tolerance_ = obj["defaultTolerance"].asFloat();

  if (obj.isMember("inertiaConfiguration")) {
    inertia_config_ = InertiaConfiguration(obj["inertiaConfiguration"]);
    if (!inertia_config_.valid()) {
      LOG(ERROR) << "invalid plan: invalid inertia config";
      return;
    }
  }

  if (obj.isMember("operatingLimits")) {
    operating_limits_ = OperatingLimits(obj["operatingLimits"]);
    if (!operating_limits_.valid()) {
      LOG(ERROR) << "invalid plan: invalid inertia config";
      return;
    }
  }

  int i = 0;
  for (Json::Value const& v : obj["sequence"]) {
    if (i % 2 == 0) {  // Station
      Station s(v);
      if (!s.valid()) {
        LOG(ERROR) << "invalid plan: invalid station " << i;
        return;
      }

      stations_.push_back(std::move(s));
    } else {  // Segment
      Segment s(v);
      if (!s.valid()) {
        LOG(ERROR) << "invalid plan: invalid segment " << i;
        return;
      }

      if (waypoint_type_ == "") {
        waypoint_type_ = s.waypoint_type();
      } else if (waypoint_type_ != s.waypoint_type()) {
        LOG(ERROR) << "invalid plan: waypoint types inconsistent: "
                   << "expected: " << waypoint_type_ << ", "
                   << "actual: " << s.waypoint_type()
                   << " (segment " << i << ")";
        return;
      }

      segments_.push_back(s);
    }

    i++;
  }

  valid_ = true;
}

bool jsonloader::Plan::valid() const noexcept {
  return valid_;
}

std::string const& jsonloader::Plan::name() const noexcept {
  return name_;
}

float jsonloader::Plan::default_speed() const noexcept {
  return default_speed_;
}

float jsonloader::Plan::default_tolerance() const noexcept {
  return default_tolerance_;
}

jsonloader::InertiaConfiguration const&
jsonloader::Plan::inertia_configuration() const noexcept {
  return inertia_config_;
}

jsonloader::OperatingLimits const&
jsonloader::Plan::operating_limits() const noexcept {
  return operating_limits_;
}

std::vector<jsonloader::Station> const&
jsonloader::Plan::stations() const noexcept {
  return stations_;
}

std::vector<jsonloader::Segment> const&
jsonloader::Plan::segments() const noexcept {
  return segments_;
}

std::size_t jsonloader::Plan::NumMilestones() const noexcept {
  return (segments_.size() + stations_.size());
}

jsonloader::Milestone const&
jsonloader::Plan::GetMilestone(std::size_t index) const {
  if (index >= NumMilestones()) {
    LOG(WARNING) << "index out of range";
    throw std::out_of_range("milestone index out of range.");
  }

  if (index % 2 == 0) {
    return stations_[index / 2];
  } else {
    return segments_[(index - 1) / 2];
  }
}

std::string const& jsonloader::Plan::waypoint_type() const noexcept {
  return waypoint_type_;
}

jsonloader::InertiaConfiguration::InertiaConfiguration()
  : valid_(false), name_(""), mass_(0.0f) {
  matrix_.fill(0.0f);
}

jsonloader::InertiaConfiguration::InertiaConfiguration(Json::Value const& obj)
  : valid_(false), name_(""), mass_(0.0f) {
  matrix_.fill(0.0f);

  if (!Validate(obj, inertiaConfFields)) {
    LOG(ERROR) << "invalid inertia configuration.";
    return;
  }

  name_ = obj["name"].asString();
  mass_ = obj["mass"].asFloat();

  if (obj["matrix"].size() != 9) {
    LOG(ERROR) << "invalid inertial matrix";
    return;
  }

  for (int i = 0; i < 9; i++) {
    matrix_[i] = obj["matrix"][i].asFloat();
  }

  valid_ = true;
}

bool jsonloader::InertiaConfiguration::valid() const noexcept {
  return valid_;
}

std::string const& jsonloader::InertiaConfiguration::name() const noexcept {
  return name_;
}

float jsonloader::InertiaConfiguration::mass() const noexcept {
  return mass_;
}

std::array<float, 9> const&
jsonloader::InertiaConfiguration::matrix() const noexcept {
  return matrix_;
}

jsonloader::OperatingLimits::OperatingLimits()
  : valid_(false), flight_mode_(""), profile_(""),
    collision_distance_(0.0f), angular_accel_(0.0f), angular_vel_(0.0f),
    linear_accel_(0.0f), linear_vel_(0.0f) {
}

jsonloader::OperatingLimits::OperatingLimits(Json::Value const& obj)
  : valid_(false), flight_mode_(""), profile_(""),
    collision_distance_(0.0f), angular_accel_(0.0f), angular_vel_(0.0f),
    linear_accel_(0.0f), linear_vel_(0.0f) {
  if (!Validate(obj, opLimitFields)) {
    LOG(ERROR) << "invalid operating limits";
    return;
  }

  flight_mode_ = obj["flightMode"].asString();
  profile_ = obj["profileName"].asString();

  collision_distance_ = obj["collisionDistance"].asFloat();

  angular_accel_ = obj["targetAngularAccel"].asFloat();
  angular_vel_ = obj["targetAngularVelocity"].asFloat();
  linear_accel_ = obj["targetLinearAccel"].asFloat();
  linear_vel_ = obj["targetLinearVelocity"].asFloat();

  valid_ = true;
}

bool jsonloader::OperatingLimits::valid() const noexcept {
  return valid_;
}

std::string const& jsonloader::OperatingLimits::flight_mode() const noexcept {
  return flight_mode_;
}

std::string const& jsonloader::OperatingLimits::profile_name() const noexcept {
  return profile_;
}

float jsonloader::OperatingLimits::collision_distance() const noexcept {
  return collision_distance_;
}

float jsonloader::OperatingLimits::angular_accel() const noexcept {
  return angular_accel_;
}

float jsonloader::OperatingLimits::angular_velocity() const noexcept {
  return angular_vel_;
}

float jsonloader::OperatingLimits::linear_accel() const noexcept {
  return linear_accel_;
}

float jsonloader::OperatingLimits::linear_velocity() const noexcept {
  return linear_vel_;
}

jsonloader::Time::Time(const uint32_t sec, const uint32_t nsec)
  : sec_(sec), nsec_(nsec) {
  Normalize();
}

jsonloader::Time::Time(const double time) {
  Set(time);
}

// TODO(tfmorse): Deal with overflows.. eventually
void jsonloader::Time::Add(Time const& other) {
  sec_ += other.sec_;
  nsec_ += other.nsec_;
  Normalize();
}

void jsonloader::Time::Set(const uint32_t sec, const uint32_t nsec) {
  sec_ = sec;
  nsec_ = nsec;
  Normalize();
}

void jsonloader::Time::Set(const double time) {
  sec_ = static_cast<uint32_t>(std::floor(time));
  double nsecd = (time - sec_) * 1e9;
  if (nsecd < 0) {
    nsec_ = static_cast<uint32_t>(std::ceil(nsecd - 0.5));
  } else {
    nsec_ = static_cast<uint32_t>(std::floor(nsecd + 0.5));
  }
}

uint32_t jsonloader::Time::sec() const noexcept {
  return sec_;
}

uint32_t jsonloader::Time::nsec() const noexcept {
  return nsec_;
}

void jsonloader::Time::Normalize() {
  uint64_t nsecs = static_cast<uint64_t>(nsec_) % 1000000000ULL;
  uint64_t secs = static_cast<uint64_t>(nsec_) / 1000000000ULL;

  // XXX(all): could this overflow?
  sec_ += secs;
  nsec_ = nsecs;
}

jsonloader::Waypoint::Waypoint(const std::size_t size)
  : waypoint_(size) {
}

jsonloader::Waypoint::Waypoint(const std::size_t size, const double time)
  : time_(time), waypoint_(size) {
}

jsonloader::Waypoint::Waypoint(const std::size_t size,
                               jsonloader::Time const& time)
  : time_(time), waypoint_(size) {
}

jsonloader::Time& jsonloader::Waypoint::time() noexcept {
  return time_;
}

jsonloader::Time const& jsonloader::Waypoint::ctime() const noexcept {
  return time_;
}

Eigen::VectorXf& jsonloader::Waypoint::waypoint() noexcept {
  return waypoint_;
}

Eigen::VectorXf const& jsonloader::Waypoint::cwaypoint() const noexcept {
  return waypoint_;
}

jsonloader::Segment::Segment(Json::Value const& obj)
  : valid_(false), waypoints_(0) {
  if (!Validate(obj, segmentFields)) {
    LOG(ERROR) << "invalid segment.";
    return;
  }

  stop_at_end_ = obj["stopAtEnd"].asBool();
  face_forward_ = obj["faceForward"].asBool();
  speed_ = obj["speed"].asFloat();
  tolerance_ = obj["tolerance"].asFloat();
  waypoint_type_ = obj["waypointType"].asString();

  // Load the waypoints
  waypoints_.reserve(obj["waypoints"].size());
  for (Json::Value const& w : obj["waypoints"]) {
    if (!w.isArray()) {
      LOG(ERROR) << "invalid segment: waypoint is not an array.";
      return;
    }

    // Copy this array to a waypoint
    Waypoint wpt(w.size() - 1, w[0].asDouble());
    Eigen::VectorXf &ew = wpt.waypoint();
    for (size_t i = 1; i < w.size(); i++) {
      ew[i-1] = w[static_cast<int>(i)].asFloat();
    }

    waypoints_.push_back(wpt);
  }

  valid_ = true;
}

jsonloader::Segment::Segment()
  : valid_(false), waypoints_(0) {
}

bool jsonloader::Segment::valid() const noexcept {
  return valid_;
}

bool jsonloader::Segment::face_forward() const noexcept {
  return face_forward_;
}

std::string const& jsonloader::Segment::waypoint_type() const noexcept {
  return waypoint_type_;
}

bool jsonloader::Segment::stop_at_end() const noexcept {
  return stop_at_end_;
}

float jsonloader::Segment::speed() const noexcept {
  return speed_;
}

float jsonloader::Segment::tolerance() const noexcept {
  return tolerance_;
}

jsonloader::Segment::WaypointSeq const&
jsonloader::Segment::waypoints() const noexcept {
  return waypoints_;
}

jsonloader::Station::Station(Json::Value const& obj)
  : valid_(false) {
  if (!Validate(obj, stationFields)) {
    LOG(ERROR) << "invalid station.";
    return;
  }

  tolerance_ = obj["tolerance"].asFloat();
  stop_on_arrival_ = obj["stopOnArrival"].asBool();

  Json::Value const& dof = obj["coordinate"];
  position_ << dof["x"].asFloat(),
               dof["y"].asFloat(),
               dof["z"].asFloat();
  orientation_ << dof["roll"].asFloat(),
                  dof["pitch"].asFloat(),
                  dof["yaw"].asFloat();

  Json::Value const& cmds = obj["sequence"];

  commands_.reserve(cmds.size());
  for (Json::Value const& c : cmds) {
    Command * cmd = Command::Make(c);
    if (cmd == nullptr) {
      LOG(ERROR) << "invalid plan: command invalid: "
                 << Json::FastWriter().write(c);
      return;
    }

    commands_.push_back(std::unique_ptr<Command>(cmd));
  }

  valid_ = true;
}

jsonloader::Station::Station(jsonloader::Station && o)
  : valid_(o.valid_), tolerance_(o.tolerance_),
    stop_on_arrival_(o.stop_on_arrival_),
    position_(std::move(o.position_)),
    orientation_(std::move(o.orientation_)),
    commands_(std::move(o.commands_)) { }

bool jsonloader::Station::valid() const noexcept {
  return valid_;
}

bool jsonloader::Station::stop_on_arrival() const noexcept {
  return stop_on_arrival_;
}

float jsonloader::Station::tolerance() const noexcept {
  return tolerance_;
}

Eigen::Vector3f const& jsonloader::Station::position() const noexcept {
  return position_;
}

Eigen::Vector3f const& jsonloader::Station::orientation() const noexcept {
  return orientation_;
}

jsonloader::Station::CommandSeq const&
jsonloader::Station::commands() const noexcept {
  return commands_;
}

