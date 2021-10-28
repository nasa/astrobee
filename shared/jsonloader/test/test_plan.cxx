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

// Test plan
// Tests that plan contruction elements are loaded correctly, detected and
// asserted.

#include <jsonloader/plan.h>
#include <jsonloader/planio.h>

#include <json/json.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

namespace fs = boost::filesystem;

const std::string kDataDir = std::string(std::getenv("DATA_DIR"));

std::string SlurpFile(std::string const& f_name) {
  std::ifstream file(f_name, std::ios::in | std::ios::binary);
  std::string data;
  data.append(std::istreambuf_iterator<char>(file),
              std::istreambuf_iterator<char>());
  file.close();

  return data;
}

TEST(PlanIO, LoadData) {
  static const std::string data = u8R"({ "hi":"mom" })";
  Json::Value v;
  ASSERT_TRUE(jsonloader::LoadData(data, &v));
  ASSERT_TRUE(v.isMember("hi"));
  EXPECT_EQ(v["hi"].asString(), "mom");
}

struct WaypointTimeCase {
  double time;
  uint32_t sec;
  uint32_t nsec;
};

const uint32_t kNsec = 100000000UL;

TEST(Time, FromDouble) {
  jsonloader::Time t;

  std::vector<WaypointTimeCase> cases = {
    { 0.0, 0, 0 },
    { 1.0, 1, 0 },
    { 2.5, 2, 5 * kNsec },
    { 5.25, 5, 25 * (kNsec / 10) },
    { 10.9673, 10, 9673 * (kNsec / 1000) }
  };

  for (auto c : cases) {
    t.Set(c.time);
    EXPECT_EQ(t.sec(), c.sec);
    EXPECT_EQ(t.nsec(), c.nsec);
  }
}

// TODO(tfmorse): Update waypointType to be proper type
TEST(Segment, NormalConstruction) {
  static const std::string data = u8R"(
    { "type": "Segment",
      "stopAtEnd": true,
      "speed": 0.25,
      "tolerance": 0.52,
      "faceForward": true,
      "waypointType": "ControlValues20",
      "waypoints": [ [ 0, 1, 2, 3, 4, 5, 6 ] ]
    }
  )";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  // Sanity checks
  jsonloader::Segment segment(v);
  ASSERT_TRUE(segment.valid());

  EXPECT_EQ(segment.waypoint_type(), "ControlValues20");
  EXPECT_TRUE(segment.stop_at_end());
  EXPECT_TRUE(segment.face_forward());
  EXPECT_FLOAT_EQ(segment.speed(), 0.25f);
  EXPECT_FLOAT_EQ(segment.tolerance(), 0.52f);

  // Make sure we are pulling the right number of waypoints
  jsonloader::Segment::WaypointSeq const& wpts = segment.waypoints();
  ASSERT_EQ(wpts.size(), 1);

  // Make sure we are creating the Eigen::Vectors right
  ASSERT_EQ(wpts[0].cwaypoint().size(), 6);
  for (int i = 0; i < 6; i++) {
    EXPECT_FLOAT_EQ(static_cast<float>(i+1), wpts[0].cwaypoint()[i]);
  }
}

TEST(Segment, InvalidWaypoint) {
  static const std::string data = u8R"(
    { "type": "Segment",
      "stopAtEnd": true,
      "speed": 0.25,
      "tolerance": 0.52,
      "faceForward": true,
      "waypointType": "ControlValues20",
      "waypoints": [ {} ]
    }
  )";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  jsonloader::Segment s(v);
  ASSERT_FALSE(s.valid());
}

TEST(Station, NormalConstruction) {
  static const std::string data = u8R"(
    { "type": "Station",
      "stopOnArrival": true,
      "tolerance": 0.52,
      "coordinate": {
        "type": "Point6Dof",
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "roll": 4.0,
        "pitch": 5.0,
        "yaw": 6.0
      },
      "sequence": [ ]
    }
  )";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  jsonloader::Station s(v);
  ASSERT_TRUE(s.valid());

  EXPECT_TRUE(s.stop_on_arrival());
  EXPECT_FLOAT_EQ(s.tolerance(), 0.52f);

  EXPECT_FLOAT_EQ(s.position()[0], 1.0f);
  EXPECT_FLOAT_EQ(s.position()[1], 2.0f);
  EXPECT_FLOAT_EQ(s.position()[2], 3.0f);
  EXPECT_FLOAT_EQ(s.orientation()[0], 4.0f);
  EXPECT_FLOAT_EQ(s.orientation()[1], 5.0f);
  EXPECT_FLOAT_EQ(s.orientation()[2], 6.0f);

  jsonloader::Station::CommandSeq const& cmds = s.commands();
  ASSERT_EQ(cmds.size(), 0);
}

TEST(InertiaConfiguration, NormalConstruction) {
  static const std::string data = u8R"(
    { "name" : "UnloadedAstrobee",
      "matrix" : [ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 ],
      "mass" : 5.0
    }
  )";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  jsonloader::InertiaConfiguration c(v);
  ASSERT_TRUE(c.valid());

  EXPECT_EQ(c.name(), "UnloadedAstrobee");
  EXPECT_FLOAT_EQ(c.mass(), 5.0f);
  EXPECT_EQ(c.matrix().size(), 9);
  for (int i = 0; i < 9; i++) {
    EXPECT_FLOAT_EQ(c.matrix()[i], static_cast<float>(i+1));
  }
}

TEST(InertiaConfiguration, BadMatrix) {
  static const std::string data = u8R"(
    { "name" : "UnloadedAstrobee",
      "matrix" : [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 ],
      "mass" : 5.0
    }
  )";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  jsonloader::InertiaConfiguration c(v);
  ASSERT_FALSE(c.valid());
}

TEST(OperatingLimits, NormalConstruction) {
  static const std::string data = u8R"(
    {
      "targetAngularAccel" : 0.1234,
      "targetLinearAccel" : 0.0123,
      "targetAngularVelocity" : 0.5678,
      "targetLinearVelocity" : 0.0567,
      "flightMode" : "nominal",
      "profileName" : "iss_nominal",
      "collisionDistance" : 0.25
    },
  )";
  Json::Value v;
  Json::Reader().parse(data, v, false);

  jsonloader::OperatingLimits l(v);
  ASSERT_TRUE(l.valid());

  EXPECT_EQ(l.flight_mode(), "nominal");
  EXPECT_EQ(l.profile_name(), "iss_nominal");
  EXPECT_FLOAT_EQ(l.collision_distance(), 0.25f);
  EXPECT_FLOAT_EQ(l.angular_accel(), 0.1234f);
  EXPECT_FLOAT_EQ(l.angular_velocity(), 0.5678f);
  EXPECT_FLOAT_EQ(l.linear_accel(), 0.0123f);
  EXPECT_FLOAT_EQ(l.linear_velocity(), 0.0567f);
}

TEST(Plan, SimplePlan) {
  Json::Value v;
  std::ifstream file(kDataDir + "simple_plan.fplan");
  file >> v;

  jsonloader::Plan p(v);
  ASSERT_TRUE(p.valid());

  EXPECT_FLOAT_EQ(p.default_tolerance(), 1.0f);
  EXPECT_FLOAT_EQ(p.default_speed(), 0.05f);

  std::vector<jsonloader::Station> const& stns = p.stations();
  std::vector<jsonloader::Segment> const& segs = p.segments();

  ASSERT_EQ(stns.size(), 2);
  ASSERT_EQ(segs.size(), 1);

  EXPECT_EQ(p.waypoint_type(), "ControlValues20");

  // TODO(tfmorse): Do we need to deep-inspect? Technically the other tests
  // have verified that the other loading parts work.
}

TEST(Plan, CompiledPlan) {
  Json::Value v;
  std::ifstream file(kDataDir + "compiled_plan.fplan");
  file >> v;

  jsonloader::Plan p(v);
  ASSERT_TRUE(p.valid());

  EXPECT_EQ(p.waypoint_type(), "ControlValues20");
}

TEST(Plan, InconsistentWaypoints) {
  Json::Value v;
  std::ifstream file(kDataDir + "inconsistent_waypoints.json");
  file >> v;

  jsonloader::Plan p(v);
  ASSERT_FALSE(p.valid());
}

TEST(PlanIO, LoadPlan) {
  std::string data = SlurpFile(kDataDir + "compiled_plan.fplan");
  jsonloader::Plan p(jsonloader::LoadPlan(data));
  ASSERT_TRUE(p.valid());
}

// Test the evolution of plans to make sure we can still load
// plans created with older versions of the workbench
TEST(PlanIO, PlanEvolution) {
  const std::string evolution_path = kDataDir + "plan_evolution/";

  // Get a list of the files
  fs::path evo_dir(evolution_path);
  ASSERT_TRUE(fs::exists(evo_dir));
  ASSERT_TRUE(fs::is_directory(evo_dir));

  for (fs::directory_entry & e :
       boost::make_iterator_range(fs::directory_iterator(evo_dir), {})) {
    const fs::path p = e.path();
    if (!fs::exists(p) || !fs::is_regular_file(p)) {
      continue;
    }

    if (p.extension().string() != ".fplan") {
      continue;
    }

    LOG(INFO) << "Loading " << p.string();
    std::string data = SlurpFile(p.string());
    jsonloader::Plan plan(jsonloader::LoadPlan(data));
    ASSERT_TRUE(plan.valid());

    jsonloader::OperatingLimits ol = plan.operating_limits();
    LOG(INFO) << "Operating Limits: " << ol.valid();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
