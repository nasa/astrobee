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

#include <jsonloader/planio.h>

#include <glog/logging.h>

#include <json/json.h>

#include <ctime>
#include <fstream>

jsonloader::Plan jsonloader::LoadPlan(std::string const& data) {
  Json::Value v, null_obj(Json::objectValue);
  if (!LoadData(data, &v)) {
    return jsonloader::Plan(null_obj);
  }

  return jsonloader::Plan(v);
}

bool jsonloader::LoadData(std::string const& data, Json::Value *json) {
  Json::Reader r;
  if (!r.parse(data, *json, false)) {
    LOG(ERROR) << "Error parsing json: " << r.getFormattedErrorMessages();
    return false;
  }

  return true;
}

// TODO(oalexan1): Must develop proper functionality for saving plans.
void jsonloader::WritePlanHeader(std::ofstream & ofs, double vel, double accel,
                                 double omega, double alpha, std::string creator) {
  // Prepare the timestamp, in the format: 2019-02-13T20:13:46Z
  std::time_t t;
  struct std::tm *curr_time;
  char time_str[256];
  std::time(&t);
  curr_time = std::localtime(&t);
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", curr_time);

  ofs << "{\n";
  ofs << "  \"type\" : \"ModuleBayPlan\",\n";
  ofs << "  \"inertiaConfiguration\" : {\n";
  ofs << "    \"name\" : \"UnloadedAstrobee\",\n";
  ofs << "    \"matrix\" : [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ],\n";
  ofs << "    \"mass\" : 5.0\n";
  ofs << "  },\n";
  ofs << "  \"operatingLimits\" : {\n";
  ofs << "    \"targetLinearVelocity\" : " << vel << ",\n";
  ofs << "    \"targetLinearAccel\" : " << accel << ",\n";
  ofs << "    \"targetAngularVelocity\" : " << omega << ",\n";
  ofs << "    \"targetAngularAccel\" : " << alpha << ",\n";
  ofs << "    \"profileName\" : \"Nominal\",\n";
  ofs << "    \"flightMode\" : \"nominal\",\n";
  ofs << "    \"collisionDistance\" : 0.0\n";
  ofs << "  },\n";
  ofs << "  \"platform\" : {\n";
  ofs << "    \"type\" : \"Platform\",\n";
  ofs << "    \"name\" : \"FreeFlyer\",\n";
  ofs << "    \"id\" : \"FreeFlyer\"\n";
  ofs << "  },\n";
  ofs << "  \"visible\" : true,\n";
  ofs << "  \"dateCreated\" : \"" << time_str << "\",\n";
  ofs << "  \"dateModified\" : \"" << time_str << "\",\n";
  ofs << "  \"xpjson\" : \"0.1\",\n";
  ofs << "  \"defaultSpeed\" : " << vel << ",\n";
  ofs << "  \"defaultTolerance\" : 1.0,\n";
  ofs << "  \"creator\" : \"" << creator << "\",\n";
  ofs << "  \"schemaUrl\" : \"http://www.example.com/freeFlyerPlanSchema.json\",\n";
  ofs << "  \"planNumber\" : 0,\n";
  ofs << "  \"libraryURLs\" : [ ],\n";
  ofs << "  \"valid\" : true,\n";
  ofs << "  \"site\" : {\n";
  ofs << "    \"type\" : \"Site\",\n";
  ofs << "    \"crs\" : {\n";
  ofs << "      \"properties\" : {\n";
  ofs << "        \"name\" : \"ISS Module\"\n";
  ofs << "      }\n";
  ofs << "    },\n";
  ofs << "    \"bbox\" : [ -0.7, -0.5, -0.65, 0.7, 0.5, 0.65 ],\n";
  ofs << "    \"name\" : \"ISS Analysis Frame\",\n";
  ofs << "    \"id\" : \"iss\"\n";
  ofs << "  },\n";
  ofs << "  \"sequence\" : [ {\n";
}

void jsonloader::WriteSegment(std::ofstream & ofs, std::vector<Eigen::VectorXd> const& SegVec,
                              double vel, double accel, double omega, double alpha, int id) {
  ofs << "  }, {\n";
  ofs << "    \"type\" : \"Segment\",\n";
  ofs << "    \"waypoints\" : [ ";
  for (size_t row = 0; row < SegVec.size(); row++) {
    std::string sep = ",";
    if (row + 1 == SegVec.size())
      sep = "";
    ofs << "[ ";
    for (int col = 0; col < SegVec[row].size(); col++) {
      ofs << SegVec[row][col];
      if (col + 1 != SegVec[row].size())
        ofs << ", ";
    }
    ofs <<  " ]" << sep << " ";
  }
  ofs << "],\n";
  ofs << "    \"waypointType\" : \"PoseVelAccel\",\n";
  ofs << "    \"tolerance\" : 0.0,\n";
  ofs << "    \"speed\" : " << vel << ",\n";
  ofs << "    \"useCustomSpeed\" : true,\n";
  ofs << "    \"maxAVel\" : " << omega << ",\n";
  ofs << "    \"maxAccel\" : " << accel << ",\n";
  ofs << "    \"maxAAccel\" : " << alpha << ",\n";
  ofs << "    \"stopAtEnd\" : true,\n";
  ofs << "    \"faceForward\" : false,\n";
  ofs << "    \"name\" : \"" << id << "-" << id + 1 << "\",\n";
  ofs << "    \"id\" : \"" << id << "\",\n";
  ofs << "    \"sequence\" : [ ]\n";
  ofs << "  }, {\n";
}

namespace {
  // A small local function

  // Add or subtract 360 degrees from an angle until it becomes between
  // -180 and 180.
  double find_remainder(double angle) {
    while (angle < -180.0) angle += 360.0;
    while (angle >  180.0) angle -= 360.0;
    return angle;
  }
}  // namespace

// The pose has x, y, z, roll, pitch yaw.
void jsonloader::WriteStation(std::ofstream & ofs, Eigen::VectorXd Pose, double tol, int id) {
  if (Pose.size() != 6)
    LOG(FATAL) << "Expecting a pose to have 6 elements.";

  // Adjust roll and yaw if they go out of range.
  Pose[3] = find_remainder(Pose[3]);
  Pose[5] = find_remainder(Pose[5]);

  // The station cannot handle a pitch > 90. Yet the planner can do
  // such motions.  If this assumption is violated, just write here 0,
  // since stations are not used when the bot is moving anyway.
  if (Pose[4] > 90.0 || Pose[4] < -90.0)
    Pose[4] = 0.0;

  ofs << "    \"type\" : \"ModuleBayStation\",\n";
  ofs << "    \"coordinate\" : {\n";
  ofs << "      \"type\" : \"ModuleBayPoint\",\n";
  ofs << "      \"x\" : "     << Pose[0] << ",\n";
  ofs << "      \"y\" : "     << Pose[1] << ",\n";
  ofs << "      \"z\" : "     << Pose[2] << ",\n";
  ofs << "      \"roll\" : "  << Pose[3] << ",\n";
  ofs << "      \"pitch\" : " << Pose[4] << ",\n";
  ofs << "      \"yaw\" : "   << Pose[5] << ",\n";
  ofs << "      \"ignoreOrientation\" : false,\n";
  ofs << "      \"moduleBayValid\" : false,\n";
  ofs << "      \"bookmark\" : null,\n";
  ofs << "      \"bookmarkValid\" : false\n";
  ofs << "    },\n";
  ofs << "    \"tolerance\" : " << tol << ",\n";
  ofs << "    \"stopOnArrival\" : false,\n";
  ofs << "    \"name\" : \"" << id << "\",\n";
  ofs << "    \"id\" : \"" << id << "\",\n";
  ofs << "    \"sequence\" : [ ]\n";
}

void jsonloader::WritePlanFooter(std::ofstream & ofs, std::string const& plan_name, int id) {
  ofs << "  } ],\n";
  ofs << "  \"name\" : \"" << plan_name << "\",\n";
  ofs << "  \"id\" : \"" << id << "\"\n";
  ofs << "}\n";
}
