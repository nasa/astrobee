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
