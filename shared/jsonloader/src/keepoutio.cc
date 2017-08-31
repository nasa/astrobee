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

#include <jsonloader/keepout.h>
#include <jsonloader/keepoutio.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range_core.hpp>

#include <glog/logging.h>

#include <json/json.h>

#include <memory>
#include <exception>
#include <fstream>

using jsonloader::Keepout;

namespace fs = boost::filesystem;

bool jsonloader::ReadKeepoutFile(std::string const& input_filename,
    Keepout *zone) {
  std::ifstream file(input_filename.c_str(), std::iostream::in);

  Json::Value value;
  file >> value;

  bool safe;

  if (value.isMember("safe") && value["safe"].isBool()) {
    safe = value["safe"].asBool();
  } else {
    LOG(ERROR) << "invalid keepout file: 'safe': missing or wrong type.";
    return false;
  }

  if (!value.isMember("sequence") || !value["sequence"].isArray()) {
    LOG(ERROR) << "invalid keepout file: 'sequence': missing or wrong type.";
    return false;
  }

  Json::Value const& sequence = value["sequence"];
  Keepout::Sequence bounds;
  bounds.reserve(sequence.size());

  for (Json::Value const& box : sequence) {
    if (!box.isArray() || box.size() != 6) {
      LOG(ERROR) << "invalid keepout file: invalid bounding box.";
      return false;
    }

    // First 3 elements are the x, y, z of one corner...
    Eigen::Vector3f min(box[0].asFloat(), box[1].asFloat(), box[2].asFloat());
    Eigen::Vector3f max(box[3].asFloat(), box[4].asFloat(), box[5].asFloat());

    Keepout::BoundingBox bbox(std::move(min), std::move(max));
    bounds.push_back(std::move(bbox));
  }

  *zone = Keepout(bounds, safe);
  return true;
}

void jsonloader::ReadKeepoutDirectory(std::string const& input_directory,
                                      Keepout *safeZone,
                                      Keepout *dangerZone) {
  // Why even bother?
  if (safeZone == NULL && dangerZone == NULL) {
    LOG(WARNING) << "no safe or danger zones passed in";
    return;
  }

  fs::path dir(input_directory);
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    LOG(WARNING) << "no such directory or not a directory.";
    return;
  }

  auto files = boost::make_iterator_range(fs::directory_iterator(dir),
    fs::directory_iterator());
  for (fs::path const& e : files) {
    if (e.extension() != ".json") {
      continue;
    }

    try {
      Keepout temp(false);  // doesn't matter
      if (!ReadKeepoutFile(e.native(), &temp)) {
        LOG(WARNING) << "invalid keepout file in directory "
                     << e.native() << ": moving onto next file.";
        continue;
      }

      if (temp.IsSafe() && safeZone != NULL) {
        safeZone->Merge(temp);
      } else if (!temp.IsSafe() && dangerZone != NULL) {
        dangerZone->Merge(temp);
      }
    } catch (std::exception& ex) {
      LOG(WARNING) << "problem reading keepout file: "
                   << ex.what();
      continue;
    }
  }
}

