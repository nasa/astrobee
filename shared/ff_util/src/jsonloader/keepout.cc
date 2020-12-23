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

#include <iterator>

jsonloader::Keepout::Keepout(Sequence const& boxes, bool const safe)
  : safe_zone_(safe) {
  std::copy(boxes.begin(), boxes.end(), std::back_inserter(bboxes_));
}

jsonloader::Keepout::Keepout(bool const safe)
  : safe_zone_(safe)
{ }

jsonloader::Keepout::Sequence const& jsonloader::Keepout::GetBoxes() const {
  return bboxes_;
}

bool jsonloader::Keepout::IsSafe() const noexcept {
  return safe_zone_;
}

void jsonloader::Keepout::Merge(jsonloader::Keepout const& zone) {
  if (IsSafe() != zone.IsSafe()) {
    return;
  }

  std::copy(zone.bboxes_.begin(), zone.bboxes_.end(),
    std::back_inserter(bboxes_));
}
