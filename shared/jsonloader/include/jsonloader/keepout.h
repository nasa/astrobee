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

#ifndef JSONLOADER_KEEPOUT_H_
#define JSONLOADER_KEEPOUT_H_

#include <Eigen/Geometry>

#include <vector>

namespace jsonloader {

// Represents a single keepout zone
class Keepout {
 public:
  using BoundingBox = Eigen::AlignedBox3f;
  using Sequence = std::vector<BoundingBox>;

  Keepout(Sequence const& boxes, bool const safe);
  explicit Keepout(bool const safe);

  // Get the bounding boxes of this keepout zone
  Sequence const& GetBoxes() const;

  // Merge another zone into this one
  void Merge(Keepout const& zone);

  // Is this a safe area for the robot to be or not?
  bool IsSafe() const noexcept;

 private:
  Sequence bboxes_;  // [ (x0, y0, z0, x1, y1, z1), ... ]
  bool safe_zone_;  // if true, positive spacesz
};

}  // end namespace jsonloader

#endif  // JSONLOADER_KEEPOUT_H_
