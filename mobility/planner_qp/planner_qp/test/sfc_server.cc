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

#include <decomp_util/ellipse_decomp.h>
#include <jps3d/planner/jps_3d_util.h>

std::shared_ptr<JPS::VoxelMapUtil> jps_map_util_;
std::unique_ptr<EllipseDecomp> decomp_util_;
std::unique_ptr<JPS::JPS3DUtil> jps_planner_;

void load_map() {
  // std::vector<ff_msgs::Zone> zones;
  Vec3f origin(0, 0, 0);
  Vec3i dim(10, 10, 10);
  int num_cell = dim(0) * dim(1) * dim(2);
  double res = 0.5;

  jps_map_util_.reset(new JPS::VoxelMapUtil());
  jps_map_util_->setMap(origin, dim, std::vector<signed char>(num_cell, 0),
                        res);

  jps_planner_.reset(new JPS::JPS3DUtil(false));
  jps_planner_->setMapUtil(jps_map_util_.get());

  decomp_util_.reset(new EllipseDecomp(
      jps_map_util_->getOrigin(),
      jps_map_util_->getDim().cast<decimal_t>() * jps_map_util_->getRes(),
      true));
  decomp_util_->set_obstacles(jps_map_util_->getCloud());

  jps_map_util_->freeUnKnown();
}
void test_plan() {
  Vec3f start3(1.0, 1.0, 1.0);
  Vec3f goal3(4.0, 4.0, 4.0);
  if (!jps_planner_->plan(start3, goal3))
    std::cout << "JPS Failed" << std::endl;
  else
    std::cout << "JPS Got Path" << std::endl;
}
int main(int argc, char** argv) {
  // ros::init(argc, argv, "test");
  // ros::NodeHandle nh("~");

  load_map();
  test_plan();

  return 0;
}
