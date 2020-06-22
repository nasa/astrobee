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

#include <ff_common/init.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/reprojection.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <sys/time.h>
#include <thread>

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);
  if (argc < 3) {
    std::cerr << "Usage: evaluate_localization map.nvm testfile.txt\n";
    std::exit(0);
  }

  // initialize map
  std::string map_file = argv[1];
  std::string test_file = argv[2];

  sparse_mapping::SparseMap map(map_file);

  int failures = 0;
  int trials = 0;

  double pos_error_sum     = 0.0;
  double pos_error_sum_2   = 0.0;
  double angle_error_sum   = 0.0;
  double angle_error_sum_2 = 0.0;
  double total_time        = 0.0;
  double total_time_2      = 0.0;

  FILE* f = fopen(test_file.c_str(), "r");
  while (true) {
    double x, y, z;
    Eigen::Matrix3d rot;
    char name[255];
    int values = fscanf(f, "%s (%lf, %lf, %lf) [%lf %lf %lf, %lf %lf %lf, %lf %lf %lf]\n",
                  name,
                  &x, &y, &z,
                  &rot(0, 0), &rot(0, 1), &rot(0, 2),
                  &rot(1, 0), &rot(1, 1), &rot(1, 2),
                  &rot(2, 0), &rot(2, 1), &rot(2, 2));
    if (values < 1)
      break;
    Eigen::Vector3d pos(x, y, z);

    trials++;
    camera::CameraModel camera(Eigen::Vector3d(), Eigen::Matrix3d::Identity(), map.GetCameraParameters());
    struct timeval a, b;
    gettimeofday(&a, NULL);
    if (!map.Localize(name, &camera)) {
      printf("%s Failure\n", name);
      failures++;
      continue;
    }
    gettimeofday(&b, NULL);

    double dist_error  = (pos - camera.GetPosition()).norm();
    pos_error_sum   += dist_error;
    pos_error_sum_2 += dist_error * dist_error;

    Eigen::Vector3d expected_angle = rot * Eigen::Vector3d::UnitX();
    Eigen::Vector3d estimated_angle = camera.GetRotation() * Eigen::Vector3d::UnitX();
    double angle_error = acos(estimated_angle.dot(expected_angle));
    angle_error_sum   += angle_error;
    angle_error_sum_2 += angle_error * angle_error;
    double t = b.tv_sec - a.tv_sec + (b.tv_usec - a.tv_usec) / 1000000.0;
    total_time += t;
    total_time_2 += t * t;

    printf("%s %g %g %g\n", name, t, dist_error, angle_error);
  }
  fclose(f);

  int suc = trials - failures;
  printf("Success Rate: %d / %d\n", suc, trials);
  printf("Distance Error: %g +/- %g\n", pos_error_sum / suc,
          sqrt(pos_error_sum_2 / suc - pow(pos_error_sum / suc, 2)));
  printf("Angle Error: %g +/- %g\n", angle_error_sum / suc,
          sqrt(angle_error_sum_2 / suc - pow(angle_error_sum / suc, 2)));
  printf("Time: %g +/- %g s\n", total_time / suc,
          sqrt(total_time_2 / suc - pow(total_time / suc, 2)));

  return 0;
}
