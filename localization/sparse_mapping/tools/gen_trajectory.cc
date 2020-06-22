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
#include <ff_common/thread.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sparse_map.pb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

// Create a P2 trajectory. Each line has the format:

// X Y Z QX QY QZ QW 0 0 0 0 0 0

// We don't use the last 6 zeros. They represent velocities, but
// despite being zero, P2 will just float through the way points.

// The granite lab has +Z facing up. P2's Z location will always be
// -.7. Keep x and y within [-0.6, 0.6]. The quaternion represents
// world_q_body. The camera is on +X body. Body's +Z is facing into
// the granite surface. The dock's location is at world's -X.

DEFINE_int32(trajectory_type, 1,
             "Trajectory type, 1: spiral-like closed path, 2: spiral-like open path, "
             "3: circular trajectory with a complex camera motion.");
DEFINE_int32(num_loops, 3,
              "The number of turns in the trajectory.");
DEFINE_int32(num_samples, 50,
              "The number sample points in the trajectory.");
DEFINE_string(trajectory_file, "trajectory.csv",
              "Save the trajectory to this file.");

int main(int argc, char** argv) {
  ff_common::InitFreeFlyerApplication(&argc, &argv);

  double z = -0.7;   // z is constant as the table is flat
  double mx = 0.6;  // x and y are within [-mx, mx]

  LOG(INFO) << "Writing: " << FLAGS_trajectory_file;
  std::ofstream tr(FLAGS_trajectory_file.c_str());
  tr.precision(18);

  // The bot will move away from the origin in a spiral pattern, then move
  // back to the origin within the same pattern.
  double max_angle = FLAGS_num_loops*2*M_PI;  // max angle, in radians
  for (int i = 0; i < FLAGS_num_samples; i++) {
    double a = (max_angle*i)/(FLAGS_num_samples-1.0);  // angle in radians

    double r;
    if (FLAGS_trajectory_type == 1) {
      r = 4.0*a*(max_angle-a) / max_angle / max_angle;  // 0 <= r <= 1
      r = r*r;                                          // make it smoother
      double d = 0.1; r = mx*(d + r)/(d + 1.0);         // stay away from origin
    } else if (FLAGS_trajectory_type == 2) {
      r = mx*a/max_angle;
    } else {
      r = mx;  // constant radius, just the orientation changes
    }

    double factor = static_cast<int>(a/(8*M_PI));
    if (FLAGS_trajectory_type == 3) {
      // Keep the angle from 0 to 8*pi
      a = a - 8*M_PI*factor;
    }

    double c = 0, s = 0;
    if (FLAGS_trajectory_type == 1 || FLAGS_trajectory_type == 2) {
      c = cos(a); s = sin(a);
    } else {
      // r = i + 0.1*FLAGS_num_samples;  // temporary, for debugging purposes

      if (a <= 2*M_PI) {
        // Camera points outward
        c = cos(a); s = sin(a);
      } else if (a <= 4*M_PI) {
        // Camera transitions from pointing outward to pointing inward
        c = cos((a-2*M_PI)/2); s = sin((a-2*M_PI)/2);
      } else if (a <= 6*M_PI) {
        // Camera points inward
        c = -cos(a); s = -sin(a);
      } else if (a <= 8*M_PI) {
        // Camera transitions from pointing inward to pointing outward
        c = -cos((a-6*M_PI)/2); s = -sin((a-6*M_PI)/2);
      } else {
        LOG(FATAL) << "Hit unhandled a value";
      }
    }

    Eigen::Vector3d P; P << r*cos(a), r*sin(a), z;
    Eigen::Matrix3d T;

    // The first column represent the body x direction in the world coordinates,
    // second column is body's y, and third is body's z.
    T << c, s, 0, s, -c, 0, 0, 0, -1;

    // Convert from body-to-world to world-to-body
    Eigen::Matrix3d TI = T.inverse();
    Eigen::Quaternion<double> Q(TI);

    tr << P[0] << ' ' << P[1] << ' ' << P[2] << ' ';
    tr <<  Q.coeffs().transpose() << " 0 0 0 0 0 0" << std::endl;
  }

  tr.close();

  return 0;
}
