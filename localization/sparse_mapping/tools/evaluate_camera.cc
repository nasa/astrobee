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

/**
 * Given camera parameters and noise estimates, this program
 * estimates the error that global localization via either
 * RANSAC or least-squares will have.
 **/
#include <common/init.h>
#include <common/thread.h>
#include <camera/camera_model.h>
#include <sparse_mapping/reprojection.h>

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/features2d/features2d.hpp>

DEFINE_bool(ransac, false,
              "Use RANSAC algorithm. If not use ceres solver.");

// Observation parameters
DEFINE_uint64(num_observations, 20,
             "Number of landmarks to observe.");
DEFINE_double(map_error, 0.01,
             "Std. dev. error of the mapped position, in m, for each dimension.");
DEFINE_double(feature_error, 0,
             "Std. dev. error from feature detection, in pixels, for each dimension.");
DEFINE_double(mismatch_prob, 0.0,
             "The probability of an observation matching to the wrong landmark.");

// Camera parameters
DEFINE_double(fov_x, 170.0,
              "Horizontal field of view in degrees.");
DEFINE_int32(xres, 640,
             "Horizontal resolution.");
DEFINE_int32(yres, 480,
             "Vertical resolution.");

// Environment parameters
DEFINE_double(visible_distance, 8.0,
             "Distance features are visible to camera.");
DEFINE_double(env_width, 6.0,
             "Width of the rectangular prism camera is inside.");
DEFINE_double(env_height, 6.0,
             "Height of the rectangular prism camera is inside.");
DEFINE_double(env_length, 20.0,
             "Length of the rectangular prism camera is inside.");
DEFINE_double(env_clutter, 0.5,
             "Distance within landmarks are located from the walls.");

// Initial error parameters
DEFINE_double(guess_angle_error, 0.04 * M_PI,
             "Std. dev. angle error of initial transform guess.");
DEFINE_double(guess_distance_error, 0.5,
             "Std. dev. distance error of initial transform guess.");

// Number of trials
DEFINE_int32(num_trials, 250,
             "Number of trials to gather statistics on");

// Ceres parameters
DEFINE_int32(max_num_iterations, 10000,
             "Maximum number of iterations for solver");

// RANSAC parameters
DEFINE_int32(ransac_inlier_tolerance, 3,
              "Maximum error in pixels for a projection to be considered correct.");
DEFINE_int32(ransac_iterations, 100,
              "Number of iterations of RANSAC.");


unsigned int rand_seed;
double rand_d() {
  return static_cast<double>(rand_r(&rand_seed)) / RAND_MAX;
}

Eigen::Vector3d EnvironmentIntersection(Eigen::Vector3d origin, Eigen::Vector3d dir) {
  double W = FLAGS_env_width / 2, H = FLAGS_env_height / 2, L = FLAGS_env_length / 2, P = FLAGS_env_clutter;
  // check each plane for intersection
  // x = -W plane
  // origin + dir * t = (-W, a, b)
  double t = (-W - origin.x()) / dir.x();
  if (t >= 0) {
    Eigen::Vector3d p = origin + t * dir;
    if (p.z() >= -L && p.z() <= L && p.y() >= -H && p.y() <= H)
      return p + rand_d() * P * Eigen::Vector3d(0, 0, 1);
  }
  // x = W
  t = (W - origin.x()) / dir.x();
  if (t >= 0) {
    Eigen::Vector3d p = origin + t * dir;
    if (p.z() >= -L && p.z() <= L && p.y() >= -H && p.y() <= H)
      return p + rand_d() * P * Eigen::Vector3d(0, 0, -1);
  }
  // y = -H
  t = (-H - origin.y()) / dir.y();
  if (t >= 0) {
    Eigen::Vector3d p = origin + t * dir;
    if (p.x() >= -W && p.x() <= W && p.z() >= -L && p.z() <= L)
      return p + rand_d() * P * Eigen::Vector3d(0, 1, 0);
  }
  // y = H
  t = (H - origin.y()) / dir.y();
  if (t >= 0) {
    Eigen::Vector3d p = origin + t * dir;
    if (p.x() >= -W && p.x() <= W && p.z() >= -L && p.z() <= L)
      return p + rand_d() * P * Eigen::Vector3d(0, -1, 0);
  }
  // z = -L
  t = (-L - origin.z()) / dir.z();
  if (t >= 0) {
    Eigen::Vector3d p = origin + t * dir;
    if (p.x() >= -W && p.x() <= W && p.y() >= -H && p.y() <= H)
      return p + rand_d() * P * Eigen::Vector3d(0, 0, 1);
  }
  // z = L
  t = (L - origin.z()) / dir.z();
  Eigen::Vector3d p = origin + t * dir;
  return p + rand_d() * P * Eigen::Vector3d(0, 0, -1);
}

Eigen::Vector3d RandomLandmark(std::default_random_engine gen) {
  double W = FLAGS_env_width / 2, H = FLAGS_env_height / 2, L = FLAGS_env_length / 2, P = FLAGS_env_clutter;
  std::uniform_int_distribution<int> face(0, 5);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  std::uniform_real_distribution<double> clutter(0.0, P);
  int f = face(gen);
  if (f == 0)
    return Eigen::Vector3d(uniform(gen) * W, uniform(gen) * H, L - clutter(gen));
  else if (f == 1)
    return Eigen::Vector3d(uniform(gen) * W, uniform(gen) * H, -L + clutter(gen));
  else if (f == 2)
    return Eigen::Vector3d(uniform(gen) * W, H - clutter(gen), uniform(gen) * L);
  else if (f == 3)
    return Eigen::Vector3d(uniform(gen) * W, -H + clutter(gen), uniform(gen) * L);
  else if (f == 4)
    return Eigen::Vector3d(W - clutter(gen), uniform(gen) * H, uniform(gen) * L);
  return Eigen::Vector3d(-W + clutter(gen), uniform(gen) * H, uniform(gen) * L);
}

void GenerateLandmarkObservations(const camera::CameraModel & camera,
                                  std::vector<Eigen::Vector3d>* pid_to_xyz,
                                  std::vector<Eigen::Vector2d>* observations) {
  std::default_random_engine gen;
  std::normal_distribution<double> map_error(0.0, FLAGS_map_error);
  std::normal_distribution<double> feature_error(0.0, FLAGS_feature_error);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  while (pid_to_xyz->size() < FLAGS_num_observations) {
    Eigen::Vector3d ray = camera.Ray(rand_d() * FLAGS_xres, rand_d() * FLAGS_yres);
    Eigen::Vector3d landmark = EnvironmentIntersection(camera.GetPosition(), ray);
    if (!camera.IsInFov(landmark) ||
        (landmark - camera.GetPosition()).norm() > FLAGS_visible_distance)
      continue;
    Eigen::Vector3d map_landmark =
      landmark + Eigen::Vector3d(map_error(gen), map_error(gen), map_error(gen));
    // Purposely introducing rounding error here.
    Eigen::Vector2i image_coords = (camera.ImageCoordinates(landmark).array() + 0.5).cast<int>();
    Eigen::Vector2d obs(image_coords.x() + feature_error(gen), image_coords.y() + feature_error(gen));

    if (FLAGS_mismatch_prob > 0.0 && uniform(gen) <= FLAGS_mismatch_prob)
      map_landmark = RandomLandmark(gen);
    pid_to_xyz->push_back(map_landmark);
    observations->push_back(obs);
  }
}

void EstimateRandomCameraError(double* dist_error, double* angle_error) {
  // initialize random camera model with specified parameters
  double C = 4 * FLAGS_env_clutter;
  double W = FLAGS_env_width - 2 * C, H = FLAGS_env_height - 2 * C, L = FLAGS_env_length - 2 * C;
  Eigen::Vector3d true_camera_pos(rand_d() * W - W / 2, rand_d() * H - H / 2, rand_d() * L - L / 2);
  Eigen::Matrix3d true_camera_rotation;
  true_camera_rotation = Eigen::AngleAxisd(rand_d() * M_2_PI, Eigen::Vector3d::UnitX())
                       * Eigen::AngleAxisd(rand_d() * M_2_PI, Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(rand_d() * M_2_PI, Eigen::Vector3d::UnitZ());
  camera::CameraModel camera(true_camera_pos, true_camera_rotation,
                     FLAGS_fov_x * M_PI / 180.0, FLAGS_xres, FLAGS_yres);

  // create landmarks in field of view and simulate observations
  std::vector<Eigen::Vector3d> pid_to_xyz;  // landmark locations
  std::vector<Eigen::Vector2d> observations;  // observed camera coordinates of each landmark
  GenerateLandmarkObservations(camera, &pid_to_xyz, &observations);

  // create initial pose estimate
  std::default_random_engine gen;
  std::normal_distribution<double> guess_dist_err(0.0, FLAGS_guess_distance_error);
  std::normal_distribution<double> guess_angle_err(0.0, FLAGS_guess_angle_error);
  Eigen::Vector3d guess_pos(true_camera_pos);
  guess_pos += Eigen::Vector3d(guess_dist_err(gen), guess_dist_err(gen), guess_dist_err(gen));
  Eigen::Matrix3d guess_rotation(true_camera_rotation);
  guess_rotation = guess_rotation * Eigen::AngleAxisd(guess_angle_err(gen), Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(guess_angle_err(gen), Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(guess_angle_err(gen), Eigen::Vector3d::UnitZ());
  camera::CameraModel guess(guess_pos, guess_rotation, camera.GetParameters());

  if (!FLAGS_ransac) {
    // Solve the problem
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = FLAGS_num_threads;
    options.max_num_iterations = FLAGS_max_num_iterations;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    sparse_mapping::EstimateCamera(&guess, &pid_to_xyz, observations, options, &summary);
  } else {
    sparse_mapping::RansacEstimateCamera(pid_to_xyz, observations, FLAGS_ransac_iterations,
                                         FLAGS_ransac_inlier_tolerance, &guess);
  }

  Eigen::Vector3d orig_angle = camera.GetRotation() * Eigen::Vector3d::UnitX();
  Eigen::Vector3d observed_angle = guess.GetRotation() * Eigen::Vector3d::UnitX();

  // set errors
  *dist_error = (true_camera_pos - guess.GetPosition()).norm();
  *angle_error = acos(observed_angle.dot(orig_angle));
}

int main(int argc, char** argv) {
  common::InitFreeFlyerApplication(&argc, &argv);
  rand_seed = time(NULL);

  double dist_error_mean = 0.0, dist_error_mean_2 = 0.0;
  double angle_error_mean = 0.0, angle_error_mean_2 = 0.0;
  for (int i = 0; i < FLAGS_num_trials; i++) {
    double dist_error = 0.0, angle_error = 0.0;
    EstimateRandomCameraError(&dist_error, &angle_error);
    dist_error_mean += dist_error;
    dist_error_mean_2 += dist_error * dist_error;
    angle_error_mean += angle_error;
    angle_error_mean_2 += angle_error * angle_error;
  }
  dist_error_mean    /= FLAGS_num_trials;
  angle_error_mean   /= FLAGS_num_trials;
  dist_error_mean_2  /= FLAGS_num_trials;
  angle_error_mean_2 /= FLAGS_num_trials;
  double dist_std_dev = sqrt(dist_error_mean_2 - dist_error_mean * dist_error_mean);
  double angle_std_dev = sqrt(angle_error_mean_2 - angle_error_mean * angle_error_mean);
  std::cout << "Distance Error: " << (dist_error_mean * 100.0) << " +/- " <<
    (dist_std_dev * 100.0) << " cm" << "\n";
  std::cout << "Angle Error: " << (angle_error_mean * 180.0 / M_PI) << " +/- " <<
    (angle_std_dev * 180.0 / M_PI) << " degrees" << "\n";

  return 0;
}
