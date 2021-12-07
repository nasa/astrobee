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

#include <sparse_mapping/reprojection.h>
#include <sparse_mapping/sparse_mapping.h>

#include <ff_common/thread.h>
#include <camera/camera_model.h>

#include <ceres/rotation.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <gflags/gflags.h>

#include <random>
#include <thread>
#include <unordered_map>

DEFINE_uint64(num_min_localization_inliers, 10,
              "If fewer than this many number of inliers, localization has failed.");

namespace sparse_mapping {

ceres::LossFunction* GetLossFunction(std::string cost_fun, double th) {
  // Convert to lower-case
  std::transform(cost_fun.begin(), cost_fun.end(), cost_fun.begin(), ::tolower);

  ceres::LossFunction* loss_function = NULL;
  if      ( cost_fun == "l2"     )
    loss_function = NULL;
  else if ( cost_fun == "huber"  )
    loss_function = new ceres::HuberLoss(th);
  else if ( cost_fun == "cauchy" )
    loss_function = new ceres::CauchyLoss(th);
  else if ( cost_fun == "l1"     )
    loss_function = new ceres::SoftLOneLoss(th);
  else
    LOG(FATAL) << "Unknown cost function: " + cost_fun;

  return loss_function;
}

// if parms is null, don't worry about converting to pixels
struct ReprojectionError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit ReprojectionError(const Eigen::Vector2d & observed)
    : observed(observed) {}

  template <typename T>
  bool operator()(const T* const camera_p_global,
                  const T* const camera_aa_global,
                  const T* const point_global,
                  const T* const focal_length,
                  T* residuals) const {
    // Project the point into the camera's coordinate frame
    T p[3];
    ceres::AngleAxisRotatePoint(camera_aa_global, point_global, p);
    p[0] += camera_p_global[0];
    p[1] += camera_p_global[1];
    p[2] += camera_p_global[2];

    T xp = (p[0] / p[2]) * focal_length[0];
    T yp = (p[1] / p[2]) * focal_length[0];

    // The error is the difference between the prediction and observed
    residuals[0] = xp - T(observed.x());
    residuals[1] = yp - T(observed.y());

    return true;
  }

  // Helper function ... make the code look nice
  static ceres::CostFunction* Create(const Eigen::Vector2d & observed) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 3, 1>
            (new ReprojectionError(observed)));
  }

  Eigen::Vector2d observed;
};

void BundleAdjust(std::vector<std::map<int, int> > const& pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map, double focal_length,
                  std::vector<Eigen::Affine3d>* cid_to_cam_t_global, std::vector<Eigen::Vector3d>* pid_to_xyz,
                  std::vector<std::map<int, int> > const& user_pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd> const& user_cid_to_keypoint_map,
                  std::vector<Eigen::Vector3d>* user_pid_to_xyz, ceres::LossFunction* loss,
                  ceres::Solver::Options const& options, ceres::Solver::Summary* summary, int first, int last,
                  bool fix_all_cameras, std::set<int> const& fixed_cameras) {
  // Perform bundle adjustment. Keep fixed all cameras with cid
  // not within [first, last] and all xyz points which project only
  // onto fixed cameras.

  // If provided, use user-set registration points in the second pass.

  // Allocate space for the angle axis representation of rotation
  std::vector<double> camera_aa_storage(3 * cid_to_cam_t_global->size());
  for (size_t cid = 0; cid < cid_to_cam_t_global->size(); cid++) {
    Eigen::Map<Eigen::Vector3d> aa_storage(camera_aa_storage.data() + 3 * cid);
    Eigen::Vector3d vec;
    camera::RotationToRodrigues(cid_to_cam_t_global->at(cid).linear(),
                               &vec);
    aa_storage = vec;
  }

  // Build problem
  ceres::Problem problem;

  // Ideally the block inside of the loop below must be a function call,
  // but the compiler does not handle that correctly with ceres.
  // So do this by changing where things are pointing.

  int num_passes = 1;
  if (!user_pid_to_xyz->empty()) num_passes = 2;  // A second pass using control points

  for (int pass = 0; pass < num_passes; pass++) {
    std::vector<std::map<int, int> > const * p_pid_to_cid_fid;
    std::vector<Eigen::Matrix2Xd >   const * p_cid_to_keypoint_map;
    std::vector<Eigen::Vector3d>           * p_pid_to_xyz;
    ceres::LossFunction * local_loss;
    if (pass == 0) {
      local_loss            = loss;  // outside-supplied loss
      p_pid_to_cid_fid      = &pid_to_cid_fid;
      p_cid_to_keypoint_map = &cid_to_keypoint_map;
      p_pid_to_xyz          = pid_to_xyz;
    } else {
      local_loss            = NULL;  // l2, as user-supplied data is reliable
      p_pid_to_cid_fid      = &user_pid_to_cid_fid;
      p_cid_to_keypoint_map = &user_cid_to_keypoint_map;
      p_pid_to_xyz          = user_pid_to_xyz;
    }

    for (size_t pid = 0; pid < p_pid_to_xyz->size(); pid++) {
      if ((*p_pid_to_cid_fid)[pid].size() < 2)
        LOG(FATAL) << "Found a track of size < 2.";

      // Don't vary points which project only into cameras which we don't vary.
      bool fix_pid = true;
      for (std::map<int, int>::value_type const& cid_fid : (*p_pid_to_cid_fid)[pid]) {
        if (cid_fid.first >= first && cid_fid.first <= last)
          fix_pid = false;
      }

      for (std::map<int, int>::value_type const& cid_fid : (*p_pid_to_cid_fid)[pid]) {
        ceres::CostFunction* cost_function =
          ReprojectionError::Create((*p_cid_to_keypoint_map)[cid_fid.first].col(cid_fid.second));

        problem.AddResidualBlock(cost_function,
                                 local_loss,
                                 &cid_to_cam_t_global->at(cid_fid.first).translation()[0],
                                 &camera_aa_storage[3 * cid_fid.first],
                                 &p_pid_to_xyz->at(pid)[0],
                                 &focal_length);

        if (fix_all_cameras || (cid_fid.first < first || cid_fid.first > last) ||
            fixed_cameras.find(cid_fid.first) != fixed_cameras.end()) {
          problem.SetParameterBlockConstant(&cid_to_cam_t_global->at(cid_fid.first).translation()[0]);
          problem.SetParameterBlockConstant(&camera_aa_storage[3 * cid_fid.first]);
        }
      }
      if (fix_pid || pass == 1) {
        // Fix pids which don't project in cameras that are floated.
        // Also, must not float points given by the user, those are measurements
        // we are supposed to reference ourselves against, and floating
        // them can make us lose the real world scale.
        problem.SetParameterBlockConstant(&p_pid_to_xyz->at(pid)[0]);
      }
    }
    problem.SetParameterBlockConstant(&focal_length);
  }

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  for (size_t cid = 0; cid < cid_to_cam_t_global->size(); cid++) {
    Eigen::Map<Eigen::Vector3d> aa_storage
      (camera_aa_storage.data() + 3 * cid);
    Eigen::Matrix3d r;
    camera::RodriguesToRotation(aa_storage, &r);
    cid_to_cam_t_global->at(cid).linear() = r;
  }
}

// This is a very specialized function
void BundleAdjustSmallSet(std::vector<Eigen::Matrix2Xd> const& features_n,
                          double focal_length,
                          std::vector<Eigen::Affine3d> * cam_t_global_n,
                          Eigen::Matrix3Xd * pid_to_xyz,
                          ceres::LossFunction * loss,
                          ceres::Solver::Options const& options,
                          ceres::Solver::Summary * summary) {
  CHECK(cam_t_global_n) << "Variable cam_t_global_n needs to be defined";
  CHECK(cam_t_global_n->size() == features_n.size())
    << "Variables features_n and cam_t_global_n need to agree on the number of cameras";
  CHECK(cam_t_global_n->size() > 1) << "Bundle adjust needs at least 2 or more cameras";
  CHECK(pid_to_xyz->cols() == features_n[0].cols())
    << "There should be an equal amount of XYZ points as there are feature observations";
  for (size_t i = 1; i < features_n.size(); i++) {
    CHECK(features_n[0].cols() == features_n[i].cols())
      << "The same amount of features should be seen in all cameras";
  }

  const size_t n_cameras = features_n.size();

  // Allocate space for the angle axis representation of rotation
  std::vector<Eigen::Vector3d> aa(n_cameras);
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RotationToRodrigues(cam_t_global_n->at(cid).linear(), &aa[cid]);
  }

  // Build the problem
  ceres::Problem problem;
  for (ptrdiff_t pid = 0; pid < pid_to_xyz->cols(); pid++) {
    for (size_t cid = 0; cid < n_cameras; cid++) {
      ceres::CostFunction* cost_function = ReprojectionError::Create(features_n[cid].col(pid));
      problem.AddResidualBlock(cost_function, loss,
                               &cam_t_global_n->at(cid).translation()[0],
                               &aa.at(cid)[0],
                               &pid_to_xyz->col(pid)[0],
                               &focal_length);
    }
  }
  problem.SetParameterBlockConstant(&focal_length);

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  Eigen::Matrix3d r;
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RodriguesToRotation(aa[cid], &r);
    cam_t_global_n->at(cid).linear() = r;
  }
}

void EstimateCamera(camera::CameraModel* camera_estimate,
                    std::vector<Eigen::Vector3d>* landmarks,
                    const std::vector<Eigen::Vector2d> & observations,
                    const ceres::Solver::Options & options,
                    ceres::Solver::Summary* summary) {
  Eigen::Affine3d guess = camera_estimate->GetTransform();
  camera::CameraParameters params = camera_estimate->GetParameters();

  // Initialize the angle axis representation of rotation
  Eigen::Vector3d aa;
  camera::RotationToRodrigues(guess.linear(), &aa);

  double focal_length = params.GetFocalLength();

  // Build problem
  ceres::Problem problem;
  for (size_t pid = 0; pid < landmarks->size(); pid++) {
    ceres::CostFunction* cost_function = ReprojectionError::Create(
                Eigen::Vector2d(observations[pid].x(), observations[pid].y()));
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0),
                             &guess.translation()[0],
                             &aa[0],
                             &landmarks->at(pid)[0],
                             &focal_length);
    problem.SetParameterBlockConstant(&landmarks->at(pid)[0]);
  }
  problem.SetParameterBlockConstant(&focal_length);

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  Eigen::Matrix3d r;
  camera::RodriguesToRotation(aa, &r);
  guess.linear() = r;
  camera_estimate->SetTransform(guess);
}

// random intger in [min, max)
int RandomInt(int min, int max) {
  static std::mt19937 generator;  // should be thread_local for thread safe, gcc 4.6 doesn't support
  std::uniform_int_distribution<int> random_item(min, max - 1);
  return random_item(generator);
}

void SelectRandomObservations(const std::vector<Eigen::Vector3d> & all_landmarks,
        const std::vector<Eigen::Vector2d> & all_observations, size_t num_selected,
        std::vector<cv::Point3d> * landmarks, std::vector<cv::Point2d> * observations) {
  std::unordered_map<int, int> used;
  // not enough observations
  if (all_observations.size() < num_selected)
    return;
  // Reserve space in the output so we don't have to keep reallocating on
  // push_back().
  landmarks->reserve(num_selected);
  observations->reserve(num_selected);
  while (observations->size() < num_selected) {
    int id = RandomInt(0, all_observations.size());
    if (used.count(id) > 0)
      continue;
    Eigen::Vector3d p = all_landmarks[id];
    landmarks->push_back(cv::Point3d(p[0], p[1], p[2]));
    observations->push_back(cv::Point2d(all_observations[id][0], all_observations[id][1]));
    used[id] = 1;
  }
}

bool P3P(const std::vector<cv::Point3d> & landmarks, const std::vector<cv::Point2d> & observations,
         const camera::CameraParameters & params, Eigen::Vector3d * pos, Eigen::Matrix3d * rotation) {
    cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
    cv::eigen2cv(params.GetIntrinsicMatrix<camera::UNDISTORTED_C>(), camera_matrix);
    cv::Mat rvec(3, 1, cv::DataType<double>::type, cv::Scalar(0));
    cv::Mat tvec(3, 1, cv::DataType<double>::type, cv::Scalar(0));
    cv::Mat distortion(4, 1, cv::DataType<double>::type, cv::Scalar(0));
    bool result = cv::solvePnP(landmarks, observations, camera_matrix, distortion, rvec, tvec, false, cv::SOLVEPNP_P3P);
    if (!result)
      return false;
    cv::cv2eigen(tvec, *pos);
    camera::RodriguesToRotation(Eigen::Vector3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), rotation);
    return true;
}

size_t CountInliers(const std::vector<Eigen::Vector3d> & landmarks, const std::vector<Eigen::Vector2d> & observations,
                 const camera::CameraModel & camera, int tolerance, std::vector<size_t>* inliers) {
  int num_inliers = 0;
  if (inliers) {
    // To save ourselves some allocation time. We'll prealloc for a 50% inlier
    // success rate
    inliers->reserve(observations.size()/2);
  }

  double tolerance_sq = tolerance * tolerance;

  for (size_t i = 0; i < landmarks.size(); i++) {
    Eigen::Vector2d pos = camera.ImageCoordinates(landmarks[i]);
    if ((observations[i] - pos).squaredNorm() <= tolerance_sq) {
      num_inliers++;
      if (inliers)
        inliers->push_back(i);
    }
  }
  return num_inliers;
}

int RansacEstimateCamera(const std::vector<Eigen::Vector3d> & landmarks,
                         const std::vector<Eigen::Vector2d> & observations,
                         int num_tries, int inlier_tolerance, camera::CameraModel * camera_estimate,
                         std::vector<Eigen::Vector3d> * inlier_landmarks_out,
                         std::vector<Eigen::Vector2d> * inlier_observations_out,
                         bool verbose) {
  size_t best_inliers = 0;
  camera::CameraParameters params = camera_estimate->GetParameters();

  // Need the minimum number of observations
  if (observations.size() < 4)
    return 1;

  // RANSAC to find the best camera with P3P
  std::vector<cv::Point3d> subset_landmarks;
  std::vector<cv::Point2d> subset_observations;
  // TODO(oalexan1): Use multiple threads here?
  for (int i = 0; i < num_tries; i++) {
    subset_landmarks.clear();
    subset_observations.clear();
    SelectRandomObservations(landmarks, observations, 4, &subset_landmarks, &subset_observations);

    Eigen::Vector3d pos;
    Eigen::Matrix3d rotation;
    bool result = P3P(subset_landmarks, subset_observations, params, &pos, &rotation);
    if (!result)
      continue;
    Eigen::Affine3d cam_t_global;
    cam_t_global.setIdentity();
    cam_t_global.translate(pos);
    cam_t_global.rotate(rotation);
    camera::CameraModel guess(cam_t_global, camera_estimate->GetParameters());

    size_t inliers = CountInliers(landmarks, observations, guess, inlier_tolerance, NULL);
    if (inliers > best_inliers) {
      best_inliers = inliers;
      *camera_estimate = guess;
    }
  }

  if (verbose)
    std::cout << observations.size() << " Ransac observations "
              << best_inliers << " inliers\n";

  // TODO(bcoltin): Return some sort of confidence?
  if (best_inliers < FLAGS_num_min_localization_inliers)
    return 2;

  std::vector<size_t> inliers;
  CountInliers(landmarks, observations, *camera_estimate, inlier_tolerance, &inliers);
  std::vector<Eigen::Vector3d> inlier_landmarks;
  std::vector<Eigen::Vector2d> inlier_observations;
  inlier_landmarks.reserve(inliers.size());
  inlier_observations.reserve(inliers.size());
  for (size_t idx : inliers) {
    inlier_landmarks.push_back(landmarks[idx]);
    inlier_observations.push_back(observations[idx]);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1;  // it is no slower with only one thread
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  // improve estimate with CERES solver
  EstimateCamera(camera_estimate, &inlier_landmarks, inlier_observations, options, &summary);

  // find inliers again with refined estimate
  inliers.clear();
  best_inliers = CountInliers(landmarks, observations, *camera_estimate, inlier_tolerance, &inliers);

  if (verbose)
    std::cout << "Number of inliers with refined camera: " << best_inliers << "\n";

  if (best_inliers < FLAGS_num_min_localization_inliers)
    return 2;

  inlier_landmarks.clear();
  inlier_observations.clear();
  inlier_landmarks.reserve(inliers.size());
  inlier_observations.reserve(inliers.size());
  for (size_t idx : inliers) {
    inlier_landmarks.push_back(landmarks[idx]);
    inlier_observations.push_back(observations[idx]);
  }
  if (inlier_landmarks_out) {
    inlier_landmarks_out->reserve(inliers.size());
    std::copy(inlier_landmarks.begin(), inlier_landmarks.end(),
        std::back_inserter(*inlier_landmarks_out));
  }
  if (inlier_observations_out) {
    inlier_observations_out->reserve(inliers.size());
    std::copy(inlier_observations.begin(), inlier_observations.end(),
        std::back_inserter(*inlier_observations_out));
  }

  return 0;
}

// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm

void Find3DAffineTransform(Eigen::Matrix3Xd const & in,
                           Eigen::Matrix3Xd const & out,
                           Eigen::Affine3d* result) {
  // Default output
  result->linear() = Eigen::Matrix3d::Identity(3, 3);
  result->translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // Local copies we can modify
  Eigen::Matrix3Xd local_in = in, local_out = out;

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < local_in.cols()-1; col++) {
    dist_in  += (local_in.col(col+1) - local_in.col(col)).norm();
    dist_out += (local_out.col(col+1) - local_out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return;
  double scale = dist_out/dist_in;
  local_out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < local_in.cols(); col++) {
    in_ctr  += local_in.col(col);
    out_ctr += local_out.col(col);
  }
  in_ctr /= local_in.cols();
  out_ctr /= local_out.cols();
  for (int col = 0; col < local_in.cols(); col++) {
    local_in.col(col)  -= in_ctr;
    local_out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::Matrix3d Cov = local_in * local_out.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  result->linear() = scale * R;
  result->translation() = scale*(out_ctr - R*in_ctr);
}

}  // namespace sparse_mapping
