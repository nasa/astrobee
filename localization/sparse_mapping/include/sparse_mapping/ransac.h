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

#ifndef SPARSE_MAPPING_RANSAC_H_
#define SPARSE_MAPPING_RANSAC_H_

#include <glog/logging.h>

#include <ctime>
#include <climits>

#include <string>
#include <vector>
#include <random>
#include <limits>


namespace sparse_mapping {

  void get_n_unique_integers(int min_val, int max_val, int num,
                             std::mt19937 * generator, std::vector<int> * values);

/// RANSAC Driver class
template <class FittingFuncT, class ErrorFuncT>
class RandomSampleConsensus {
  const FittingFuncT& m_fitting_func;
  const ErrorFuncT  & m_error_func;
  int           m_num_iterations;
  double        m_inlier_threshold;
  int           m_min_num_output_inliers;
  bool          m_reduce_min_num_output_inliers_if_no_fit;
  bool          m_increase_threshold_if_no_fit;
  std::mt19937  m_generator;

 public:
  // Returns the list of inliers.
  template <class ContainerT1, class ContainerT2>
  void inliers(typename FittingFuncT::result_type const& H,
               std::vector<ContainerT1>  const& p1,
               std::vector<ContainerT2>  const& p2,
               std::vector<ContainerT1>       & inliers1,
               std::vector<ContainerT2>       & inliers2) const {
    inliers1.clear();
    inliers2.clear();

    for (size_t i = 0; i < p1.size(); i++) {
      if (m_error_func(H, p1[i], p2[i]) < m_inlier_threshold) {
        inliers1.push_back(p1[i]);
        inliers2.push_back(p2[i]);
      }
    }
  }

  // Returns the list of inlier indices.
  template <class ContainerT1, class ContainerT2>
  std::vector<size_t> inlier_indices(typename FittingFuncT::result_type const& H,
                                     std::vector<ContainerT1>  const& p1,
                                     std::vector<ContainerT2>  const& p2) const {
    std::vector<size_t> result;
    for (size_t i = 0; i < p1.size(); i++)
      if (m_error_func(H, p1[i], p2[i]) < m_inlier_threshold)
        result.push_back(i);

    LOG(INFO) << "RANSAC inliers / total = " << result.size() << " / " << p1.size() << ".\n";

    return result;
  }

  void reduce_min_num_output_inliers() {
    m_min_num_output_inliers = static_cast<int>(m_min_num_output_inliers/1.5);
  }

  /// Constructor - Stores all the inputs in member variables
  RandomSampleConsensus(FittingFuncT const& fitting_func,
                        ErrorFuncT   const& error_func,
                        int    num_iterations,
                        double inlier_threshold,
                        int    min_num_output_inliers,
                        bool   reduce_min_num_output_inliers_if_no_fit,
                        bool   increase_threshold_if_no_fit):
    m_fitting_func(fitting_func), m_error_func(error_func),
    m_num_iterations(num_iterations),
    m_inlier_threshold(inlier_threshold),
    m_min_num_output_inliers(min_num_output_inliers),
    m_reduce_min_num_output_inliers_if_no_fit(reduce_min_num_output_inliers_if_no_fit),
    m_increase_threshold_if_no_fit(increase_threshold_if_no_fit),
    m_generator(std::mt19937(std::time(0))) {}

  /// As attempt_ransac but keep trying with smaller numbers of required inliers.
  template <class ContainerT1, class ContainerT2>
  typename FittingFuncT::result_type operator()(std::vector<ContainerT1> const& p1,
                                                std::vector<ContainerT2> const& p2) {
    // Try to fit using RANSAC. Perform repeated fits with smaller
    // m_min_num_output_inliers if the fit fails and
    // m_reduce_min_num_output_inliers_if_no_fit is true.

    typename FittingFuncT::result_type H;
    bool success = false;

    int orig_num_inliers = m_min_num_output_inliers;

    // Try various attempts while getting ever more generous with the
    // threshold and the number of liners. for each of these, there
    // will be m_num_iterations attempts.
    for (int attempt_thresh = 0; attempt_thresh < 10; attempt_thresh++) {
      for (int attempt_inlier = 0; attempt_inlier < 10; attempt_inlier++) {
        try {
          H = attempt_ransac(p1, p2);
          success = true;
          break;
        } catch (const std::exception& e) {
          LOG(INFO) << e.what() << "\n";
          if (!m_reduce_min_num_output_inliers_if_no_fit)
            break;
          reduce_min_num_output_inliers();
          // A similarity transform needs at least 3 samples
          if (m_min_num_output_inliers < 3)
            break;
          LOG(INFO) << "Attempting RANSAC with " << m_min_num_output_inliers
                    << " output inliers.\n";
        }
      }

      if (success)
        break;
      if (!m_increase_threshold_if_no_fit)
        break;

      m_min_num_output_inliers = orig_num_inliers;  // restore this
      m_inlier_threshold *= 1.5;
      LOG(INFO) << "Increasing the inlier threshold to: " << m_inlier_threshold << ".\n";
    }

    if (!success)
      LOG(FATAL) << "RANSAC was unable to find a fit that matched the supplied data.";

    return H;
  }

  /// Run RANSAC on two input data lists using the current parameters.
  template <class ContainerT1, class ContainerT2>
  typename FittingFuncT::result_type attempt_ransac(std::vector<ContainerT1> const& p1,
                                                    std::vector<ContainerT2> const& p2) {
    if (p1.empty())
      LOG(FATAL) << "RANSAC error. Insufficient data.\n";
    if (p1.size() != p2.size())
      LOG(FATAL) << "RANSAC error. Data vectors are not the same size.\n";

    int min_elems_for_fit = m_fitting_func.min_elements_needed_for_fit();

    if (static_cast<int>(p1.size()) < min_elems_for_fit)
      LOG(FATAL) << "RANSAC error. Not enough potential matches for this fitting functor.\n";

    // This one we want to catch.
    if (m_min_num_output_inliers < min_elems_for_fit)
      throw std::runtime_error("RANSAC error. Number of requested inliers is less than "
                               "min number of elements needed for fit.\n");

    typename FittingFuncT::result_type best_H;

    std::vector<ContainerT1> try1;
    std::vector<ContainerT2> try2;
    std::vector<int> random_indices(min_elems_for_fit);

    int num_inliers = 0;
    double min_err = std::numeric_limits<double>::max();
    for (int iteration = 0; iteration < m_num_iterations; iteration++) {
      // Get min_elems_for_fit points at random, taking care not to
      // select the same point twice.
      int num = p1.size();
      get_n_unique_integers(0, num - 1, min_elems_for_fit, &m_generator, &random_indices);

      // Resizing below is essential, as by now their size may have changed
      try1.resize(min_elems_for_fit);
      try2.resize(min_elems_for_fit);
      for (int i = 0; i < min_elems_for_fit; i++) {
        try1[i] = p1[random_indices[i]];
        try2[i] = p2[random_indices[i]];
      }

      // 1. Compute the fit using these samples.
      typename FittingFuncT::result_type H = m_fitting_func(try1, try2);

      // 2. Find all the inliers for this fit.
      inliers(H, p1, p2, try1, try2);

      // 3. Skip this model if too few inliers.
      if (static_cast<int>(try1.size()) < m_min_num_output_inliers)
        continue;

      // 4. Re-estimate the model using the inliers.
      H = m_fitting_func(try1, try2);

      // 5. Find the mean error for the inliers.
      double err_val = 0.0;
      for (size_t i = 0; i < try1.size(); i++)
        err_val += m_error_func(H, try1[i], try2[i]);
      err_val /= try1.size();

      // 6. Save this model if its error is lowest so far.
      if (err_val < min_err) {
        min_err     = err_val;
        best_H      = H;
        num_inliers = try1.size();
      }
    }

    if (num_inliers < m_min_num_output_inliers) {
      // Here throw, to be able to catch it in the caller.
      std::ostringstream os;
      os << "RANSAC was unable to find a fit with " << m_min_num_output_inliers << " inliers.";
      throw std::runtime_error(os.str());
    }
    return best_H;
  }
};  // End of RandomSampleConsensus class definition

  // Helper function to instantiate a RANSAC class object and immediately call it
  template <class ContainerT1, class ContainerT2, class FittingFuncT, class ErrorFuncT>
  typename FittingFuncT::result_type ransac(std::vector<ContainerT1> const& p1,
                                            std::vector<ContainerT2> const& p2,
                                            FittingFuncT             const& fitting_func,
                                            ErrorFuncT               const& error_func,
                                            int     num_iterations,
                                            double  inlier_threshold,
                                            int     min_num_output_inliers,
                                            bool    reduce_min_num_output_inliers_if_no_fit,
                                            bool    increase_threshold_if_no_fit) {
    RandomSampleConsensus < FittingFuncT, ErrorFuncT>
      ransac_instance(fitting_func, error_func,
                      num_iterations, inlier_threshold,
                      min_num_output_inliers,
                      reduce_min_num_output_inliers_if_no_fit,
                      increase_threshold_if_no_fit);
    return ransac_instance(p1, p2);
  }

}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_RANSAC_H_
