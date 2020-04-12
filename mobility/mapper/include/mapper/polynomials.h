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

#ifndef MAPPER_POLYNOMIALS_H_
#define MAPPER_POLYNOMIALS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <complex>
#include <iostream>
#include <vector>

#include "ff_msgs/ControlState.h"
#include "ff_msgs/Segment.h"

namespace polynomials {

// Coefficients go from higher order to lower order
//  e.g.: (t-t0)^2 + 2(t-t0) + 3 ==> coeff = [1 2 3]
class Polynomial {
 public:
  double t0_;              // Initial time for definition of polynomial
  double tf_;              // Ginal time for definition of polynomial
  int order_;              // Polynomial order
  Eigen::VectorXd coeff_;  // Coefficients

  // Constructor
  Polynomial(const double t0, const double tf, const int coeff_size);
  Polynomial();

  // Methods
  Polynomial &operator=(const Polynomial &other) {
    t0_ = other.t0_;
    tf_ = other.tf_;
    order_ = other.order_;
    coeff_ = other.coeff_;
    return *this;
  }
  void PrintPolyCoeff();  // Print all coefficients
  void PolyConv(const Polynomial *poly2,
                Polynomial *poly_out);    // Convolute with another polynomial
  void PolySquare(Polynomial *poly_out);  // Square a polynomial
  void PolyDiff(Polynomial *poly_out);    // Get the derivative of a polynomial
  void PolyAtTime(
      const double time,
      double *result) const;  // Return polynomial value at given time
  std::vector<std::complex<double>> Roots2ndOrderPoly();
  std::vector<std::complex<double>> Roots3rdOrderPoly();
};

// 3D trajectories characterized by three polynomials
class Poly3D {
 public:
  Polynomial poly_x_;
  Polynomial poly_y_;
  Polynomial poly_z_;
  double t0_;  // Initial time for definition of polynomials
  double tf_;  // Final time for definition of polynomials
  int order_;  // Polynomial order

  // Constructor: define a polynomial from a "ControlState" msg type (2nd order
  // polynomials only)
  Poly3D(const double t0_in, const double tf_in,
         const ff_msgs::ControlState segment);
  Poly3D();

  // Methods
  void PrintSegmentCoeff();  // Print all coefficients
  void SegmentAtTime(const double time,
                     Eigen::Vector3d *result)
      const;  // Return 3d value for polynomials at a given time
  void SegmentAtTime(const double time, pcl::PointXYZ *result) const;
};

// 3D trajectories characterized by a set of 3D polynomials
class Trajectory3D {
 public:
  std::vector<Poly3D> segments_poly_;
  double t0_;  // Initial time for the first segment
  double tf_;  // Final time for the last segment
  int n_segments_;

  // Constructor
  explicit Trajectory3D(const ff_msgs::Segment segments);

  // Methods
  void PrintTrajCoeff();  // Print coefficients from all segments
  void TrajectoryAtTime(const double time,
                        Eigen::Vector3d *result)
      const;  // Return 3d value for polynomials at a given time
  void TrajectoryAtTime(const double time,
                        pcl::PointXYZ *result)
      const;  // Return 3d value for polynomials at a given time
};

}  // namespace polynomials

#endif  // MAPPER_POLYNOMIALS_H_
