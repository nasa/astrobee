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

#include <algorithm>
#include <vector>
#include "mapper/polynomials.h"

namespace polynomials {

// constructor method
Polynomial::Polynomial(const double t0,
                       const double tf,
                       const int coeff_size) {
    t0_ = t0;
    tf_ = tf;
    order_ = coeff_size - 1;
    coeff_ = Eigen::MatrixXd::Zero(coeff_size, 1);
}

Polynomial::Polynomial() {
    t0_ = 0.0;
    tf_ = 1.0;
    order_ = 2;
    coeff_ = Eigen::MatrixXd::Zero(1, 1);
}

// print all coefficients
void Polynomial::PrintPolyCoeff() {
    int n_poly_coeff = order_ + 1;
    // std::cout << "vector length: " << n_polyCoeff << std::endl;
    // std::cout << "t0: " << poly.t0 << "\ttf: " << poly.tf << std::endl;
    for (int i = 0; i < n_poly_coeff; i++) {
        std::cout << coeff_(i) << " ";
    }
    std::cout << std::endl;
}

// convolute with another polynomial
void Polynomial::PolyConv(const Polynomial *poly2,
                          Polynomial *poly_out) {
    static int m, n, n_coeff_out, j_min, j_max;
    m = order_ + 1;
    n = poly2->order_ + 1;
    n_coeff_out = m + n - 1;

    for (int k = 0; k < n_coeff_out; k++) {
        j_min = std::max(0, k-n+1);
        j_max = std::min(k, m-1);
        for (int j = j_min; j <= j_max; j++) {
            poly_out->coeff_(k) = poly_out->coeff_(k) + coeff_(j)*poly2->coeff_(k-j);
        }
    }
}

// square a polynomial
void Polynomial::PolySquare(Polynomial *poly_out) {
    if (order_ == 2) {  // closed form solution
        static Eigen::VectorXd coeff_out(5);
        static double a, b, c;
        // Polynomial poly_copy = &this;
        // poly_copy = this;
        a = coeff_(0);
        b = coeff_(1);
        c = coeff_(2);

        coeff_out << a*a, 2*a*b, 2*a*c+b*b, 2*b*c, c*c;
        poly_out->coeff_ = coeff_out;
    } else {  // convolute the polynomial with itself
        this->PolyConv(this, poly_out);
    }
    // polyOut->printPolyCoeff(); std::cout << std::endl;
}

// set the derivative of a polynomial
void Polynomial::PolyDiff(Polynomial *poly_out) {
    static int power;
    for (int i = 0; i < order_; i++) {
        power = order_ - i;
        poly_out->coeff_(i) = coeff_(i)*power;
    }
}

// return polynomial value at given time
void Polynomial::PolyAtTime(const double time,
                            double *result) const {
    static int m, power;
    static double t;
    m = order_ + 1;
    *result = 0.0;

    t = time - t0_;
    if (t != 0) {
        for (int i = 0; i < m; i++) {
            power = m - i - 1;
            *result = *result + coeff_(i)*pow(t, power);
        }
    } else {
        *result = coeff_(m-1);
    }
    // return result;
}

std::vector<std::complex<double>> Polynomial::Roots2ndOrderPoly() {
    // set the coefficients in friendly form
    double a = coeff_(0);
    double b = coeff_(1);
    double c = coeff_(2);

    // calculate the quadratic formula for all cases
    double delta = b*b - 4*a*c;
    std::vector<std::complex<double>> solution;
    std::complex<double> sol1, sol2;
    if (delta > 0) {  // there are two distinct real roots
        sol1 = std::complex<double>((-b+sqrt(delta))/(2*a) , 0.0);
        sol2 = std::complex<double>((-b-sqrt(delta))/(2*a) , 0.0);
    } else if (delta == 0) {  // there are two real multiple roots
        sol1 = std::complex<double>(-b/(2*a) , 0.0);
        sol2 = std::complex<double>(-b/(2*a) , 0.0);
    } else {  // there are two complex roots
        sol1 = std::complex<double>(-b/(2*a) ,  sqrt(-delta)/(2*a));
        sol2 = std::complex<double>(-b/(2*a) , -sqrt(-delta)/(2*a));
    }
    solution.push_back(sol1);
    solution.push_back(sol2);

    // std::cout << solution[0] << std::endl <<
    //              solution[1] << std::endl;

    return solution;
}

// find the roots of a 3rd order polynomial
// algebraic solution from https://en.wikipedia.org/wiki/Casus_irreducibilis
std::vector<std::complex<double>> Polynomial::Roots3rdOrderPoly() {
    // set the coefficients in friendly form
    static double a, b, c, d;
    a = coeff_(0);
    b = coeff_(1);
    c = coeff_(2);
    d = coeff_(3);

    // set Cardano's coefficients
    static double p, q;
    p = (3*a*c - b*b)/(3*a*a);
    q = (2*b*b*b - 9*a*b*c + 27*a*a*d)/(27*a*a*a);

    // cube roots of 1
    std::vector<std::complex<double>> w;
    w.push_back(std::complex<double>(1.0, 0.0));
    w.push_back(std::complex<double>(-0.5, 0.5*sqrt(3)));
    w.push_back(std::complex<double>(-0.5, -0.5*sqrt(3)));

    // implementation of the solution
    static double delta, term, term1, term2;
    std::vector<std::complex<double>> solution;
    delta = q*q/4.0 + p*p*p/27.0;
    term = b/(3.0*a);
    if (delta >= 0.0) {  // casus irreducibilis
        term1 = -q/2.0 + sqrt(delta);
        term2 = -q/2.0 - sqrt(delta);

        // calculate the three solutions
        for (int i = 0; i < 3; i++) {
            std::complex<double> t = w[i]*cbrt(term1) + w[i]*w[i]*cbrt(term2);
            solution.push_back(t - term);
        }
    } else {  // trigonometric solution
        term1 = acos(3.0*q*sqrt(-3.0/p)/(2.0*p));
        for (int i = 0; i < 3; i++) {
            std::complex<double> t = 2*sqrt(-p/3.0)*cos(term1/3.0 - static_cast<double>(i)*2*M_PI/3.0);
            solution.push_back(t - term);
            // std::cout << t.real() << std::endl;
        }
    }

    return solution;
}


// constructor: define a polynomial from a "ControlState" msg type (2nd order polynomials only)
Poly3D::Poly3D(const double t0_in,
               const double tf_in,
               const ff_msgs::ControlState segment) {
    int n_coeff = 3;
    poly_x_.coeff_.resize(n_coeff);
    poly_y_.coeff_.resize(n_coeff);
    poly_z_.coeff_.resize(n_coeff);
    poly_x_ = Polynomial(t0_in, tf_in, n_coeff);
    poly_y_ = Polynomial(t0_in, tf_in, n_coeff);
    poly_z_ = Polynomial(t0_in, tf_in, n_coeff);
    t0_ = t0_in;
    tf_ = tf_in;
    order_ = n_coeff - 1;

    // set the polynomial coefficients
    Eigen::VectorXd coeff_x(n_coeff), coeff_y(n_coeff), coeff_z(n_coeff);
    coeff_x << 0.5*segment.accel.linear.x,
               segment.twist.linear.x,
               segment.pose.position.x;
    coeff_y << 0.5*segment.accel.linear.y,
               segment.twist.linear.y,
               segment.pose.position.y;
    coeff_z << 0.5*segment.accel.linear.z,
               segment.twist.linear.z,
               segment.pose.position.z;
    poly_x_.coeff_ = coeff_x;
    poly_y_.coeff_ = coeff_y;
    poly_z_.coeff_ = coeff_z;
}

Poly3D::Poly3D() {
    int n_coeff = 1;
    double t0_in = 0.0;
    double tf_in = 1.0;
    poly_x_.coeff_.resize(n_coeff);
    poly_y_.coeff_.resize(n_coeff);
    poly_z_.coeff_.resize(n_coeff);
    poly_x_ = Polynomial(t0_in, tf_in, n_coeff);
    poly_y_ = Polynomial(t0_in, tf_in, n_coeff);
    poly_z_ = Polynomial(t0_in, tf_in, n_coeff);
    t0_ = t0_in;
    tf_ = tf_in;
}

void Poly3D::PrintSegmentCoeff() {
        std::cout << "t0: " << t0_ << "\ttf: " << tf_ <<  std::endl;
        std::cout << "polyX: "; poly_x_.PrintPolyCoeff();
        std::cout << "polyY: "; poly_y_.PrintPolyCoeff();
        std::cout << "polyZ: "; poly_z_.PrintPolyCoeff();
}

void Poly3D::SegmentAtTime(const double time,
                            Eigen::Vector3d *result) const {
    double x, y, z;
    poly_x_.PolyAtTime(time, &x);
    poly_y_.PolyAtTime(time, &y);
    poly_z_.PolyAtTime(time, &z);
    *result = Eigen::Vector3d(x, y, z);
}

void Poly3D::SegmentAtTime(const double time,
                            pcl::PointXYZ *result) const {
    double x, y, z;
    poly_x_.PolyAtTime(time, &x);
    poly_y_.PolyAtTime(time, &y);
    poly_z_.PolyAtTime(time, &z);
    result->x = x;
    result->y = y;
    result->z = z;
}

Trajectory3D::Trajectory3D(const ff_msgs::Segment segments) {
    n_segments_ = segments.segment.size() - 1;

    if (n_segments_ > 0) {
        ros::Time t0_segment;
        ros::Time tf_segment;

        t0_ = segments.segment[0].when.toSec();
        tf_ = segments.segment[n_segments_].when.toSec();

        for (int i = 0; i < n_segments_; i++) {
            t0_segment = segments.segment[i].when;
            tf_segment = segments.segment[i+1].when;

            Poly3D newSegment(t0_segment.toSec(),
                              tf_segment.toSec(),
                              segments.segment[i]);
            segments_poly_.push_back(newSegment);
        }
    } else {
        t0_ = 0;
        tf_ = 0;
    }
}

void Trajectory3D::PrintTrajCoeff() {
    std::cout << "n_segments: " << n_segments_ << std::endl;
    std::cout << "t0: "   << t0_
              << "\ttf: " << tf_ << std::endl;
    for (int i = 0; i < n_segments_; i++) {
        std::cout << "Segment: " << i+1 << std::endl;
        segments_poly_[i].PrintSegmentCoeff();
    }
}

void Trajectory3D::TrajectoryAtTime(const double time,
                                     Eigen::Vector3d *result) const {
    // find which segment the given "time" belongs to
    for (int i = 0; i < n_segments_; i++) {
        if (time <= segments_poly_[i].tf_) {
            segments_poly_[i].SegmentAtTime(time, result);
            return;
        }
    }

    // if the code reaches this point, it means that we have to
    // return the value of the polynomial some time after tf (extrapolation)
    segments_poly_[n_segments_-1].SegmentAtTime(time, result);
    ROS_WARN("Time extrapolation when returning polynomial!");
    return;
}

void Trajectory3D::TrajectoryAtTime(const double time,
                                    pcl::PointXYZ *result) const {
    // find which segment the given "time" belongs to
    for (int i = 0; i < n_segments_; i++) {
        if (time <= segments_poly_[i].tf_) {
            segments_poly_[i].SegmentAtTime(time, result);
            return;
        }
    }

    // if the code reaches this point, it means that we have to
    // return the value of the polynomial some time after tf (extrapolation)
    segments_poly_[n_segments_-1].SegmentAtTime(time, result);
    ROS_WARN("Time extrapolation when returning polynomial!");
    return;
}

}  // namespace polynomials
