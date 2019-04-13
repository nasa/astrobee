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

#ifndef PLANNER_TRAPEZOIDAL_PLANNER_TRAPEZOIDAL_H_
#define PLANNER_TRAPEZOIDAL_PLANNER_TRAPEZOIDAL_H_

// Standard includes
#include <ros/ros.h>

// FSW includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_names.h>
#include <msg_conversions/msg_conversions.h>

/**
 * \ingroup planner
 */
namespace planner_trapezoidal {
  // Insert a trapezoid between two poses
  void InsertTrapezoid(ff_util::Segment &segment, ros::Time & offset, double dt,
                       const Eigen::Affine3d & p0, const Eigen::Affine3d & p1,
                       double lin_v, double rot_v, double lin_a, double rot_a,
                       double min_control_period, double epsilon);

}
#endif  // PLANNER_TRAPEZOIDAL_PLANNER_TRAPEZOIDAL_H_
