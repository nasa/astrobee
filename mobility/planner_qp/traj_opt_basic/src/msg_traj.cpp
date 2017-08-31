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

#include <traj_opt_basic/msg_traj.h>
#include <boost/shared_ptr.hpp>
#include <vector>
namespace traj_opt {

MsgTrajectory::MsgTrajectory(const TrajData &traj) : traj_(traj) {
  num_secs_ = traj_.data.front().segments;
  dim_ = traj_.dimensions;

  for (auto &poly : traj_.data.front().segs) dts.push_back(poly.dt);
  exec_t = traj.data.front().t_total;

  deg_ = traj_.data.front().segs.front().degree;

  polyies_.reserve(num_secs_);
  // unpack coefficents
  //  auto it = traj.coefficents.begin();
  for (uint i = 0; i < num_secs_; i++) {
    std::vector<boost::shared_ptr<Poly>> polys;
    polys.clear();
    polys.reserve(dim_);
    for (int j = 0; j < dim_; j++) {
      polys.push_back(boost::make_shared<Poly>(
          traj.data.at(j).segs.at(i).coeffs.data(), deg_));
      //      std::vector<decimal_t> vals;
      //      vals.clear();
      //      vals.reserve(traj_.degree + 1);
      //      for (int k = 0; k <= traj_.degree; k++) {
      //        vals.push_back(*it);
      //        it++;
      //      }
      //      polys.push_back(std::make_shared<LPoly>(vals.data(),
      //      traj_.degree));
      //      //      ROS_ERROR_STREAM("Poly " << *(polys.back()));
    }
    polyies_.push_back(polys);
  }
  // take derrivatives
  derrives_.push_back(polyies_);
  for (int i = 0; i < 3; i++) {
    for (auto &p : polyies_)
      for (auto &q : p)
        q = boost::make_shared<Poly>(PolyCalculus::differentiate(*q));

    derrives_.push_back(polyies_);
  }
}

bool MsgTrajectory::evaluate(decimal_t t, uint derr,
                             VecD &out) const {  // returns false when out
  out = VecD::Zero(dim_, 1);
  //  out << 0.0,0.0,0.0,0.0;
  bool success = false;

  decimal_t dt, dx;

  // of time range, but still
  // sets out to endpoint
  std::vector<boost::shared_ptr<Poly>> const *poly;
  if (t < 0) {
    poly = &derrives_.at(derr).front();
    dt = dts.front();
    dx = 0.0;
    success = false;
  } else {
    // find appropriate section
    auto dt_it = dts.begin();
    for (auto &it : derrives_.at(derr)) {
      if (t < *dt_it) {
        poly = &(it);
        dt = *dt_it;
        dx = t / (*dt_it);
        success = true;
        break;
      }
      t -= *dt_it;
      ++dt_it;
    }
    if (!success) {
      poly = &derrives_.at(derr).back();
      dt = dts.back();
      dx = 1.0;
    }
    success = false;
  }
  decimal_t ratio = std::pow(1 / dt, decimal_t(derr));
  for (int i = 0; i < dim_; i++) out(i) = ratio * poly->at(i)->evaluate(dx);

  //  if(t < 0.1)
  //  if(derr == 0)
  //      ROS_INFO_STREAM("result " << out.transpose());
  //  ROS_INFO_STREAM("result " << out.transpose());
  return success;
}
decimal_t MsgTrajectory::getTotalTime() const {
  decimal_t tt = 0.0;
  for (auto &t : dts) tt += t;
  return tt;
}

TrajData MsgTrajectory::serialize() { return traj_; }
decimal_t MsgTrajectory::getCost() { return NAN; }
}  // namespace traj_opt
