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

#ifndef NODE_UPDATERS_NODE_UPDATE_MODEL_H_
#define NODE_UPDATERS_NODE_UPDATE_MODEL_H_

#include <node_updaters/node_update_model_params.h>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace node_updaters {
template <typename NodesType>
class NodeUpdateModel {
 public:
  virtual ~NodeUpdateModel() = 0;
  // TODO(rsoussan) return key vector!
  virtual boost::optional<gtsam::Key> AddNode(const localization_common::Time timestamp, NodesType& nodes) = 0;
  // TODO(rsoussan) pass key vectors for key a and key b???
  virtual bool AddRelativeFactor(const gtsam::Key key_a, const localization_common::Time timestamp_a,
                                 const gtsam::Key key_b, const localization_common::Time timestamp_b,
                                 gtsam::NonlinearFactorGraph& factors) const = 0;

 protected:
  // TODO(rsoussan): Add constructor to set these, template on params?
  NodeUpdateModelParams params_;
};
}  // namespace node_updaters

#endif  // NODE_UPDATERS_NODE_UPDATE_MODEL_H_
