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
#ifndef GRAPH_BAG_MESSAGE_BUFFER_PARAMS_H_
#define GRAPH_BAG_MESSAGE_BUFFER_PARAMS_H_

namespace graph_bag {
struct MessageBufferParams {
  double msg_delay;
  // Some message providers drop messages too close together in time. Drop all messages < min_msg_spacing from each
  // other.
  double min_msg_spacing;
};
}  // namespace graph_bag

#endif  // GRAPH_BAG_MESSAGE_BUFFER_PARAMS_H_
