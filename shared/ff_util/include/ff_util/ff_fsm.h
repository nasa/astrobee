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

#ifndef FF_UTIL_FF_FSM_H_
#define FF_UTIL_FF_FSM_H_

// C++11 includes
#include <functional>
#include <utility>
#include <vector>
#include <map>

namespace ff_util {

template<typename State, typename Event>
class FiniteStateMachine {
 public:
  typedef std::function<State(Event const&)> TransitionCallback;
  typedef std::function<void(State const&, Event const&)> UpdateCallback;

  // Initialize the FSM with a given initial state
  FiniteStateMachine(State const& initial_state,
    UpdateCallback callback = nullptr) :
      state_(initial_state), callback_(callback) {}

  // Class templates and variadic templated arguments gets a little
  // hairy, so I'm going to hard-code up to three possible events that
  // trigger the same transition.
  void Add(Event const& e1, Event const& e2, Event const& e3, Event const& e4,
    State const& prev, TransitionCallback callback) {
    fsm_[std::make_pair(prev, e1)] = callback;
    fsm_[std::make_pair(prev, e2)] = callback;
    fsm_[std::make_pair(prev, e3)] = callback;
    fsm_[std::make_pair(prev, e4)] = callback;
  }
  void Add(Event const& e1, Event const& e2, Event const& e3,
    State const& prev, TransitionCallback callback) {
    fsm_[std::make_pair(prev, e1)] = callback;
    fsm_[std::make_pair(prev, e2)] = callback;
    fsm_[std::make_pair(prev, e3)] = callback;
  }
  void Add(Event const& e1, Event const& e2,
    State const& prev, TransitionCallback callback) {
    fsm_[std::make_pair(prev, e1)] = callback;
    fsm_[std::make_pair(prev, e2)] = callback;
  }
  void Add(Event const& e1,
    State const& prev, TransitionCallback callback) {
    fsm_[std::make_pair(prev, e1)] = callback;
  }

  // Get the current state
  State GetState() {
    return state_;
  }

  // Set the current state
  void SetState(State const& state) {
    state_ = state;
  }

  // Update the state machine
  void Update(Event const& event) {
    std::pair<State, Event> key = std::make_pair(state_, event);
    if (fsm_.find(key) != fsm_.end()) {
      state_ = fsm_[key](event);    // Call the transition function
      if (callback_)                // Post-update callback
        callback_(state_, event);
    }
  }

 private:
  State state_;
  std::map<std::pair<State, Event>, TransitionCallback> fsm_;
  UpdateCallback callback_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_FSM_H_
