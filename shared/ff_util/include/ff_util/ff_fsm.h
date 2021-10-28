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

// C++ includes
#include <functional>
#include <utility>
#include <vector>
#include <map>

namespace ff_util {

class FSM {
 public:
  typedef   int8_t State;
  typedef uint32_t Event;
  typedef std::function<State(Event const&)> TransitionCallback;
  typedef std::function<State(State const&, Event const&)> CatchallCallback;
  typedef std::function<void(State const&, Event const&)> UpdateCallback;

  // Initialize the FSM with a given initial state
  FSM(State const& initial_state,
    UpdateCallback callback = nullptr) :
      state_(initial_state), callback_(callback) {}

  // 1-state transition
  void Add(State const& s1,
           Event const& mask,
           TransitionCallback callback) {
    for (size_t i = 0; i < 8 * sizeof(Event); i++) {
      if (mask & (1 << i)) {
        fsm_[std::make_pair(s1, 1 << i)] = callback;
      }
    }
  }

  // 2-state transition
  void Add(State const& s1, State const& s2,
           Event const& mask,
           TransitionCallback callback) {
    for (size_t i = 0; i < 8 * sizeof(Event); i++) {
      if (mask & (1 << i)) {
        fsm_[std::make_pair(s1, 1 << i)] = callback;
        fsm_[std::make_pair(s2, 1 << i)] = callback;
      }
    }
  }

  // 3-state transition
  void Add(State const& s1, State const& s2, State const& s3,
           Event const& mask,
           TransitionCallback callback) {
    for (size_t i = 0; i < 8 * sizeof(Event); i++) {
      if (mask & (1 << i)) {
        fsm_[std::make_pair(s1, 1 << i)] = callback;
        fsm_[std::make_pair(s2, 1 << i)] = callback;
        fsm_[std::make_pair(s3, 1 << i)] = callback;
      }
    }
  }

  // Catch-all for a single event. Takes priority.
  void Add(Event const& mask, CatchallCallback callback) {
    for (size_t i = 0; i < 8 * sizeof(Event); i++)
      if (mask & (1 << i)) catchall_[1 << i] = callback;
  }

  // Get the current state
  State GetState() {
    return state_;
  }

  // Set the current state
  void SetState(State const& state) {
    state_ = state;
  }

  // Update the state machine - we only expect one event here
  void Update(Event const& event) {
    // Case 1 : A catch-all event occured
    if (catchall_.find(event) != catchall_.end()) {
      state_ = catchall_[event](state_, event);
      if (callback_)
        callback_(state_, event);
      return;
    }
    // Case 2: Valid transition in the state machine
    std::pair<State, Event> key = std::make_pair(state_, event);
    if (fsm_.find(key) != fsm_.end()) {
      state_ = fsm_[key](event);    // Call the transition function
      if (callback_)                // Post-update callback
        callback_(state_, event);
    }
    // Case 3: Quietly ignore
  }

 private:
  State state_;
  std::map<std::pair<State, Event>, TransitionCallback> fsm_;
  std::map<Event, CatchallCallback> catchall_;
  UpdateCallback callback_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_FSM_H_
