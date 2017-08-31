% Copyright (c) 2017, United States Government, as represented by the
% Administrator of the National Aeronautics and Space Administration.
%
% All rights reserved.
%
% The Astrobee platform is licensed under the Apache License, Version 2.0
% (the "License"); you may not use this file except in compliance with the
% License. You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
% WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
% License for the specific language governing permissions and limitations
% under the License.

% Apply delta state to the current state

function [state] = apply_delta_to_state(state, delta)
%#codegen
state(1:4) = update_quaternion(state(1:4), delta(1:3));
  state(5:16) = state(5:16) + delta(4:15)';
  num_cameras = (size(state, 2) - 16) / 7;
  for i=1:num_cameras
    os = 17 + (i - 1) * 7;
    od = 16 + (i - 1) * 6;
    state(os:os+3) = update_quaternion(state(os:os+3), delta(od:od+2));
    state(os+4:os+6) = state(os+4:os+6) + delta(od+3:od+5)';
  end
end
