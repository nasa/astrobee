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

% construct rotation matrix from quaternion
% From Zack and Brian's ekf.m  Used as a nested function in the optical
% flow update

function [R] = quaternion_to_rotation(q)
%#codegen
  S = skew(q(1:3));
  a = [q(4) * eye(3, 3) - S; -q(1:3)'];
  b = [q(4) * eye(3, 3) + S; -q(1:3)'];
  R = b' * a;
end