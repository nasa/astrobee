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

function [r, H, R_mat] = residual_and_h_compress(r, H, r_vec)
%#codegen
  [q1, T_H] = qr(H);

  N = min(size(H));
  %N = rank(H);

  q1 = q1(:, 1:N);
  H = T_H(1:N, :);
  r = q1' * r;
  R_mat = q1' * diag(r_vec) * q1;
end
