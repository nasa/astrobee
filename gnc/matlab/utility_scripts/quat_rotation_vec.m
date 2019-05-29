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


%%
% Q = q_a2b, rotation represents a vector rotating from frame b2a
% q_out = quat_rotation(vector, Q2)
%
% rotates a row-vector of 3-vectors by a row-vector of quaternion Q2
% 
% We accomplish this rotation by using DCMs as an intermediate step
%
function vec_out = quat_rotation_vec(vector, Q)

if size(vector, 1) == 1 % Check to see if the vector is a single row
    vector = repmat(vector, size(Q,1), 1);
end
    


[nquat, ~] = size(Q);
vec_out = zeros(nquat, 3, 'like', vector);

for ii = 1:nquat
    vec_out(ii, :) = quaternion_to_dcm(Q(ii, :))*vector(ii,:)';
end





