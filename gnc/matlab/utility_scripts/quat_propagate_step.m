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

function quat_out = quat_propagate_step(quat_in, omega, time_in)
%#codegen

% omega row vector
%
% From: Indirect Kalman Filter for 3D Attitude Estimation: A tutorial for Quaternion Algebra
% Equation below is from Eq. 122, with Omega matrix and 
% identity matrix multiplied together
%
%

omega_mag = rssrow(omega);
quat_out = zeros(length(omega_mag),4);

for ii = 1:length(omega_mag)
    if omega_mag(ii) == 0 || time_in(ii) <= 0
        quat_out(ii,:) = quat_in(ii,:);
    else
        c = cos(.5 .* omega_mag(ii) .* time_in(ii));
        sine_mag = sin(.5 .* omega_mag(ii) .* time_in(ii));
        s = sine_mag .* omega(ii,:) ./ omega_mag(ii);
        quat_mat = [c, -s(3), s(2), -s(1); s(3), c, -s(1), -s(2); -s(2), s(1), c, -s(3); s(1), s(2), s(3), c]'; % Rollup of trig equations and Omega matrix
        quat_out(ii,:) = (quat_mat * quat_in(ii,:)')';
        quat_out(ii,:) = sign(quat_out(ii,4)) * quat_out(ii,:) / rssrow(quat_out(ii,:));
        
    end
end