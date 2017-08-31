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

function quat = eulers_to_quat(phi, theta, psi)
% Convert Euler angles to quaternions.  Assumes 3-2-1 rotation sequence for
% euler angles.  Corrected typo where first and 3rd rows were flipped.
% From: Fundamentals of Spacecraft Attitude Determination and Control, F.
% Landis Markley and John L. Crassisdis. Table: B.5
quat = zeros(size(psi,1), 4, 'like', psi);
 
c_psi = cos(psi/2); s_psi = sin(psi/2);
c_theta = cos(theta/2); s_theta = sin(theta/2);
c_phi = cos(phi/2); s_phi = sin(phi/2);

quat(:,1) = s_phi.*c_theta.*c_psi - c_phi.*s_theta.*s_psi;
quat(:,3) = c_phi.*c_theta.*s_psi - s_phi.*s_theta.*c_psi;
quat(:,2) = c_phi.*s_theta.*c_psi + s_phi.*c_theta.*s_psi;
quat(:,4) = c_phi.*c_theta.*c_psi + s_phi.*s_theta.*s_psi;



