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

% Convert quaterion to a DCM.  DCM will rotate a vector V by Q_A2B from
% reference frame A to reference frame B.
%
% Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for Aquaternion Algebra.
% Nikolas Trawny and Stergios I. Roumeliotis
% Equation 78
function dcm = quaternion_to_dcm(q_in)

q4 = q_in(4);

dcm =  (2*q4^2 - 1)*eye(3, 'like', q_in) - 2*q4*skew(q_in) + 2*(q_in(1:3)'*q_in(1:3));
