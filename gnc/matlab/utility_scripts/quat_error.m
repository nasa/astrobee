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

% quat_error                                                             %
%                                                                        %
% Inputs: two quaternions                                                %
%                                                                        %
% Outpus: angular error, in millirads       
% Source: https://math.stackexchange.com/questions/90081/quaternion-distance
function total_angle_millirad = quat_error( q1, q2 )

q1 = quat_norm_and_sign_check(q1);
q2 = quat_norm_and_sign_check(q2);

total_angle_millirad = acos((2*dot(q1, q2, 2).^2) - 1)*1000;

% Wrap to pi
% Should be bounded by the stackexchange equation
total_angle_millirad(total_angle_millirad > pi*1000) = total_angle_millirad(total_angle_millirad > pi*1000) - 2000*pi;
