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

% Converts a quaternion to 3-2-1 Euler angles
function eulers = quat_to_eulers(q)

eulers(:,1) = atan2(2*(q(:,4).*q(:,1) + q(:,2).*q(:,3)), 1-2.*(q(:,1).^2 + q(:,2).^2));
eulers(:,2) = asin(2*(q(:,4).*q(:,2) - q(:,3).*q(:,1)));
eulers(:,3) = atan2(2*(q(:,4).*q(:,3) + q(:,1).*q(:,2)), 1-2.*(q(:,2).^2 + q(:,3).^2));

