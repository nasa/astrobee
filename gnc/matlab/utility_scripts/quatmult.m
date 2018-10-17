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

% Quaternion Multiplication:
% Uses Hamilton's convention where the rotation order is left to right,
% q1*q2 corresponds to the first rotation q1, followed by the second
% rotation q2.
%
% Fundamentals of Spacecraft Attitude Determination and Control,
% F. Landis Markley and John L. Crassidis
% Equation: 2.82b
function qOut = quatmult(p, q)

qOut = [q(:,4) .* p(:,1:3) + p(:,4).*q(:,1:3) + cross(p(:,1:3), q(:,1:3), 2), ...
    p(:,4).*q(:,4) - dot(p(:,1:3), q(:,1:3),2)];

qOut = qOut ./ repmat(rssrow(qOut), 1, 4); % Normalize

end
