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



function [body_rates, time_out] = rate_from_quat(time, quat)
% Fundamentals of Spacecraft Attitude Determinatinon and Control
% F. Landis Markley and John L. Crassidis
% Eqn 3.21

time_out = time(2:end);
body_rates = zeros(length(time)-1, 3);
for ii = 2:length(time)
    q = quat(ii,:);
    q_dot = (q-quat(ii-1,:))/(time(ii) - time(ii-1));
    body_rates(ii,:) = 2*xi(q)'*q_dot';
    
end

end

function mat_out = xi(q)
% Fundamentals of Spacecraft Attitude Determinatinon and Control
% F. Landis Markley and John L. Crassidis
% Eqn A.9b

    q4 = q(4);
    q_vec = q(1:3)';
    mat_out = [diag([q4, q4, q4])+skew(q_vec); -q_vec'];

end
