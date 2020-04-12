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



% Using axis angle as an intermediate step to do quaternion interpretation
% Loosely based on https://en.wikipedia.org/wiki/Slerp, but utilizing
% axis-angle to do linear interpolation of the angle.  May yield different
% results than the common SLERP equations

function q_out = interp_slerp(q_in, time_in, time_out)

if isa(q_in, 'quaternion') % Check if the input quaternion is a quaternion class, we don't use this functionality presently
    q_in = q_in.value;
    use_quat_class = true;
else
    use_quat_class = false;
end

if ~all(diff(time_in) > 0)
    error('Time is not monotonically increasing in sample')
end
    
n = length(time_out);

q_out = zeros(n,4);
for ii=1:n
    if time_out(ii) > max(time_in) || time_out(ii) < min(time_in)
        q_out(ii,:) = [NaN NaN NaN NaN];
        continue;
    end
    
    if any(time_out(ii) == time_in)
        match_time_indx = find(time_out(ii) == time_in, 1, 'first');
        q_out(ii,:) = q_in(match_time_indx,:);
    else
        % Consider adding min angle check, below which we just linearly
        % interpolate
        prev_time_indx = find(time_out(ii) > time_in, 1, 'last');
        q_prev = q_in(prev_time_indx, :);
        q_next = q_in(prev_time_indx+1, :);
        deltaT = (time_out(ii)-time_in(prev_time_indx))/(time_in(prev_time_indx+1)-time_in(prev_time_indx));
        
        delta_q = quatmult(quat_inv(q_prev), q_next); % Calculate a quaternion that rotates from the previous quat to the next quat
        % Add singularity protections here
        
        [axis, angle] = quat2axisAngle(delta_q); % Represent this delta quaternion as an axis/angle
        
        norm_angle = angle*deltaT; % Find what portion of the angle should used this step
        
        delta_q =  axisAngle2quat(axis, norm_angle); % Calculate the corresponding quaternion
    
         q_out(ii,:) = quatmult(q_prev, delta_q); % Rotate q_prev by the new delta quaternion to get the interpolated quat
         q_out(ii,:) = q_out(ii,:) ./ rssrow(q_out(ii,:)); % Normalize
       
    end
    
    
        
end

if use_quat_class
    q_out = quaternion(q_out);
end



