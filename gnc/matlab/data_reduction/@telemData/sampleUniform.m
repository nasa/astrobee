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

function [A_Out, B_Out] = sampleUniform(A, B)
% resamples the telemData variables A and B according the following rules:
% 1. If one has its masterTime property set, the output is resampled to its
% time base for the period of time that overlaps both variables
% 2. If bother have masterTime or neither has masterTime set then the least
% numerous dataset becomes the masterTime
% 2a. In the event they both have equal number of points the first input
% becomes the masterTime
minTime = max(min(A.time),min(B.time));
maxTime = min(max(A.time),max(B.time));
A_indx = find(A.time >= minTime & A.time <= maxTime);
B_indx = find(B.time >= minTime & B.time <= maxTime);

if A.masterTime && ~B.masterTime
    % Use A as masterTime
    master = 'A'; 
elseif ~A.masterTime && B.masterTime
    % Use B as masterTime
    master = 'B';
elseif length(B_indx) > length(A_indx)
    master = 'B';
    % Use the most numerous one of the overlapping period as the masterTime
else % if its a tie then use A
    master = 'A';    
end

% Generate the master time vector
masterT = eval([master '.time(' master '_indx);']);




%% Generate the interpolated data
if strcmp(master, 'A')
    slave = 'B';
else
    slave = 'A';
end

% Just downsample the master to the overlapping points and output
% First we copy the the telemData structure over then replace the data,
% this will preserve all the other properties
eval([master '_Out = ' master ';']);

%A_Out = telemData(masterT, A.data(A_indx));
eval([master '_Out.time = masterT;']);
try
    eval([master '_Out.data = ' master '.data(' master '_indx,:);']);
catch
    eval([master '_Out.data = ' master '.data(' master '_indx);']);
end
%% Interpolate the slave
slaveTM = eval(slave);
    % Add special check for quaternion
    if isa(eval( [slave '.data'] ), 'quaternion') % if its a quaternion,
            interp_type = 'slerp';  % use SLERP
    elseif all(all(mod( slaveTM.data ,1)==0))
        interp_type = 'nearest';  % Use nearest if integer values
    else %if non-integer values
        interp_type = 'linear';   % Otherwise, use linear
    end
    if strcmp(interp_type, 'slerp')
        try 
            interpedData = interp_slerp(slaveTM.data, slaveTM.time, masterT);
        catch
            interp_type = 'nearest';
            interpedData = interp1(slaveTM.time, slaveTM.data.value, masterT, interp_type);
            interpedData = quaternion(interpedData);
            warning('Using nearest interpolation for quaternions, define interp_slerp on the matlab path');
        end
    else
        interpedData = interp1(slaveTM.time, slaveTM.data, masterT, interp_type);
    end
    
    % First we copy the the telemData structure over then replace the data,
    % this will preserve all the other properties
    
    
    %eval([slave '_Out =  telemData(masterT, interpedData);']);
    eval([slave '_Out = ' slave ';']);

    %A_Out = telemData(masterT, A.data(A_indx));
    eval([slave '_Out.time = masterT;']);
    eval([slave '_Out.data = interpedData;']);
    

end
