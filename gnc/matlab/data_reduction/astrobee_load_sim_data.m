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

% Currently assumes the Astrobee sim has been run and the sim data is
% sitting in the workspace
%
%%
wsVars = whos; % Find all the variables in the workspace
structVars = strcmp('struct', {wsVars(:).class});
structIndx = find(structVars);

% Walk through all the structures in the workspace, looking for structures
% of time series objects
for ii = 1:length(structIndx)
    testStruct = eval([wsVars(structIndx(ii)).name]);
    % Simulink outputs are structures of timeseries and length 1
    if length(testStruct) ~= 1
        continue;
    end
            
    % Test if the elements of the structure are timeseries
    ts_test = structfun(@(x) (isa(x, 'timeseries')), testStruct, 'UniformOutput', true);
    if all(ts_test)
        % Replace the Time Series structure with the the telemData
        % structure
        assignin('base', [wsVars(structIndx(ii)).name], structfun(@(x) (telemData(x)), testStruct, 'UniformOutput', false));
    end
end

%% Hack because it is currently non-recursive
if exist('out_cmc_msg', 'var')
    out_cmc_msg.cmc_mode_cmd = telemData(out_cmc_msg.cmc_mode_cmd);
    out_cmc_msg.cmc_state_cmd_a = structfun(@(x) (telemData(x)), out_cmc_msg.cmc_state_cmd_a, 'UniformOutput', false);
    out_cmc_msg.cmc_state_cmd_b = structfun(@(x) (telemData(x)), out_cmc_msg.cmc_state_cmd_b, 'UniformOutput', false);
end

clear wsVars structVars structIndx testStruct

