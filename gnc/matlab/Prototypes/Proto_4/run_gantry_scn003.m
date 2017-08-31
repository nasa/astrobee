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
%
%% Run the baseline astrobee scenario
% Gantry scenario 001, straight line forward and backward.
%
%
%% Configure Simulation
% Stop time
% Set clock
open_sys = find_system;
if isempty(strfind(open_sys, 'astrobee'))
    open('astrobee');
end

% Configure the scenario
% Init files
astrobee_init;
proto4_init;
setup_gantry_scn003;



% Dependencies 
astrobee_prep;

%% Simulate
display('Running Simulation');
tic
sim('astrobee')
toc
display('Simulation Complete');
%% Post Process Data
dlmwrite('gantry_scn003.csv', [out_ctl_msg.body_accel_cmd.time, out_ctl_msg.body_accel_cmd.data, out_ctl_msg.body_alpha_cmd.data])
save gantry_scn003_truth out_env_msg

display('Converting Simulation Data Stores to telemData');
astrobee_load_sim_data;
simData = 1;
display('Performing post processing calculations');
astrobee_post_process;


