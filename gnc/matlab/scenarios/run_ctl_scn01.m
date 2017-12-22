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
% Perching scenario
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
astrobee_init;
proto4_init;
setup_scn01;

mlp_speed_gain_cmd_list = uint8([3 3 3]');
abp_nozzle_max_open_angle     = single(79.91)*units_deg_2_rad;
abp_servo_max_PWM               = single(100); 

astrobee_prep;


%% Simulate
display('Running Simulation');
tic
sim('astrobee_control_sim')
toc
display('Simulation Complete');
%% Post Process Data
display('Converting Simulation Data Stores to telemData');
astrobee_load_sim_data;
simData = 1;
display('Performing post processing calculations');
astrobee_post_process;

trans_times = [60 75];
slew_times = [100 125];
scn1_error_vec = [max(calcData.error.att.total_mag.timeRange(trans_times(1), trans_times(2)))*180/pi, calcData.error.pos.total.mag.timeRange(trans_times(1), trans_times(2)).maxerror, max(calcData.error.att.total_mag.timeRange(slew_times(1), slew_times(2)))*180/pi, calcData.error.pos.total.mag.timeRange(slew_times(1), slew_times(2)).maxerror];
fprintf(1, 'Max att error (translation), deg: %f\n',max(calcData.error.att.total_mag.timeRange(trans_times(1), trans_times(2)))*180/pi);
fprintf(1, 'Max pos error (translation), m: %f\n',calcData.error.pos.total.mag.timeRange(trans_times(1), trans_times(2)).maxerror);
fprintf(1, 'Max att error (translation), deg: %f\n',max(calcData.error.att.total_mag.timeRange(slew_times(1), slew_times(2)))*180/pi);
fprintf(1, 'Max pos error (translation), m: %f\n',calcData.error.pos.total.mag.timeRange(slew_times(1), slew_times(2)).maxerror);
