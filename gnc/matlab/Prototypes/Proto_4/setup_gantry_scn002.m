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
% Configuration file for Gantry Scenario
% TO DO:
%   Need to put the localization system into docking mode once we have the
%   dock AR targets in the correct location
astrobee_init;
proto4_init;
astrobee_version = 'flight';
tunable_init;
astrobee_prep;

tun_debug_ctl_use_truth = true;
astrobee_stop_time = 300;
tun_ase_enable_of = uint8(0); % Disable optical flow

init_pos = [0 0 0];

[setpoints1, cmd_times1] = calc_trapazoidal_waypoints_with_att_end_point([0 0 0], [4 0 0], .01, .25, [0 0 0 1],[0 0 0 1], 0, 0, 60);
[setpoints2, cmd_times2] = calc_trapazoidal_waypoints_with_att_end_point([4 0 0], [4 0 0], .01, .25, [0 0 0 1],[0 0 .7071 .7071], 1*pi/180, 10*pi/180, 100);
[setpoints3, cmd_times3] = calc_trapazoidal_waypoints_with_att_end_point([4 0 0], [4 .7 0], .01, .25, [0 0 .7071 .7071],[0 0 .7071 .7071], 1*pi/180, 10*pi/180, 130);
[setpoints4, cmd_times4] = calc_trapazoidal_waypoints_with_att_end_point([4 .7 0], [4 .7 0], .01, .25, [0 0 .7071 .7071] ,[0 0 1 0], 1*pi/180, 10*pi/180, 150);
[setpoints5, cmd_times5] = calc_trapazoidal_waypoints_with_att_end_point([4 .7 0], [0 .7 0], .01, .25, [0 0 1 0],[0 0 1 0], 1*pi/180, 10*pi/180, 180);
[setpoints6, cmd_times6] = calc_trapazoidal_waypoints_with_att_end_point([0 .7 0], [0 .7 0], .01, .25, [0 0 1 0],[0 0 -.7071 .7071], 1*pi/180, 10*pi/180, 220);
[setpoints7, cmd_times7] = calc_trapazoidal_waypoints_with_att_end_point([0 .7 0], [0 0 0], .01, .25, [0 0 -.7071 .7071],[0 0 -.7071 .7071], 1*pi/180, 10*pi/180, 260);
[setpoints8, cmd_times8] = calc_trapazoidal_waypoints_with_att_end_point([0 0 0], [0 0 0], .01, .25, [0 0 -.7071 .7071], [0 0 0 1], 1*pi/180, 10*pi/180, 280);


mlp_command_times = uint32([0 0; cmd_times1; cmd_times2; cmd_times3; cmd_times4; cmd_times5; cmd_times6; ...
                            cmd_times7; cmd_times8]);
mlp_command_list = single([init_pos 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
                          setpoints1; ...
                          setpoints2; setpoints3; setpoints4; setpoints5; ...
                          setpoints6; setpoints7; setpoints8]);

ini_sim_initial_conditions_init;
tun_ini_P_B_ISS_ISS = single(init_pos);
tun_ase_state_ic_P_B_ISS_ISS = single(tun_ini_P_B_ISS_ISS);
tun_cmd_hard_accel_limit = single(.025);




%localization mode commanding
mlp_loc_mode_cmd_times      = uint32([  0 , 0; ...                          %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
                              20 , 0; ...
                              1300 , 0]);
mlp_loc_mode_cmd_times      = [mlp_loc_mode_cmd_times(:,1)+ini_time_seconds, mlp_loc_mode_cmd_times(:,2)+ini_time_nanoseconds];

% Docking mode, 1
mlp_loc_mode_cmd_list       = uint8([0; 0; 0]);                             %[] Mode Commands to be sent at specified times.

% Nominal command mode
mlp_mode_cmd_list = uint8([2;2;2]);
mlp_mode_cmd_times = uint32([  0 , 0; ...                          %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
                              60 , 0; ...
                              120 , 0]);
mlp_mode_cmd_times = [mlp_mode_cmd_times(:,1)+ini_time_seconds, mlp_mode_cmd_times(:,2)+ini_time_nanoseconds];



