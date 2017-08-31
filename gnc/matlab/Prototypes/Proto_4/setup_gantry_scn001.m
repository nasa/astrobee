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
astrobee_stop_time = 180;
tun_ase_enable_of = uint8(0); % Disable optical flow

init_pos = [0 .7 0];

[setpoint_out, cmd_times_out] = calc_trapazoidal_waypoints_with_att_end_point([0 .7 0], [4 .7 0], .01, .25, [0 0 0 1],[0 0 0 1], 0, 0, 60);
[setpoint_back, cmd_times_back] = calc_trapazoidal_waypoints_with_att_end_point([4 .7 0], [0 .7 0], .01, .25, [0 0 0 1],[0 0 0 1], 0, 0, 120);
%[mlp_command_list, mlp_command_times] = gen_cmd_list([ASTROBEE_ROOT filesep 'scenarios' filesep 'scn05.fplan']);    
%[pos_x,     pos_y,     pos_z,     vel_x,     vel_y,    vel_z,     acc_x,     acc_y,     acc_z,   quat_x,   quat_y,   quat_z,    quat_w,   angv_x,   angv_y,   angv_z,    anga_x,    anga_y,    anga_z]
mlp_command_times = uint32([0 0; cmd_times_out; cmd_times_back]);
mlp_command_list = single([init_pos 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
                          setpoint_out; ...
                          setpoint_back]);

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



