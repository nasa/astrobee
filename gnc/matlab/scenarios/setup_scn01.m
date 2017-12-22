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
%[mlp_command_list, mlp_command_times] = gen_cmd_list([ASTROBEE_ROOT filesep 'scenarios' filesep 'scn05.fplan']);
%[pos_x,     pos_y,     pos_z,     vel_x,     vel_y,    vel_z,     acc_x,     acc_y,     acc_z,   quat_x,   quat_y,   quat_z,    quat_w,   angv_x,   angv_y,   angv_z,    anga_x,    anga_y,    anga_z]
lin_accel = .05;
[setpoints1, cmd_times1] = calc_trapazoidal_waypoints_with_att_end_point([0 0 -.7], [1 0 -.7], lin_accel, .2, [0 0 0 1],[0 0 0 1], 0, 0, 60);
[setpoints2, cmd_times2] = calc_trapazoidal_waypoints_with_att_end_point([1 0 -.7], [1 0 -.7], lin_accel, .2, [0 0 0 1],[0 0 1 0], 10*pi/180, 10*pi/180, 100);
[setpoints3, cmd_times3] = calc_trapazoidal_waypoints_with_att_end_point([0 0 -.7], [1.0 0 -.7], lin_accel, .2, [0 0 1 0],[0 0 1 0], 10*pi/180, 10*pi/180, 130);

ini_sim_initial_conditions_init


% mlp_command_times = uint32([0 0; cmd_times1; cmd_times2; cmd_times3]);
% mlp_command_list = single([ini_P_B_ISS_ISS 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
%     setpoints1; ...
%     setpoints2; setpoints3]);

init_pos = [0.0 0 -.7];

mlp_command_times = uint32([0 0; cmd_times1; cmd_times2]);
mlp_command_list = single([init_pos 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
    setpoints1; ...
    setpoints2]);



%ini_veh_quat = single([0, 0, .7071, .7071]);

%localization mode commanding
mlp_loc_mode_cmd_times      = uint32([  0 , 0; ...                          %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
    100 , 0; ...
    1300 , 0]);
mlp_loc_mode_cmd_times      = [mlp_loc_mode_cmd_times(:,1)+ini_time_seconds, mlp_loc_mode_cmd_times(:,2)+ini_time_nanoseconds];

mlp_loc_mode_cmd_list       = uint8([0; 0; 0]);                             %[] Mode Commands to be sent at specified times.

mlp_mode_cmd_list = uint8([2;2;2]);
mlp_mode_cmd_times = uint32([  0 , 0; ...                          %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
    60 , 0; ...
    120 , 0]);
mlp_mode_cmd_times = [mlp_mode_cmd_times(:,1)+ini_time_seconds, mlp_mode_cmd_times(:,2)+ini_time_nanoseconds];




astrobee_stop_time = 150;
