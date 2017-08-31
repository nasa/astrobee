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

%% mlp_mid_level_processor_init.m

mlp_dummy_state_cmd = single(zeros(1,19));

%capture command plan
if isunix
    [mlp_command_list, mlp_command_times] = gen_cmd_list([ASTROBEE_ROOT '/sim_model/mlp_mid_level_processor_model/cmc_command_and_mode_control/scenario2.fplan']);
else
    [mlp_command_list, mlp_command_times] = gen_cmd_list([ASTROBEE_ROOT '\sim_model\mlp_mid_level_processor_model\cmc_command_and_mode_control\scenario2.fplan']);    
end

%mlp_command_list  = single[pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, quat_x, quat_y, quat_z,quat_w, angv_x, angv_y, angv_z, anga_x, anga_y, anga_z]
%mlp_command_times = uint32[time_sec, time_nsec]


% BELOW COMMENTED OUT: is an alternative way to specify a command list
% mlp_command_times = [
%     %           time_sec,                  time_nsec
% 	    ini_time_seconds+0,   ini_time_nanoseconds+0; ...
% 	    ini_time_seconds+1,   ini_time_nanoseconds+0; ...
%       ini_time_seconds+2,   ini_time_nanoseconds+14; ...
%      ini_time_seconds+49,   ini_time_nanoseconds+332179715; ...
%      ini_time_seconds+50,   ini_time_nanoseconds+332179715; ...
%      ini_time_seconds+60,	  ini_time_nanoseconds+332179715; ...
%      ini_time_seconds+61,   ini_time_nanoseconds+332179715; ...
%     ini_time_seconds+100,   ini_time_nanoseconds+332179715; ...
%     ini_time_seconds+101,   ini_time_nanoseconds+332179715;
%     ini_time_seconds+150,   ini_time_nanoseconds+332179715];
% mlp_command_times = uint32(mlp_command_times);
% 
% mlp_command_list = [ ...
%     %   pos_x,     pos_y,     pos_z,     vel_x,     vel_y,    vel_z,     acc_x,     acc_y,     acc_z,   quat_x,   quat_y,   quat_z,    quat_w,   angv_x,   angv_y,   angv_z,    anga_x,    anga_y,    anga_z
%      0.000000,  0.000000,  0.000000,  0.000000,  0.000000, 0.000000,  0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%      0.000000,  0.000000,  0.000000,  0.000000,  0.000000, 0.000000, -0.045518,  0.020690,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%     -0.022760,  0.010346,  0.000000, -0.045518,  0.020690, 0.000000,  0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%     -2.177241,  0.989655,  0.000000, -0.045518,  0.020690, 0.000000,  0.045518, -0.020690,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%     -2.200000,  1.000000,  0.000000,  0.000000,  0.000000, 0.000000,  0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%     -2.200000,  1.000000,  0.000000,  0.000000,  0.000000, 0.000000,  0.000000,  0.050000,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%     -2.200000,  1.025001,  0.000000,  0.000000,  0.050000, 0.000000,  0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%     -2.200000,  2.975001,  0.000000,  0.000000,  0.050000, 0.000000,  0.000000, -0.050000,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000; ...
%     -2.200000,  3.000001,  0.000000,  0.000000,  0.000000, 0.000000,  0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000;
%      -2.200000,  3.000001,  0.000000,  0.000000,  0.000000, 0.000000,  0.000000,  0.000000,  0.000000, 0.000000, 0.000000, 0.707000,  0.707000, 0.000000, 0.000000, 0.000000,  0.000000,  0.000000,  0.000000];
% mlp_command_list = single(mlp_command_list);


%below is Jesse's hack to put in two extra commands before the rest
mlp_command_times = [mlp_command_times(:,1)+150 mlp_command_times(:,2)];
mlp_command_times = [ini_time_seconds, ini_time_nanoseconds; ...
                    ini_time_seconds+30, ini_time_nanoseconds; 
                    mlp_command_times];

mlp_command_list = [ ...
    0,0,0, 0,0,0, 0,0,0, 0,0,0,1, 0,0,0, 0,0,0;
    0,0,0, 0,0,0, 0,0,0, 0,0,0,1, 0,0,0, 0,0,0;
    mlp_command_list];


%Mode commanding 
mlp_mode_cmd_times          = uint32([  0 , 0; ...                          %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
                              1000 , 0; ...
                              1300 , 0]);
mlp_mode_cmd_times          = [mlp_mode_cmd_times(:,1)+ini_time_seconds, mlp_mode_cmd_times(:,2)+ini_time_nanoseconds];

mlp_mode_cmd_list           = uint8([2; 2; 2]);                             %[] Mode Commands to be sent at specified times.
%2=nominal, 1 = stopped, 0 = idle

%Speed Gain Commanding
mlp_speed_gain_cmd_times    = uint32([  0 , 0; ...                          %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
                              1000 , 0; ...
                              1300 , 0]);
mlp_speed_gain_cmd_times    = [mlp_speed_gain_cmd_times(:,1)+ini_time_seconds, mlp_speed_gain_cmd_times(:,2)+ini_time_nanoseconds];

mlp_speed_gain_cmd_list     = uint8([2; 2; 2]);                             %[] Mode Commands to be sent at specified times.


%localization mode commanding
mlp_loc_mode_cmd_times      = uint32([  0 , 0; ...                          %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
                              1000 , 0; ...
                              1300 , 0]);
mlp_loc_mode_cmd_times      = [mlp_loc_mode_cmd_times(:,1)+ini_time_seconds, mlp_loc_mode_cmd_times(:,2)+ini_time_nanoseconds];

mlp_loc_mode_cmd_list       = uint8([0; 0; 0]);                             %[] Mode Commands to be sent at specified times.
%bit field: [7:x,x,x,x,3:handrail,2:docking,1:x,0:power]

