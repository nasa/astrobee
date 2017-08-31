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

function [setpoint, cmd_times] = calc_trapazoidal_waypoints(init_pos, cmd_accel, cmd_vel, init_t)
%% [pos_x,     pos_y,     pos_z,     vel_x,     vel_y,    vel_z,     acc_x,     acc_y,     acc_z,   quat_x,   quat_y,   quat_z,    quat_w,   angv_x,   angv_y,   angv_z,    anga_x,    anga_y,    anga_z]
% Currently only outputs dummy attitude
init_quat = [0 0 0 1];
init_omega = [0 0 0];
init_alpha = [0 0 0];


% Currently velocity must be vector and accel has to be scalar
init_vel = [0 0 0];
delta_vel_mag = norm(cmd_vel - init_vel, 'fro');
boost_end_t = delta_vel_mag / cmd_accel;
vel_unit_vec = (cmd_vel - init_vel) ./ delta_vel_mag;
accel_vec = vel_unit_vec*cmd_accel;
boost_end_pos = init_pos + init_vel*boost_end_t + .5*accel_vec*boost_end_t.^2;

setpoint(1,:) = [init_pos, init_vel, accel_vec, init_quat, init_omega, init_alpha];
setpoint(2,:) = [boost_end_pos, cmd_vel, [0 0 0], init_quat, init_omega, init_alpha];

% time are seconds and nanoseconds
abs_boost_end_t = boost_end_t+init_t;

cmd_times = [init_t 0; floor(abs_boost_end_t) mod(abs_boost_end_t,1)*1E9];

