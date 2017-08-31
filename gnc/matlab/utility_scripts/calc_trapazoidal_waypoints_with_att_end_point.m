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

function [setpoint, cmd_times] = calc_trapazoidal_waypoints_with_att_end_point(init_pos, end_pos, cmd_accel, cmd_vel, init_quat,end_quat, cmd_alpha, cmd_omega, init_t)
%% [pos_x,     pos_y,     pos_z,     vel_x,     vel_y,    vel_z,     acc_x,     acc_y,     acc_z,   quat_x,   quat_y,   quat_z,    quat_w,   angv_x,   angv_y,   angv_z,    anga_x,    anga_y,    anga_z]
% Attitude can only be used if there is a zero position change command
% Attitude propagation uses small angle approximation
% Currently only uses times from position

% Currently velocity must be vector and accel has to be scalar
init_vel = 0; % Only works for initial velocity  = 0, right now.
no_cruise_flag = 0;
pos_stationary_flag = 0;
delta_vel_mag = norm(cmd_vel - init_vel, 'fro');
if delta_vel_mag == 0 || all(init_pos == end_pos)
    pos_stationary_flag = 1;
    boost_t = 0;
    accel_vec = [0 0 0];
    boost_end_pos = init_pos;
else
    boost_t = delta_vel_mag / cmd_accel;
    pos_delta = end_pos - init_pos;
    dir_unit_vec = pos_delta./ norm(pos_delta);
    accel_vec = dir_unit_vec*cmd_accel;
    boost_pos_delta = init_vel*boost_t + .5*accel_vec*boost_t.^2;
    boost_end_pos = init_pos + boost_pos_delta;
    
    deboost_start_pos = end_pos - boost_pos_delta;
    
    cruise_vec = deboost_start_pos - boost_end_pos;
    cruise_time = norm(cruise_vec) / cmd_vel;
    
    
    deboost_t = boost_t;
    
    
    % Case where you never reach full speed
    if any(((deboost_start_pos - boost_end_pos).*sign(dir_unit_vec)) < 0) % Test if you will ever reach full speed
        no_cruise_flag = 1;
        % If not there will never be a cruise phase
        boost_pos_delta = pos_delta / 2;
        boost_end_pos = init_pos + boost_pos_delta;
        deboost_start_pos = end_pos - boost_pos_delta;
        t_roots = roots([.5*cmd_accel init_vel -norm(boost_pos_delta)]);
        boost_t = t_roots(find(t_roots > 0,1, 'first'));
        deboost_t = boost_t;
        cruise_time = 0;
        
        cmd_vel = boost_t*cmd_accel; % Replace command velocity, with achieved velocity
    end
    
    setpoint(1,:) = [init_pos, [0 0 0], accel_vec, init_quat, [0 0 0], [0 0 0]]; % Boost
    setpoint(2,:) = [boost_end_pos, cmd_vel*dir_unit_vec, [0 0 0], init_quat, [0 0 0], [0 0 0]]; % Cruise
    setpoint(3,:) = [deboost_start_pos, cmd_vel*dir_unit_vec, -accel_vec, init_quat, [0 0 0], [0 0 0]]; % Deboost
    setpoint(4,:) = [end_pos, [0 0 0], [0 0 0], end_quat, [0 0 0], [0 0 0]]; % Hold
    
    % time are seconds and nanoseconds
    abs_boost_end_t = boost_t+init_t;
    abs_cruise_end_t = abs_boost_end_t + cruise_time;
    abs_deboost_end_t = abs_cruise_end_t + deboost_t;
    
    cmd_times = [init_t 0;
        floor(abs_boost_end_t), mod(abs_boost_end_t,1)*1E9;...
        floor(abs_cruise_end_t), mod(abs_cruise_end_t,1)*1E9;...
        floor(abs_deboost_end_t), mod(abs_deboost_end_t,1)*1E9;...
        ];
    
    
    if no_cruise_flag
        setpoint(2,:) = []; % Delete cruise setpoint
        cmd_times(2,:) = [];
    
    end

    
    
end

%% Attitude Portion
% init_quat = [.7071 .7071 0 0] % 90 deg yaw, followed by 180 deg roll
% end_quat = [0 0 0 1] % Null quat, axes aligned with nav frame
if pos_stationary_flag
    % Currently velocity must be vector and accel has to be scalar
    init_omega = 0; % Only works for initial velocity  = 0, right now.
    no_cruise_flag = 0;
    delta_omega_mag = norm(cmd_omega - init_omega, 'fro');
    if delta_omega_mag == 0 || all(init_quat == end_quat)
        boost_t = 0;
        alpha_vec = [0 0 0];
        boost_end_quat = init_quat;
        delta_vec = [1 0 0];
        deboost_start_quat = init_quat;
    else
        boost_t = delta_omega_mag / cmd_alpha;
        q_delta = quatmult( quat_inv( init_quat ), end_quat );
        
        if  ~(q_delta(4) == 0)
            q_delta = sign(q_delta(4))*q_delta; % Ensure the shortest route is taken.
        end
        delta_angle = 2*acos(q_delta(4));
        delta_vec = q_delta(1:3) / sqrt(1 - q_delta(4)*q_delta(4));
        
        alpha_vec = delta_vec*cmd_alpha;
        boost_angle_delta = init_omega*boost_t + .5*cmd_alpha*boost_t.^2;
        body_accel_rate = cmd_alpha*delta_vec;
        boost_end_quat = quatmult( init_quat,  axis_angle_to_quat(delta_vec./norm(delta_vec), boost_angle_delta) ); % Could have done q_init * delta_quat^-1
        
        deboost_start_quat = quatmult( end_quat, axis_angle_to_quat(delta_vec./norm(delta_vec), -boost_angle_delta) );
        deboost_t = boost_t;
        
        q_delta_cruise = quatmult( quat_inv( boost_end_quat ), deboost_start_quat );
        delta_cruise_angle = 2*acos(q_delta_cruise(4));
        delta_cruise_vec = q_delta_cruise(1:3) / sqrt(1 - q_delta_cruise(4)*q_delta_cruise(4));
        cruise_body_rates = cmd_omega * delta_cruise_vec;
        cruise_time = delta_cruise_angle / cmd_omega;
        
        
        % Case where you never reach full speed
        % Look to make sure the resultant axis from the end of boost to the
        % start of de-boost is aligned with the initial axis of rotation and
        % that the angles are of the same sign.  This is a good check because I
        % calculate deboost_start_q as (end quat - angle change while
        % boosting), and we check to make sure that this has not overlapped
        % with boost_end_q (i.e. we would need to start slowing down before we
        % ever reached full speed).
        if ~(norm(delta_vec - delta_cruise_vec) < 1E-4 && sign(delta_cruise_angle) == sign(delta_angle))
            no_cruise_flag = 1;
            % If not there will never be a cruise phase
            boost_angle_delta = delta_angle / 2; % Assume we accelerate for half the time, then decel for the other half
            boost_end_quat = quatmult( init_quat,  axis_angle_to_quat(delta_vec./norm(delta_vec), boost_angle_delta) ); % Could have done q_init * delta_quat^-1
            deboost_start_quat = quatmult( end_quat, axis_angle_to_quat(delta_vec./norm(delta_vec), -boost_angle_delta) );
            
            t_roots = roots([.5*cmd_alpha init_omega -norm(boost_angle_delta)]);
            boost_t = t_roots(find(t_roots > 0,1, 'first'));
            deboost_t = boost_t;
            cruise_time = 0;
            
            cmd_omega = boost_t*cmd_alpha; % Replace command velocity, with achieved velocity
        end
        
    end
    
        setpoint(1,:) = [init_pos, [0 0 0], [0 0 0], init_quat, [0 0 0], alpha_vec]; % Boost
    setpoint(2,:) = [init_pos, [0 0 0], [0 0 0], boost_end_quat, delta_vec*cmd_omega, [0 0 0]]; % Cruise
    setpoint(3,:) = [init_pos, [0 0 0], [0 0 0], deboost_start_quat, delta_vec*cmd_omega, -alpha_vec]; % Deboost
    setpoint(4,:) = [end_pos, [0 0 0], [0 0 0], end_quat, [0 0 0], [0 0 0]]; % Hold
    
    % time are seconds and nanoseconds
    abs_boost_end_t = boost_t+init_t;
    abs_cruise_end_t = abs_boost_end_t + cruise_time;
    abs_deboost_end_t = abs_cruise_end_t + deboost_t;
    
    cmd_times = [init_t 0;
        floor(abs_boost_end_t), mod(abs_boost_end_t,1)*1E9;...
        floor(abs_cruise_end_t), mod(abs_cruise_end_t,1)*1E9;...
        floor(abs_deboost_end_t), mod(abs_deboost_end_t,1)*1E9;...
        ];
    
    
    if no_cruise_flag
        setpoint(2,:) = []; % Delete cruise setpoint
        cmd_times(2,:) = [];
    end    
    
end
