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

% Test file to stimulate the FAM
%
%
%% Dependency Prep
% Init model parameters
astrobee_init;


fam_nozzle_discharge_coeff  = single(0.675*ones(12,1));                             %[-] Nozzle Discharge coefficient.  Determined via Test
% fam_nozzle_discharge_coeff  = single(0.6852*ones(12,1));                             %[-] Nozzle Discharge coefficient.  Determined via Test


astrobee_prep;

sim_stop_time = 150;

%% Input Prep
test_impeller_speed_idx.signals.dimensions  = 1;
test_impeller_speed_idx.time                = [0 70];
test_impeller_speed_idx.signals.values      = single([2; 2;]);

test_cmd_force.signals.dimensions       = 3;
test_cmd_force.time                     = [0 10 20 30 40 50 60 70 80 90 100 110];
test_cmd_force.signals.values           = single([ 0, 0.0, 0; ...
                                                   0, 0.5, 0; ...
                                                   0, 2.5, 0; ...
                                                   0, 3.0, 0; ...
                                                   0, 6.0, 0; ...
                                                   0, 11.5, 0; ...
                                                   0, 15.5, 0; ...
                                                   0, 18.5, 0; ...
                                                   0, 20.5, 0; ...
                                                   0, 22.5, 0; ...
                                                   0, 22.5, 0; ...
                                                   0, 22.5, 0]*0.0098);

test_cmd_torque.signals.dimensions      = 3;
test_cmd_torque.time                    = [0 10 20 30 40 50];
test_cmd_torque.signals.values          = single([0, 0, 0; ...
                                                  0, 0, 0; ...
                                                  0, 0, 0; ...
                                                  0, 0, 0; ...
                                                  0, 0, 0; ...
                                                  0, 0, 0]);


%% run sim
sim('fam_force_allocation_module_hrn');

%% Output Display
close all;

figure; plot(out_act_msg.act_impeller_speed_cmd);
title('commanded impeller speed'); xlabel('time'); ylabel('speed');

figure; plot(out_act_msg.act_servo_pwm_cmd);
title('commanded servo PWM'); xlabel('time'); ylabel('PWM'); legend('1','2','3','4','5','6','7','8','9','10','11','12');


