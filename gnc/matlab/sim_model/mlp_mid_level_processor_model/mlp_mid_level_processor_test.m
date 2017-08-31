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

% Simple Unit Test for the mid level processor

%% init
disp('initializing...')
astrobee_init;
mlp_stop_time = 300;
cvs_noise_on            = boolean(1);                                      %[flag] 0=no position noise, 1=noise!

mlp_loc_mode_cmd_times  = uint32([  0 , 0; ...                      %[sec] Sim time to set the current mode command to the cooresponding value in the mode command list
                              100 , 0; ...
                              200 , 0]);
mlp_loc_mode_cmd_times = [mlp_loc_mode_cmd_times(:,1)+ini_time_seconds, mlp_loc_mode_cmd_times(:,2)+ini_time_nanoseconds];

mlp_loc_mode_cmd_list   = uint8([1; 5; 9]);                         %[] Mode Commands to be sent at specified times.
%1=power, 9=power&handrail, 5 = power&ARtags
astrobee_prep;

%% Configure Dummy Waypoints
wpts_t_lin  = [10 30 60]+60;
wpts_lin    = [0 0 0;
               0 0 0;
               0 0 0];
% wpts_lin    = [1 0 0;
%                1 1 0;
%                1 1 1];

wpts_t_ang  = [1 2 100 200 250];
wpts_ang    = [0 0 0;
               0 0 -0.0157;
               0 0 +0.0157;
               0 0 0;
               0 0 0];
% wpts_ang    = [0.15 0 0;
%                0 0 0;
%                -0.15 0 0
%                0 0 0;
%                0 0.15 0;
%                0 0 0;
%                0 -0.15 0;
%                0 0 0.15 ;
%                0 0 0;
%                0 0 -0.15;
%                0 0 0;
%                ];

%%
disp('running sim...')
sim('mlp_mid_level_processor_hrn');           
           
%% Plot data
disp('plotting...')
close all;

figure; plot(out_cmc_msg.cmc_mode_cmd)


figure; plot(out_cvs_landmark_msg.cvs_valid_flag.Time, squeeze(sum(out_cvs_landmark_msg.cvs_valid_flag.Data,2)));
title('Total landmarks in view');

figure; subplot(3,1,1); plot(out_cvs_reg_pulse.cvs_landmark_pulse);
subplot(3,1,2); plot(out_cvs_reg_pulse.cvs_optical_flow_pulse);
subplot(3,1,3); plot(out_cvs_reg_pulse.cvs_handrail_pulse);

figure; subplot(3,1,1); plot(out_cvs_landmark_msg.cvs_landmarks.Time, squeeze(out_cvs_landmark_msg.cvs_landmarks.Data(:,1,:))')
title('landmarks');
subplot(3,1,2); plot(out_cvs_landmark_msg.cvs_landmarks.Time, squeeze(out_cvs_landmark_msg.cvs_landmarks.Data(:,2,:))')
subplot(3,1,3); plot(out_cvs_landmark_msg.cvs_landmarks.Time, squeeze(out_cvs_landmark_msg.cvs_landmarks.Data(:,3,:))')

figure; subplot(2,1,1); plot(out_cvs_landmark_msg.cvs_observations.Time, squeeze(out_cvs_landmark_msg.cvs_observations.Data(:,1,:))');
title('observations');
subplot(2,1,2); plot(out_cvs_landmark_msg.cvs_observations.Time, squeeze(out_cvs_landmark_msg.cvs_observations.Data(:,2,:))');

figure; plot(out_cvs_opticalflow_msg.cvs_valid_flag.Time, squeeze(sum(squeeze(out_cvs_opticalflow_msg.cvs_valid_flag.Data(:,1,:)),1)));
title('Total OF points in view');

figure; subplot(2,1,1); plot(out_cvs_opticalflow_msg.cvs_observations.Time, squeeze(out_cvs_opticalflow_msg.cvs_observations.Data(:,1,1,:))');
title('observations');
subplot(2,1,2); plot(out_cvs_opticalflow_msg.cvs_observations.Time, squeeze(out_cvs_opticalflow_msg.cvs_observations.Data(:,2,1,:))');

figure; plot(out_cvs_handrail_msg.cvs_valid_flag.Time, squeeze(sum(out_cvs_handrail_msg.cvs_valid_flag.Data,2)));
title('Total handrail points in view');

figure; plot(out_cvs_handrail_msg.cvs_handrail_vector);
title('handrail vector');

figure; subplot(3,1,1); plot(out_cvs_handrail_msg.cvs_landmarks.Time, squeeze(out_cvs_handrail_msg.cvs_landmarks.Data(:,1,:))')
title('handrail - landmarks');
subplot(3,1,2); plot(out_cvs_handrail_msg.cvs_landmarks.Time, squeeze(out_cvs_handrail_msg.cvs_landmarks.Data(:,2,:))')
subplot(3,1,3); plot(out_cvs_handrail_msg.cvs_landmarks.Time, squeeze(out_cvs_handrail_msg.cvs_landmarks.Data(:,3,:))')

figure; subplot(3,1,1); plot(out_cvs_handrail_msg.cvs_observations.Time, squeeze(out_cvs_handrail_msg.cvs_observations.Data(:,1,:))');
title('handrail - observations');
subplot(3,1,2); plot(out_cvs_handrail_msg.cvs_observations.Time, squeeze(out_cvs_handrail_msg.cvs_observations.Data(:,2,:))');
subplot(3,1,3); plot(out_cvs_handrail_msg.cvs_observations.Time, squeeze(out_cvs_handrail_msg.cvs_observations.Data(:,3,:))');

%% save data
% save('comp_vis_model_out.mat', 'cvs_truth_fixed_points_iss', 'cvs_truth_flow_points_iss', 'cvs_flow_ids', 'out_cvs_landmark_msg', 'out_cvs_opticalflow_msg', 'out_cvs_reg_pulse')


