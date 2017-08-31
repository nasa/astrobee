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

% Astrobee Augmented State Estimator (ASE) prep file.  Configures
% parameters used in the library file augmented_state_estimator.
%
%% Initial Conditions

ase_total_num_states = double((15 + 6 + 6 * ase_of_num_aug.Value));
ase_state_ic_cov_diag = ones(ase_total_num_states,1, 'single');

ase_state_ic_of_quat_ISS2cam = single(repmat([0 0 0 1],ase_of_num_aug.Value, 1));
ase_state_ic_of_P_cam_ISS_ISS = zeros(ase_of_num_aug.Value,3, 'single');

 %% Indices
ase_of_p_indx = 22:15 + 6 + 6 * ase_of_num_aug.Value;
ase_diag_indx = (1:ase_total_num_states:ase_total_num_states^2)+(0:ase_total_num_states-1);

